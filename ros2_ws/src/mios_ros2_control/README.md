# MIOS ROS 2 control bridge

`MiosEffortController` and `MiosJointVelocityController` are migration
boundaries. They claim, respectively, the seven FR3 effort or seven FR3 joint
velocity interfaces from `franka_hardware`. Neither creates a `franka::Robot`
and therefore neither can compete with `franka_ros2` for the FCI.

## Real-time MIOS control core

`MiosControlCore` now runs in every controller-manager `update()` cycle. It
uses transport-neutral MIOS joint state data and applies finite-state checks,
per-joint torque saturation, torque-rate limiting, stale-command timeout, and
immediate zero torque on stale/invalid input or on Franka's direct user-stop
state.

By default, the controller claims only joint position, velocity, effort, and
the seven effort command interfaces. It claims Franka's ROS-owned
`${arm_id}/robot_model` and `${arm_id}/robot_state` semantic interfaces only
when a Cartesian MIOS safety term is enabled, then reads only the Jacobian or
pose that term needs. It never creates a direct FCI connection.

Robot mode is intentionally not converted into a full ROS message in the
1 kHz update loop. A normal best-effort subscription receives the robot-state
topic and writes only a timestamp and user-stop bit into a real-time buffer.
A missing, stale, or user-stopped safety state forces zero torque.

The ported deterministic safety subset is evaluated in this update loop: joint
virtual walls, Cartesian velocity damping, and an arming virtual workspace.
The Cartesian terms are disabled with zero limits in the shipped configuration
and require robot-specific commissioning before use.

At activation, the controller requires a fresh, stationary robot-state message
in Franka's `IDLE` or `MOVE` mode and validates the same joint state directly
from hardware. Guiding, reflex, user-stop, and recovery modes are fail-safe.
The shipped commissioning baseline writes exactly zero *external* torque, the
same output as Franka's official gravity-compensation controller; the Franka
master controller compensates the robot's weight. This is not a rigid position
hold and is separately locked with `allow_zero_effort_activation: false`.
Physical effort activation is also locked by default with
`allow_effort_activation: false`. Both locks must remain false until their
specific hardware tests are approved. The local spring/damper position hold is
disabled by default and needs its own commissioning before it can be enabled.
Raw `desired_effort` input is disabled by default with
`allow_external_effort_commands: false`. The original MIOS task/skill engine is
not called from `update()` yet because it currently does database, portal,
logging, and allocation work that is not real-time safe.

The implemented actuator modes cover the joint-impedance fields used by the
legacy joint-torque pipeline, plus a base-frame Cartesian impedance snapshot
using Franka's ROS-owned zero Jacobian and optional Coriolis vector. Cartesian
commands are independently locked with
`allow_runtime_cartesian_actuator_commands: false`; when that gate is enabled,
the controller claims the necessary model semantic interfaces. Desired-wrench
adaptation and null-space pipelines are still separate migration work; they
must not be represented as an effort command until their semantic interfaces
and tests are implemented.

## Cartesian-force mode

`MiosEffortController` now accepts `MODE_CARTESIAN_FORCE` as the ROS boundary
for the legacy desired-wrench path. The target uses the base-frame
`[force xyz, torque xyz]` convention of Franka's `O_F_ext_hat_K` measurement.
The controller evaluates a bounded PID correction at 1 kHz. The derivative is
filtered per axis, then its wrench slew is limited and mapped through Franka's
zero Jacobian before the same global torque limits and local hold used by every
effort request.

This is not enabled by the existing effort lock alone: both
`allow_runtime_cartesian_force_commands` and `cartesian_force_enabled` must
be commissioned. All supplied gains and wrench limits are zero, so this stage
is inert by default. Contact-frame/sign validation and hardware commissioning
are mandatory before either setting is changed.

## Cartesian impedance adaptation

The Cartesian-impedance path also contains an optional contact-aware stiffness
stage. It raises each requested stiffness in proportion to the measured
base-frame external-wrench magnitude, clamps it to configured bounds, and
slew-limits the change before the normal impedance wrench is evaluated. It is
disabled by `cartesian_impedance_adaptation_enabled: false`; all contact gains
are zero in the supplied configuration.

## Null-space mode

`MODE_NULLSPACE` applies a joint impedance target only through the damped
kinematic null space of Franka's zero Jacobian. The controller uses
`N = I - Jᵀ(JJᵀ + λ²I)⁻¹J`, caps the candidate and projected joint torques,
and then sends it through the normal effort safety path. It is separate from
Cartesian impedance and force requests; the newest high-level request wins.
`allow_runtime_nullspace_commands` and `nullspace_enabled` are both false, and
the supplied null-space torque limits are zero.

## Joint-velocity mode

`MiosJointVelocityController` is the ROS implementation boundary for the
legacy `JointVelocityControllerPipeline`, which outputs `Actuator::dq_d`.
It accepts only `MODE_JOINT_VELOCITY`, uses a fixed-size command, enforces
commissioning velocity and acceleration caps, requires a fresh `IDLE` or
`MOVE` robot state, and clears its output immediately for stale/invalid/user
stop input. It reads live position as well as velocity and refuses an outward
velocity command inside a configured 0.10 rad joint-limit margin; a command
back toward the valid range is still accepted. It claims velocity command
interfaces, so it must be switched by controller manager instead of being activated alongside
`MiosEffortController`.

Both `allow_velocity_activation` and
`allow_runtime_joint_velocity_commands` are false in the supplied YAML. The
commissioning caps are 0.05 rad/s and 0.10 rad/s²—not FR3 limits.

## Cartesian-velocity mode

`MiosCartesianVelocityController` is the ROS implementation boundary for the
legacy `CartesianVelocityControllerPipeline`, which outputs
`Actuator::TF_dX_d`. It claims Franka's six native Cartesian velocity
interfaces (`vx`, `vy`, `vz`, `wx`, `wy`, and `wz`) and receives
`MODE_CARTESIAN_VELOCITY` in `[linear xyz, angular xyz]` order. It verifies
fresh `IDLE`/`MOVE` state, joint stationarity before switching, finite input,
per-component velocity/acceleration limits, and immediate zero output for
stale or stop input.

It is a separate controller-manager mode: do not activate it together with
effort/impedance or joint velocity. The shipped gates
`allow_cartesian_velocity_activation` and
`allow_runtime_cartesian_velocity_commands` are both false. Its limits are
commissioning values (0.01 m/s linear and 0.10 rad/s angular), not FR3 limits.

## Joint-position mode

`MiosJointPositionController` is the ROS implementation boundary for the
legacy MIOS joint-position actuator output. It accepts only
`MODE_JOINT_POSITION`, claims the seven native joint position interfaces, and
is a separate controller-manager mode. At activation it writes Franka's
current desired joint reference (`q_d`), not a default, zero, or lagging
measured target. Stale, invalid, or stop input preserves that reference in the
same cycle. Each requested
target is clamped to the configured FR3 joint range and rate-limited to the
commissioning cap of 0.05 rad/s.

Both `allow_position_activation` and
`allow_runtime_joint_position_commands` are false in the supplied YAML. It
must not be activated with effort/impedance, joint velocity, or Cartesian
velocity controllers.

## Post-HandGuiding bounded effort hold

`mios_effort_hold_controller` is an opt-in, second instance of
`MiosEffortController` intended for the interval after a HandGuiding pose is
confirmed. It claims the same seven effort interfaces as
`mios_effort_controller`; controller manager switches one effort controller
off while activating the other. Consequently, it does not start Franka's
native joint-position motion generator at the compliant release pose.

On activation it requires a fresh stationary state, captures the live measured
joint pose, and applies a local spring/damper only within the supplied
per-joint added-torque limits. Franka gravity compensation remains the master
controller's responsibility. The shipped supervised commissioning tune is
1.0 Nm for joints 1–3, 2.0 Nm for joint 4, and 0.5 Nm for joints 5–7, rather
than robot torque limits; all external and runtime command sources remain
disabled on the hold controller.

The controller is present in the YAML but is inert and cannot be activated
because `allow_effort_activation: false`. It must be mock-tested and then
commissioned as a supervised non-zero-torque feature before that gate is
changed. Never enable its gate merely to recover from a failed switch or a
Franka reflex. The teaching script's `--hold-between-handguiding` option
preflights both effort controllers and atomically switches between them; it
does not load controllers, change parameters, or bypass their lifecycle
checks.

During supervised hold commissioning, the supplied hold controller publishes
`/mios_effort_hold_controller/effort_diagnostic` at 20 Hz. Its
`Float64MultiArray.data` contains five consecutive groups of seven values:
commanded effort, measured effort, joint position, joint velocity, and the
pose captured as the hold reference. The non-blocking publisher drops a frame
if necessary and never participates in the 1 kHz control path. Set
`effort_diagnostic_publish_rate: 0.0` to disable it.

## Cartesian-pose mode

`MiosCartesianPoseController` is the ROS implementation boundary for legacy
`ArmCommand::kCartesianPose` output. It claims Franka's sixteen native
`0/cartesian_pose_command` through `15/cartesian_pose_command` interfaces and
receives `MODE_CARTESIAN_POSE` as a column-major, base-to-end-effector 4×4
transform. It validates a finite right-handed rigid transform, clamps only its
translation to the configured commissioning workspace, and rate-limits linear
and angular motion independently.

Activation requires a fresh stationary `IDLE`/`MOVE` state and sends the live
end-effector pose. Stale, invalid, and user-stop commands send that live pose
again rather than a default matrix. `allow_cartesian_pose_activation` and
`allow_runtime_cartesian_pose_commands` are both false in the supplied YAML.
Do not activate this mode with effort/impedance, joint-position, joint-
velocity, or Cartesian-velocity controllers.

## ROS-only MIOS runtime boundary

`mios_ros2_runtime` contains `Ros2RobotBackend`, a non-real-time transport
backend that owns no `franka::Robot` or FCI connection. It subscribes to
`/franka_robot_state_broadcaster/robot_state` and converts the complete Franka
state message into a neutral snapshot with joint/motor state, external
torque/wrenches, end-effector pose, timestamp, and user-stop state.

The runtime publishes a fixed-size `mios_msgs/msg/MiosEffortCommand` on
`/mios_effort_controller/mios_effort_command` only for the temporary raw-torque
bridge. It is locked by `allow_runtime_effort_commands: false`.

The MIOS migration path is `mios_msgs/msg/MiosActuatorCommand` on
`/mios_effort_controller/mios_actuator_command`. Its first supported mode is a
joint-impedance actuator snapshot: target position/velocity, feed-forward
torque, stiffness, and damping. `MiosEffortController` evaluates it from live
joint state in its 1 kHz update loop, before applying the same safety,
saturation, and torque-rate limits as every other source. A fresh actuator
snapshot takes precedence over raw torque; sources are never summed. It is
also locked by default with `allow_runtime_actuator_commands: false`, and the
runtime node publishes no actuator command by default.

The same runtime backend also has explicit typed publishers for the separate
joint-velocity, Cartesian-velocity, joint-position, and Cartesian-pose
controller topics. The controller gates remain false, so constructing the
runtime does not command or move hardware.

This boundary does not make the legacy MIOS `Core` real-time safe. The
ROS-owned `mios_ros2_core_runtime` process keeps database, portal, task
scheduling, and logging outside the 1 kHz controller loop; its Core scheduler
and command path are disabled by default. The controller/safety computation
still requires commissioning before it can be allowed to publish commands.

When a ROS controller claims a Franka command interface, Desk reports robot
mode `MOVE` even if that controller is publishing only its fail-safe zero
command. The legacy task engine originally rejected that state because it
owned FCI control itself. `mios_ros2_core_runtime` therefore keeps
`allow_controller_owned_move_mode:=false` by default. A physical Core task
requires all three independent gates: `enable_core_scheduler:=true`,
`enable_core_task_execution:=true`, and
`allow_controller_owned_move_mode:=true`, plus the selected controller's own
activation and runtime-command gates. Enabling the third gate does not send a
command; it only permits an already commissioned ROS controller-owned `MOVE`
state to pass the legacy task precheck.

For a supervised scheduler-load diagnostic with task dispatch still disabled,
the runtime has a fourth, fail-closed gate:
`allow_controller_owned_move_mode_without_task_execution:=true`. It is useful
only after the controller-owned zero-command state has been commissioned. The
Core backend still rejects every arm and gripper request while
`enable_core_task_execution:=false`; this diagnostic gate merely prevents the
legacy scheduler from treating ROS-owned `MOVE` as an unexpected state. It is
not required, and must remain false, for normal idle and physical-task runs.

## Running the FR3 hardware stack in Docker

The `franka_hardware` plugin uses a real-time thread. Run the image with
`CAP_SYS_NICE`, an RT-priority limit, and an unlimited memory-lock limit:

```bash
docker run --rm -it --network host \
  --cap-add SYS_NICE \
  --ulimit rtprio=99 \
  --ulimit memlock=-1:-1 \
  mios-ros2-jazzy:local bash
```

`mios/docker/ros2/docker-compose.yml` supplies the equivalent settings. On the
NUC host, verify that the robot is reachable before launching: `ping` and
`nc -vz -w 3 <robot-ip> 1337` must both succeed. Do not run a legacy MIOS
process that creates a direct `franka::Robot` at the same time.

When explicitly enabled for bench testing, it accepts a seven-element
`std_msgs/msg/Float64MultiArray` on
`/mios_effort_controller/desired_effort`. Values are torque commands without
gravity compensation, as required by the Franka effort interface. The supplied
limits are deliberately commissioning-only (2 Nm and 10 Nm/s); they are not
robot limits.

Add `config/mios_controllers.yaml` to the controller-manager parameters used by
your `franka_bringup` launch, then load the controller inactive. Verify its
claimed interfaces and parameter values before activating it:

```bash
ros2 control load_controller mios_effort_controller
ros2 control list_controllers
ros2 control set_controller_state mios_effort_controller active
```

Do not activate this controller on hardware after a Franka Desk reflex or
speed-limit event until the error has been investigated, the changed image has
passed the Docker mock-hardware lifecycle test, and the exact zero-effort mode
has been separately approved for a supervised test. Do not publish non-zero
commands until the non-real-time MIOS runtime supplies command snapshots and
the complete pipeline has passed robot commissioning.

## Offline build check

The Docker image can be checked without a robot connection:

```bash
docker run --rm --network none --entrypoint bash mios-ros2-jazzy:local -lc '
  source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
  ros2 interface show mios_msgs/msg/MiosActuatorCommand
  timeout 2 ros2 run mios_ros2_runtime mios_runtime_node
  test $? -eq 124
  timeout 2 /opt/mios/install/bin/mios_ros2_core_runtime
  test $? -eq 124
'
```

The runtime should report that it started without libfranka/FCI ownership and
that zero-command publishing is disabled. The Core executable should report
that it owns Core through the ROS backend while its scheduler and command path
remain disabled.

## Offline test suite

The Docker image builds MIOS-only CTest executables by default; it does not
enable the upstream Franka test suites. Run them with networking disabled:

```bash
docker run --rm --network none --entrypoint bash mios-ros2-jazzy:local -lc '
  source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
  ctest --test-dir /ws/build/mios_ros2_control --output-on-failure
  ctest --test-dir /ws/build/mios_ros2_runtime --output-on-failure
'
```

This validates command limits, stale/invalid/user-stop handling, state and
parameter conversions, and the fail-closed ROS backend. It also starts a
temporary `controller_manager` with the seven-joint
`mock_components/GenericSystem`, loads/configures the MIOS effort controller,
and verifies both production commissioning-lock rejections, a test-only
stationary fake-hardware exact-zero-effort activation/deactivation, an
effort-to-effort HandGuiding/hold controller switch, and a runtime relock
before a later activation attempt. It does not start `franka_hardware` or
send a command to a robot.
