# MIOS ROS 2 runtime transport

`mios_runtime_node` is the non-real-time ROS boundary for MIOS. It owns no
`franka::Robot`, does not activate FCI, and obtains state only from
`franka_robot_state_broadcaster`.

`Ros2RobotBackend` converts `franka_msgs/msg/FrankaRobotState` into a complete
neutral `RobotSnapshot`: joint and motor state, external torque/wrenches,
end-effector pose, timestamp, and user-stop state. It publishes typed ROS
commands for the controller boundary.

The snapshot retains Franka's complete robot mode and has a ROS-independent
conversion to the legacy neutral `mios::control::RobotState`. The backend
exposes that converted state through `latest_mios_robot_state()`; this is the
state input for the ROS Core adapter, not a ros2_control update-loop entry
point.

`MiosRobotModelBroadcaster` is a read-only `ros2_control` controller. It reads
the ROS-owned Franka semantic model interfaces and publishes mass, Coriolis,
gravity, and end-effector Jacobians on
`/mios_robot_model_broadcaster/robot_model`. `Ros2RobotBackend` validates and
stores the latest model snapshot. It claims no command interfaces and must be
explicitly loaded/activated through controller manager.

`Ros2GripperClient` provides explicit asynchronous `grasp`, `move`, `home`,
and `stop` requests to the upstream `franka_gripper` action/service API. It
does not create a libfranka gripper object, wait indefinitely, or issue any
request at runtime-node startup. It also subscribes to the upstream
`/franka_gripper/joint_states` topic and reconstructs opening width from its
two half-width finger positions. `gripper_max_width` is an explicit runtime
parameter (default `0.08` m); upstream does not publish temperature or maximum
width, so temperature remains `0.0` and the configured maximum is reported.
Successful explicit grasp, move, and homing actions update the retained grasp
flag.

`Ros2RobotParameterClient` is the explicit, non-real-time adapter for the
upstream `/service_server` parameter services: load, TCP frame, collision
thresholds, stiffness frame, Cartesian stiffness, and joint stiffness. It
validates a complete `RobotParameterSnapshot`, requires every service to be
ready before it sends the first request, and preserves the original MIOS
application order. `allow_robot_parameter_application` defaults to `false`,
and node startup never applies parameters. This client has no libfranka object
or FCI ownership.

`Ros2CoreRobotBackend` consumes a Core-owned, immutable
`ParameterSnapshotProvider` and synchronously waits for that explicit service
operation outside a controller update cycle. It rejects a disabled task
runtime, missing provider, invalid snapshot, unavailable service, timeout, or
unsuccessful response. The shipped runtime node deliberately supplies no
provider because it does not yet create the legacy Core/Memory scheduler.

Core maps its Memory-owned legacy `mios::Parameters` through
`mios::control::make_robot_parameters()`, preserving the exact fields used by
`PandaBody::set_robot_parameters()`: load, `EE_T_TCP`, `EE_T_K`, joint and
Cartesian stiffness, and collision thresholds. Core installs that
transport-neutral provider on its `RobotBackend` after Memory initialization.
The ROS-side
`make_robot_parameter_snapshot()` translates the neutral Core contract to ROS
service data, validates it, and returns no snapshot for invalid data. Eigen's
column-major transform representation is preserved at the legacy-to-Core
boundary, leaving ROS without database or FCI ownership.

Each command includes the snapshot's `user_stopped` flag. The effort
controller treats that flag as fail-safe zero torque; it independently reads
the same robot mode from Franka's semantic state interface in its real-time
update loop.

The original MIOS controller pipelines and both safety stages now use the same
neutral `mios::control::ArmCommand` boundary: they no longer construct or
modify libfranka command objects. `Ros2ArmCommandDispatcher` maps every
currently defined `ArmCommand` mode (torque, joint/Cartesian velocity, joint
position, and Cartesian pose) to the matching typed ROS publisher. It is
fail-closed for a new/unknown mode, and a completed motion becomes an explicit
safe stop. `MiosAlgorithmExecutor` runs that pipeline and safety sequence from
an immutable configuration snapshot, without reading the database in its
control cycle.

`Ros2CoreRobotBackend` implements the original `mios::RobotBackend` contract
without libfranka. It consumes fresh state/model snapshots, runs a supplied
Core callback from the non-real-time state-subscription path, and serializes
the resulting command through `Ros2ArmCommandDispatcher`. It permits only one
active callback and finishes it on a completed MIOS command. Its gripper calls
wait for the explicit upstream ROS action/service result and provide the
validated gripper-state snapshot when the upstream topic is available.

`mios_runtime_node` remains a transport-only process: it creates a
Core-compatible backend but intentionally does not create Core or access the
database. The separate `mios_ros2_core_runtime` executable is built from the
repository root with `MIOS_BUILD_ROS2_CORE_RUNTIME=ON`. It constructs
`mios::Core` with `Ros2CoreRobotBackend` as its only backend; it never creates
`PandaBody`, `franka::Robot`, or an FCI connection.

`enable_core_scheduler` and `enable_core_task_execution` both default to
`false`. With the defaults, the Core construction test does not initialize the
database, portal, gripper, parameter services, or arm command path. Enabling
the scheduler starts the existing Core database/portal/task infrastructure;
the second flag is still required before the adapter can dispatch any arm or
gripper command. Core installs its Memory-owned parameter provider only during
that explicit initialization path. Desk operations and error recovery remain
fail-closed until dedicated ROS/Desk adapters and commissioning tests exist.

The separate `mios_ros2_core_runtime` also keeps controller-owned `MOVE`
disabled by default. Its
`allow_controller_owned_move_mode_without_task_execution` parameter is an
opt-in scheduler-load diagnostic gate: together with
`allow_controller_owned_move_mode:=true`, it lets the legacy scheduler observe
the ROS-owned zero-command `MOVE` state while task execution is disabled. It
does not permit arm or gripper dispatch; those paths remain blocked until
`enable_core_task_execution:=true`.

The optional `publish_zero_commands` setting emits only zero torques and is
intended for transport verification after the effort controller has been
explicitly enabled.
