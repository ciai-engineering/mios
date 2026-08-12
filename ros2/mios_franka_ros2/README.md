# MIOS Franka ROS 2 backend

This package controls a Franka arm through the official `franka_ros2` stack.
`franka_hardware` remains the sole owner of the FCI connection and invokes
`libfranka` internally. Do not run the native MIOS robot backend, another
`libfranka` process, or another command controller against the same arm at the
same time.

The package supplies:

- a command-line client for robot state, asynchronous joint PTP motion,
  gripper actions, and FCI error recovery;
- a rate-limited Cartesian pose `ros2_control` controller;
- a Cartesian velocity controller with norm limits and a command watchdog;
- a launch file that starts `franka_bringup` and one Cartesian controller.

High-level MIOS skills such as insertion, contact search, taxonomy execution,
and object-memory lookup are not silently translated. Their implementations
live in the native MIOS backend, which is not part of this repository. They
must be ported as real-time `ros2_control` controllers before they can use this
backend safely.

## Build

Use Ubuntu with the real-time setup required by Franka. The current
`franka_ros2` Humble branch is the reference dependency.

```bash
mkdir -p ~/franka_ws/src
cd ~/franka_ws/src
git clone --branch humble https://github.com/frankarobotics/franka_ros2.git
git clone <this-repository-url> mios
cd ..
vcs import src < src/franka_ros2/dependency.repos --recursive --skip-existing
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Because the ROS package is below `mios/ros2`, build with this repository's
root in the workspace. Colcon recursively discovers the package.

## Start the robot

Activate FCI in Desk, unlock the joints, and launch one command mode:

```bash
ros2 launch mios_franka_ros2 mios_franka.launch.py \
  robot_ip:=192.168.1.10 \
  controller:=mios_cartesian_pose_controller
```

For velocity control:

```bash
ros2 launch mios_franka_ros2 mios_franka.launch.py \
  robot_ip:=192.168.1.10 \
  controller:=mios_cartesian_velocity_controller
```

Only one arm command controller can claim the Cartesian command interfaces.
Switch controllers with `ros2 control switch_controllers` when changing mode.
For prefixed or namespaced arms, set `namespace`, `arm_prefix`, and the
controller parameters in `config/controllers.yaml` consistently.

To start only the Franka hardware, broadcasters, action server, and gripper,
without a Cartesian command controller:

```bash
ros2 launch mios_franka_ros2 mios_franka.launch.py \
  robot_ip:=192.168.1.10 start_cartesian_controller:=false
```

## Commands

Read one full robot-state sample:

```bash
ros2 run mios_franka_ros2 mios_franka state
```

Execute a seven-joint point-to-point motion through the official
`PTPMotion` action:

```bash
ros2 run mios_franka_ros2 mios_franka joint \
  0.0 -0.785 0.0 -2.356 0.0 1.571 0.785 \
  --max-velocity 0.2
```

The PTP action must not run while a controller is claiming incompatible arm
command interfaces. Launch with `start_cartesian_controller:=false`, or stop
the active controller first.

Send an absolute Cartesian target. The controller interpolates toward it at
the configured translational and angular limits:

```bash
ros2 run mios_franka_ros2 mios_franka pose \
  0.45 0.0 0.40 0.0 0.0 0.0 1.0
```

Send a Cartesian velocity for one second. Commands are published continuously;
the controller commands zero motion if updates stop for 100 ms:

```bash
ros2 run mios_franka_ros2 mios_franka twist \
  0.02 0.0 0.0 0.0 0.0 0.0 --duration 1.0
```

Control the Franka Hand and recover from an FCI error:

```bash
ros2 run mios_franka_ros2 mios_franka home
ros2 run mios_franka_ros2 mios_franka gripper-move 0.06 --speed 0.05
ros2 run mios_franka_ros2 mios_franka grasp 0.03 --force 30
ros2 run mios_franka_ros2 mios_franka recover
```

After recovery, `franka_hardware` may also require hardware and controller
reactivation, depending on the installed `franka_ros2` version:

```bash
ros2 control set_hardware_component_state FrankaHardwareInterface active
ros2 control switch_controllers --activate mios_cartesian_pose_controller
```

All action and topic names can be overridden with the global CLI options shown
by `mios_franka --help`. Use `--namespace` for a namespaced robot.

## Safety behavior

- Cartesian pose targets are quaternion-validated and interpolated inside the
  1 kHz controller update.
- Cartesian velocities are norm-limited in the controller, not only in the
  command-line client.
- A stale velocity command becomes a zero command after `command_timeout`.
- Messages with a frame different from `base_frame` are rejected. This package
  intentionally does not perform TF lookup in the real-time controller.
- Franka collision behavior, joint/cartesian limits, FCI exclusivity, and the
  external emergency stop remain authoritative.

Test first with `use_fake_hardware:=true`. Fake hardware validates ROS graph
integration but does not reproduce all FCI reflexes or timing behavior.
