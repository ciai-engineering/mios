#include "mios_ros2_runtime/robot_state_translation.hpp"

namespace mios_ros2_runtime {
namespace {

static_assert(kJointCount == mios::control::kJointCount,
              "ROS and MIOS joint-state contracts must have the same size.");

mios::control::RobotMode to_mios_robot_mode(const RobotRuntimeMode mode) {
  switch (mode) {
    case RobotRuntimeMode::kOther:
      return mios::control::RobotMode::kOther;
    case RobotRuntimeMode::kIdle:
      return mios::control::RobotMode::kIdle;
    case RobotRuntimeMode::kMove:
      return mios::control::RobotMode::kMove;
    case RobotRuntimeMode::kGuiding:
      return mios::control::RobotMode::kGuiding;
    case RobotRuntimeMode::kReflex:
      return mios::control::RobotMode::kReflex;
    case RobotRuntimeMode::kUserStopped:
      return mios::control::RobotMode::kUserStopped;
    case RobotRuntimeMode::kAutomaticErrorRecovery:
      return mios::control::RobotMode::kAutomaticErrorRecovery;
  }
  return mios::control::RobotMode::kOther;
}

}  // namespace

mios::control::RobotState to_mios_robot_state(const RobotSnapshot& snapshot) {
  mios::control::RobotState state;
  state.position = snapshot.position;
  state.velocity = snapshot.velocity;
  state.motor_position = snapshot.motor_position;
  state.motor_velocity = snapshot.motor_velocity;
  state.effort = snapshot.effort;
  state.external_effort = snapshot.external_effort;
  state.base_to_end_effector = snapshot.base_to_end_effector;
  state.external_wrench_origin = snapshot.external_wrench_origin;
  state.external_wrench_stiffness = snapshot.external_wrench_stiffness;
  state.robot_mode = snapshot.user_stopped ? mios::control::RobotMode::kUserStopped
                                           : to_mios_robot_mode(snapshot.robot_mode);
  state.user_stopped = snapshot.user_stopped;
  return state;
}

}  // namespace mios_ros2_runtime
