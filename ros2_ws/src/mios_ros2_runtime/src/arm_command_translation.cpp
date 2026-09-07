#include "mios_ros2_runtime/arm_command_translation.hpp"

namespace mios_ros2_runtime {

std::optional<ArmCommandDispatch> translate_arm_command(
    const mios::control::ArmCommand& command, const bool robot_user_stopped) {
  ArmCommandDispatch dispatch;
  dispatch.joints = command.joints;
  dispatch.cartesian = command.cartesian;
  dispatch.pose = command.pose;
  dispatch.user_stopped = robot_user_stopped || command.motion_finished;

  switch (command.mode) {
    case mios::control::CommandMode::kTorque:
      dispatch.mode = ArmCommandDispatchMode::kEffort;
      break;
    case mios::control::CommandMode::kJointVelocity:
      dispatch.mode = ArmCommandDispatchMode::kJointVelocity;
      break;
    case mios::control::CommandMode::kCartesianVelocity:
      dispatch.mode = ArmCommandDispatchMode::kCartesianVelocity;
      break;
    case mios::control::CommandMode::kJointPosition:
      dispatch.mode = ArmCommandDispatchMode::kJointPosition;
      break;
    case mios::control::CommandMode::kCartesianPose:
      dispatch.mode = ArmCommandDispatchMode::kCartesianPose;
      break;
    default:
      return std::nullopt;
  }
  return dispatch;
}

}  // namespace mios_ros2_runtime
