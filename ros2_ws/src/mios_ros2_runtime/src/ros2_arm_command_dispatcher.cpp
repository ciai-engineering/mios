#include "mios_ros2_runtime/ros2_arm_command_dispatcher.hpp"

#include "mios_ros2_runtime/arm_command_translation.hpp"

namespace mios_ros2_runtime {

bool Ros2ArmCommandDispatcher::dispatch(const mios::control::ArmCommand& command,
                                        const bool robot_user_stopped) const {
  const auto translated = translate_arm_command(command, robot_user_stopped);
  if (!translated.has_value()) {
    return false;
  }

  const ArmCommandDispatch& dispatch = *translated;
  switch (dispatch.mode) {
    case ArmCommandDispatchMode::kEffort:
      // Do not forward a completed/stopped torque vector, even though the
      // controller independently rejects it. The publisher itself sends an
      // explicit zero-effort safe stop.
      if (dispatch.user_stopped) {
        return backend_.publish_zero_effort(true);
      }
      return backend_.publish_effort(dispatch.joints);
    case ArmCommandDispatchMode::kJointVelocity:
      return backend_.publish_joint_velocity(dispatch.joints, dispatch.user_stopped);
    case ArmCommandDispatchMode::kCartesianVelocity:
      return backend_.publish_cartesian_velocity(dispatch.cartesian, dispatch.user_stopped);
    case ArmCommandDispatchMode::kJointPosition:
      return backend_.publish_joint_position(dispatch.joints, dispatch.user_stopped);
    case ArmCommandDispatchMode::kCartesianPose:
      return backend_.publish_cartesian_pose(dispatch.pose, dispatch.user_stopped);
  }
  return false;
}

}  // namespace mios_ros2_runtime
