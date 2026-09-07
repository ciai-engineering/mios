#pragma once

#include "mios/control/control_types.hpp"
#include "mios_ros2_runtime/ros2_robot_backend.hpp"

namespace mios_ros2_runtime {

// Non-real-time bridge from the MIOS algorithm result to a typed ROS command
// topic. It never opens an FCI connection and does not choose or activate a
// ros2_control controller; controller-manager mode switching remains an
// explicit operator action.
class Ros2ArmCommandDispatcher {
 public:
  explicit Ros2ArmCommandDispatcher(Ros2RobotBackend& backend) : backend_(backend) {}

  // robot_user_stopped must come from the latest robot snapshot. A completed
  // MIOS command is also converted to a safe stop by translate_arm_command().
  bool dispatch(const mios::control::ArmCommand& command, bool robot_user_stopped) const;

 private:
  Ros2RobotBackend& backend_;
};

}  // namespace mios_ros2_runtime
