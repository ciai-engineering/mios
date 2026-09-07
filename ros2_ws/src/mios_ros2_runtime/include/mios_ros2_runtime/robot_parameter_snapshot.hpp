#pragma once

#include <array>

namespace mios_ros2_runtime {

// ROS-independent robot configuration equivalent to the values applied by
// legacy PandaBody::set_robot_parameters(). Matrices use the Franka/libfranka
// column-major layout required by the upstream parameter services.
struct RobotParameterSnapshot {
  double load_mass{0.0};
  std::array<double, 3> load_center_of_mass{};
  std::array<double, 9> load_inertia{};
  std::array<double, 16> tcp_frame{};
  std::array<double, 16> stiffness_frame{};
  std::array<double, 7> joint_stiffness{};
  std::array<double, 6> cartesian_stiffness{};
  std::array<double, 7> lower_torque_thresholds{};
  std::array<double, 7> upper_torque_thresholds{};
  std::array<double, 6> lower_force_thresholds{};
  std::array<double, 6> upper_force_thresholds{};
};

bool valid_robot_parameter_snapshot(const RobotParameterSnapshot& snapshot);

}  // namespace mios_ros2_runtime
