#pragma once

#include <optional>

#include "mios/control/control_types.hpp"
#include "mios_ros2_runtime/robot_parameter_snapshot.hpp"

namespace mios_ros2_runtime {
// Converts Core's transport-neutral parameter contract to the ROS-owned
// service snapshot. The Core-side mapper owns the legacy mios::Parameters and
// Eigen conversion; this adapter owns validation at the ROS boundary.
std::optional<RobotParameterSnapshot> make_robot_parameter_snapshot(
    const mios::control::RobotParameters& parameters) {
  RobotParameterSnapshot snapshot;
  snapshot.load_mass = parameters.load_mass;
  snapshot.load_center_of_mass = parameters.load_center_of_mass;
  snapshot.load_inertia = parameters.load_inertia;
  snapshot.tcp_frame = parameters.tcp_frame;
  snapshot.stiffness_frame = parameters.stiffness_frame;
  snapshot.joint_stiffness = parameters.joint_stiffness;
  snapshot.cartesian_stiffness = parameters.cartesian_stiffness;
  snapshot.lower_torque_thresholds = parameters.lower_torque_thresholds;
  snapshot.upper_torque_thresholds = parameters.upper_torque_thresholds;
  snapshot.lower_force_thresholds = parameters.lower_force_thresholds;
  snapshot.upper_force_thresholds = parameters.upper_force_thresholds;

  if (!valid_robot_parameter_snapshot(snapshot)) {
    return std::nullopt;
  }
  return snapshot;
}

}  // namespace mios_ros2_runtime
