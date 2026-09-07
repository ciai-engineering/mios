#include <cassert>

#include "mios_ros2_runtime/robot_parameter_snapshot.hpp"

namespace {

std::array<double, 16> identity_transform() {
  return {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
}

mios_ros2_runtime::RobotParameterSnapshot valid_snapshot() {
  mios_ros2_runtime::RobotParameterSnapshot snapshot;
  snapshot.load_mass = 1.0;
  snapshot.load_inertia = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
  snapshot.tcp_frame = identity_transform();
  snapshot.stiffness_frame = identity_transform();
  snapshot.joint_stiffness.fill(100.0);
  snapshot.cartesian_stiffness.fill(200.0);
  snapshot.lower_torque_thresholds.fill(10.0);
  snapshot.upper_torque_thresholds.fill(20.0);
  snapshot.lower_force_thresholds.fill(10.0);
  snapshot.upper_force_thresholds.fill(20.0);
  return snapshot;
}

}  // namespace

int main() {
  auto snapshot = valid_snapshot();
  assert(mios_ros2_runtime::valid_robot_parameter_snapshot(snapshot));

  snapshot.upper_force_thresholds[2] = 5.0;
  assert(!mios_ros2_runtime::valid_robot_parameter_snapshot(snapshot));
  snapshot = valid_snapshot();
  snapshot.tcp_frame[0] = 2.0;
  assert(!mios_ros2_runtime::valid_robot_parameter_snapshot(snapshot));
  snapshot = valid_snapshot();
  snapshot.load_mass = -1.0;
  assert(!mios_ros2_runtime::valid_robot_parameter_snapshot(snapshot));
  return 0;
}
