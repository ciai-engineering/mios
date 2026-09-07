#include <cassert>
#include <cmath>

#include "mios_ros2_runtime/gripper_state_translation.hpp"

int main() {
  sensor_msgs::msg::JointState message;
  message.header.stamp.sec = 12;
  message.header.stamp.nanosec = 34;
  message.position = {0.0125, 0.0125};
  const auto snapshot = mios_ros2_runtime::to_mios_gripper_state(message, 0.08, true);
  assert(snapshot.has_value());
  assert(std::abs(snapshot->state.width - 0.025) < 1.0e-12);
  assert(std::abs(snapshot->state.max_width - 0.08) < 1.0e-12);
  assert(snapshot->state.is_grasped);
  assert(snapshot->state.temperature == 0.0);
  assert(snapshot->stamp_nanoseconds == 12000000034LL);

  // The installed gripper reports 40.526 mm per finger at its physical open
  // stop, slightly beyond the nominal 80 mm action-command range. This must
  // remain a valid live state rather than being discarded by Core.
  message.position = {0.040526, 0.040526};
  const auto open_snapshot = mios_ros2_runtime::to_mios_gripper_state(message, 0.08, false);
  assert(open_snapshot.has_value());
  assert(std::abs(open_snapshot->state.width - 0.081052) < 1.0e-12);
  assert(std::abs(open_snapshot->state.max_width - 0.08) < 1.0e-12);

  message.position = {0.05, 0.05};
  assert(!mios_ros2_runtime::to_mios_gripper_state(message, 0.08, false));
  message.position = {0.01};
  assert(!mios_ros2_runtime::to_mios_gripper_state(message, 0.08, false));
  return 0;
}
