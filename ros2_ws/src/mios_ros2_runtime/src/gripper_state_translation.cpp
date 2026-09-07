#include "mios_ros2_runtime/gripper_state_translation.hpp"

#include <cmath>
#include <cstdint>

namespace mios_ros2_runtime {
namespace {

// The gripper action API uses its nominal stroke (80 mm for the installed
// gripper), while the encoder-based joint-state feedback can report a small
// mechanical overtravel at the open stop.  Keep the action limit unchanged,
// but accept that calibrated feedback so Core does not discard a real state
// and retain its zero-initialized gripper percept.
constexpr double kReportedWidthTolerance = 0.002;

}  // namespace

std::optional<GripperStateSnapshot> to_mios_gripper_state(
    const sensor_msgs::msg::JointState& message, const double configured_max_width,
    const bool is_grasped) {
  if (!std::isfinite(configured_max_width) || configured_max_width <= 0.0 ||
      message.position.size() != 2 || !std::isfinite(message.position[0]) ||
      !std::isfinite(message.position[1]) || message.position[0] < 0.0 ||
      message.position[1] < 0.0) {
    return std::nullopt;
  }
  const double width = message.position[0] + message.position[1];
  // Accept the small calibrated overtravel reported at a physical open stop.
  // The nominal configured maximum remains the action-command limit stored in
  // state.max_width; state.width remains the actual measured opening.
  if (!std::isfinite(width) || width > configured_max_width + kReportedWidthTolerance) {
    return std::nullopt;
  }

  GripperStateSnapshot snapshot;
  snapshot.state.width = width;
  snapshot.state.max_width = configured_max_width;
  snapshot.state.temperature = 0.0;  // Not published by upstream franka_gripper.
  snapshot.state.is_grasped = is_grasped;
  snapshot.stamp_nanoseconds = static_cast<std::int64_t>(message.header.stamp.sec) * 1000000000LL +
                             static_cast<std::int64_t>(message.header.stamp.nanosec);
  return snapshot;
}

}  // namespace mios_ros2_runtime
