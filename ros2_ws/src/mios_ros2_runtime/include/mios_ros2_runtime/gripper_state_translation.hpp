#pragma once

#include <optional>

#include "mios_ros2_runtime/gripper_state_snapshot.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace mios_ros2_runtime {

// Franka's upstream gripper node publishes two equal finger positions, each
// equal to half of the physical opening. The encoder can report a small
// calibrated overtravel at the physical open stop, which is accepted while
// retaining configured_max_width as the gripper action-command limit. Reject
// malformed or materially out-of-range messages instead of silently producing
// a misleading Core percept.
std::optional<GripperStateSnapshot> to_mios_gripper_state(
    const sensor_msgs::msg::JointState& message, double configured_max_width,
    bool is_grasped);

}  // namespace mios_ros2_runtime
