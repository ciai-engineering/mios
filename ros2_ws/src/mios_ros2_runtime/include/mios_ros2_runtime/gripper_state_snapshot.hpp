#pragma once

#include <cstdint>

#include "mios/control/control_types.hpp"

namespace mios_ros2_runtime {

// Non-real-time gripper state assembled from the ROS-owned Franka gripper
// node. The upstream joint-state topic exposes opening width but not native
// temperature/max-width/grasped fields, so those are explicitly documented by
// the adapter rather than invented from libfranka.
struct GripperStateSnapshot {
  mios::control::GripperState state{};
  std::int64_t stamp_nanoseconds{0};
};

}  // namespace mios_ros2_runtime
