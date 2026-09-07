#pragma once

#include <cstdint>

#include "mios/control/control_types.hpp"

namespace mios_ros2_runtime {

// Immutable non-real-time model snapshot received from the read-only model
// broadcaster. Its timestamp lets a future Core scheduler reject stale model
// data before running a model-dependent MIOS algorithm.
struct RobotModelSnapshot {
  mios::control::RobotModel model{};
  std::int64_t stamp_nanoseconds{0};
};

}  // namespace mios_ros2_runtime
