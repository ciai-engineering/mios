#pragma once

#include "mios/data_structures/parameters.hpp"

namespace mios::control {

// Immutable configuration snapshot consumed by the deterministic MIOS
// pipelines. Runtime/database code creates this before a control run; the
// real-time executor never reads Memory or any external service.
struct ControlRuntimeConfig {
  ControlParameters control{};
  SafetyParameters safety{};
  LimitParameters limits{};
  FramesParameters frames{};
};

}  // namespace mios::control
