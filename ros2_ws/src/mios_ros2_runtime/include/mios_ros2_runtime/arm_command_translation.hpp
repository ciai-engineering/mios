#pragma once

#include <array>
#include <optional>

#include "mios/control/control_types.hpp"

namespace mios_ros2_runtime {

// The ROS runtime consumes the same transport-neutral command that the
// existing MIOS algorithms return. This type records which typed ROS command
// publisher must be used; it contains no ROS handles and is unit-testable.
enum class ArmCommandDispatchMode {
  kEffort,
  kJointVelocity,
  kCartesianVelocity,
  kJointPosition,
  kCartesianPose,
};

struct ArmCommandDispatch {
  ArmCommandDispatchMode mode{ArmCommandDispatchMode::kEffort};
  std::array<double, mios::control::kJointCount> joints{};
  std::array<double, 6> cartesian{};
  std::array<double, 16> pose{};
  // A completed MIOS motion is always treated as a safe stop at the ROS
  // controller boundary. The active typed controller then holds/zeros by its
  // own fail-safe policy.
  bool user_stopped{false};
};

// Returns no dispatch for an unknown enum value. This is deliberately
// fail-closed so a future legacy command mode cannot move hardware until a
// typed ROS controller mapping and its tests are added.
std::optional<ArmCommandDispatch> translate_arm_command(
    const mios::control::ArmCommand& command, bool robot_user_stopped);

}  // namespace mios_ros2_runtime
