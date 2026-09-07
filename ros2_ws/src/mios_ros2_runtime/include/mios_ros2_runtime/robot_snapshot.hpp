#pragma once

#include <array>
#include <cstdint>

namespace mios_ros2_runtime {

inline constexpr std::size_t kJointCount = 7;

// Mirrors the Franka ROS state message without making the neutral snapshot
// depend on a ROS message type. Keep values explicit to make mode conversion
// auditable at the legacy MIOS boundary.
enum class RobotRuntimeMode : std::uint8_t {
  kOther,
  kIdle,
  kMove,
  kGuiding,
  kReflex,
  kUserStopped,
  kAutomaticErrorRecovery,
};

// Transport-neutral state received by the non-real-time MIOS runtime. This
// package owns no libfranka object; franka_hardware remains the FCI owner.
struct RobotSnapshot {
  std::array<double, kJointCount> position{};
  std::array<double, kJointCount> velocity{};
  std::array<double, kJointCount> effort{};
  std::array<double, kJointCount> motor_position{};
  std::array<double, kJointCount> motor_velocity{};
  std::array<double, kJointCount> external_effort{};
  std::array<double, 16> base_to_end_effector{};
  std::array<double, 6> external_wrench_origin{};
  std::array<double, 6> external_wrench_stiffness{};
  std::int64_t stamp_nanoseconds{0};
  RobotRuntimeMode robot_mode{RobotRuntimeMode::kOther};
  bool user_stopped{false};
};

}  // namespace mios_ros2_runtime
