#pragma once

#include <array>
#include <cstdint>

namespace mios_ros2_control {

struct MiosPositionRequest {
  std::array<double, 7> position{};
  std::int64_t received_nanoseconds{0};
  bool user_stopped{false};
};

// Joint-position mode must start from a settled Franka motion-generator
// reference. Checking measured velocity alone is insufficient: switching to
// a measured-position hold while q_d, dq_d, or ddq_d still describe an active
// trajectory can create a discontinuous first position command.
struct MiosPositionActivationState {
  using Position = std::array<double, 7>;

  Position measured_position{};
  Position measured_velocity{};
  Position live_position{};
  Position live_velocity{};
  Position desired_position{};
  Position desired_velocity{};
  Position desired_acceleration{};
};

bool is_safe_position_activation(const MiosPositionActivationState& state,
                                 double velocity_threshold,
                                 double position_tolerance,
                                 double acceleration_threshold);

// Deterministic output conditioning for MIOS joint-position commands.
class MiosPositionControlCore {
 public:
  using Position = std::array<double, 7>;

  void configure(const Position& lower_limits, const Position& upper_limits,
                 const Position& velocity_limits, const Position& acceleration_limits,
                 std::int64_t command_timeout_nanoseconds);
  void reset();
  void capture_hold_reference(const Position& position);
  [[nodiscard]] const Position& hold_reference() const;

  // A no-target hold preserves the captured Franka desired reference. Replacing
  // it with q (the measured position) can create a discontinuity relative to
  // q_d on the first 1 kHz JointPosition command, even when q and q_d differ
  // by only a few microradians.
  [[nodiscard]] Position step(const Position& measured_position, bool user_stopped,
                              const MiosPositionRequest* request,
                              std::int64_t now_nanoseconds, double period_seconds);

 private:
  Position lower_limits_{};
  Position upper_limits_{};
  Position velocity_limits_{};
  Position acceleration_limits_{};
  Position applied_position_{};
  Position applied_velocity_{};
  bool hold_reference_valid_{false};
  std::int64_t command_timeout_nanoseconds_{0};
};

}  // namespace mios_ros2_control
