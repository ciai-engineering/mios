#pragma once

#include <array>
#include <cstdint>

namespace mios_ros2_control {

struct MiosVelocityRequest {
  std::array<double, 7> velocity{};
  std::int64_t received_nanoseconds{0};
  bool user_stopped{false};
};

// Deterministic command conditioning for the MIOS joint-velocity pipeline.
// This class intentionally contains no ROS handles or hardware interfaces.
class MiosVelocityControlCore {
 public:
  using Velocity = std::array<double, 7>;
  using Position = std::array<double, 7>;

  void configure(const Velocity& velocity_limits, const Velocity& acceleration_limits,
                 const Position& lower_limits, const Position& upper_limits,
                 const Position& joint_limit_margins,
                 std::int64_t command_timeout_nanoseconds);
  void reset();

  // Invalid robot state, user stop, stale commands, and invalid periods all
  // clear the output immediately instead of waiting for acceleration limiting.
  [[nodiscard]] Velocity step(const Position& measured_position,
                              const Velocity& measured_velocity, bool user_stopped,
                              const MiosVelocityRequest* request,
                              std::int64_t now_nanoseconds, double period_seconds);

 private:
  Velocity velocity_limits_{};
  Velocity acceleration_limits_{};
  Position lower_limits_{};
  Position upper_limits_{};
  Position joint_limit_margins_{};
  Velocity applied_velocity_{};
  std::int64_t command_timeout_nanoseconds_{0};
};

}  // namespace mios_ros2_control
