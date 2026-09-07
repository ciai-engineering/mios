#include "mios_ros2_control/mios_velocity_control_core.hpp"

#include <algorithm>
#include <cmath>

namespace mios_ros2_control {
namespace {

template <std::size_t Size>
bool all_finite(const std::array<double, Size>& values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

}  // namespace

void MiosVelocityControlCore::configure(const Velocity& velocity_limits,
                                        const Velocity& acceleration_limits,
                                        const Position& lower_limits,
                                        const Position& upper_limits,
                                        const Position& joint_limit_margins,
                                        const std::int64_t command_timeout_nanoseconds) {
  velocity_limits_ = velocity_limits;
  acceleration_limits_ = acceleration_limits;
  lower_limits_ = lower_limits;
  upper_limits_ = upper_limits;
  joint_limit_margins_ = joint_limit_margins;
  command_timeout_nanoseconds_ = command_timeout_nanoseconds;
  reset();
}

void MiosVelocityControlCore::reset() {
  applied_velocity_.fill(0.0);
}

MiosVelocityControlCore::Velocity MiosVelocityControlCore::step(
    const Position& measured_position, const Velocity& measured_velocity,
    const bool user_stopped,
    const MiosVelocityRequest* request, const std::int64_t now_nanoseconds,
    const double period_seconds) {
  const bool valid_period = std::isfinite(period_seconds) && period_seconds > 0.0;
  const bool valid_request = request == nullptr || all_finite(request->velocity);
  const bool request_is_fresh = request != nullptr && request->received_nanoseconds > 0 &&
                                now_nanoseconds >= request->received_nanoseconds &&
                                now_nanoseconds - request->received_nanoseconds <=
                                    command_timeout_nanoseconds_;
  if (!all_finite(measured_position) || !all_finite(measured_velocity) || user_stopped ||
      !valid_period || !valid_request ||
      (request != nullptr && request->user_stopped) || !request_is_fresh) {
    applied_velocity_.fill(0.0);
    return applied_velocity_;
  }

  for (std::size_t index = 0; index < applied_velocity_.size(); ++index) {
    const double desired = std::clamp(request->velocity[index], -velocity_limits_[index],
                                      velocity_limits_[index]);
    // This independent ROS-control boundary is deliberately inside Franka's
    // physical limits. A MIOS velocity task may not command farther into the
    // configured margin; a command back toward the safe range remains valid.
    const bool moving_beyond_lower_margin =
        measured_position[index] <= lower_limits_[index] + joint_limit_margins_[index] &&
        (desired < 0.0 || applied_velocity_[index] < 0.0);
    const bool moving_beyond_upper_margin =
        measured_position[index] >= upper_limits_[index] - joint_limit_margins_[index] &&
        (desired > 0.0 || applied_velocity_[index] > 0.0);
    if (moving_beyond_lower_margin || moving_beyond_upper_margin) {
      applied_velocity_[index] = 0.0;
      continue;
    }
    const double max_delta = acceleration_limits_[index] * period_seconds;
    applied_velocity_[index] +=
        std::clamp(desired - applied_velocity_[index], -max_delta, max_delta);
  }
  return applied_velocity_;
}

}  // namespace mios_ros2_control
