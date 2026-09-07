#include "mios_ros2_control/mios_position_control_core.hpp"

#include <algorithm>
#include <cmath>

namespace mios_ros2_control {
namespace {

template <std::size_t Size>
bool all_finite(const std::array<double, Size>& values) {
  return std::all_of(values.begin(), values.end(),
                     [](const double value) { return std::isfinite(value); });
}

}  // namespace

bool is_safe_position_activation(const MiosPositionActivationState& state,
                                 const double velocity_threshold,
                                 const double position_tolerance,
                                 const double acceleration_threshold) {
  if (!std::isfinite(velocity_threshold) || velocity_threshold <= 0.0 ||
      !std::isfinite(position_tolerance) || position_tolerance <= 0.0 ||
      !std::isfinite(acceleration_threshold) || acceleration_threshold <= 0.0) {
    return false;
  }

  for (std::size_t index = 0; index < state.measured_position.size(); ++index) {
    const double measured_position = state.measured_position[index];
    const double live_position = state.live_position[index];
    const double desired_position = state.desired_position[index];
    const double measured_velocity = state.measured_velocity[index];
    const double live_velocity = state.live_velocity[index];
    const double desired_velocity = state.desired_velocity[index];
    const double desired_acceleration = state.desired_acceleration[index];
    if (!std::isfinite(measured_position) || !std::isfinite(live_position) ||
        !std::isfinite(desired_position) || !std::isfinite(measured_velocity) ||
        !std::isfinite(live_velocity) || !std::isfinite(desired_velocity) ||
        !std::isfinite(desired_acceleration) ||
        std::abs(measured_position - live_position) > position_tolerance ||
        std::abs(measured_position - desired_position) > position_tolerance ||
        std::abs(measured_velocity) > velocity_threshold ||
        std::abs(live_velocity) > velocity_threshold ||
        std::abs(desired_velocity) > velocity_threshold ||
        std::abs(desired_acceleration) > acceleration_threshold) {
      return false;
    }
  }
  return true;
}

void MiosPositionControlCore::configure(const Position& lower_limits,
                                        const Position& upper_limits,
                                        const Position& velocity_limits,
                                        const Position& acceleration_limits,
                                        const std::int64_t command_timeout_nanoseconds) {
  lower_limits_ = lower_limits;
  upper_limits_ = upper_limits;
  velocity_limits_ = velocity_limits;
  acceleration_limits_ = acceleration_limits;
  command_timeout_nanoseconds_ = command_timeout_nanoseconds;
  reset();
}

void MiosPositionControlCore::reset() {
  applied_position_.fill(0.0);
  applied_velocity_.fill(0.0);
  hold_reference_valid_ = false;
}

void MiosPositionControlCore::capture_hold_reference(const Position& position) {
  applied_position_ = position;
  applied_velocity_.fill(0.0);
  hold_reference_valid_ = all_finite(position);
}

const MiosPositionControlCore::Position& MiosPositionControlCore::hold_reference() const {
  return applied_position_;
}

MiosPositionControlCore::Position MiosPositionControlCore::step(
    const Position& measured_position, const bool user_stopped,
    const MiosPositionRequest* request, const std::int64_t now_nanoseconds,
    const double period_seconds) {
  const bool valid_period = std::isfinite(period_seconds) && period_seconds > 0.0;
  const bool valid_request = request == nullptr || all_finite(request->position);
  const bool request_is_fresh = request != nullptr && request->received_nanoseconds > 0 &&
                                now_nanoseconds >= request->received_nanoseconds &&
                                now_nanoseconds - request->received_nanoseconds <=
                                    command_timeout_nanoseconds_;
  if (!all_finite(measured_position) || !valid_period || !valid_request ||
      !hold_reference_valid_) {
    // Measured position is the only universally safe fallback in a position
    // controller when no finite Franka desired reference has been captured.
    // This avoids a zero/default command becoming a movement.
    applied_position_ = measured_position;
    hold_reference_valid_ = all_finite(measured_position);
    return applied_position_;
  }
  const bool hold_request = user_stopped || (request != nullptr && request->user_stopped) ||
                            !request_is_fresh;
  for (std::size_t index = 0; index < applied_position_.size(); ++index) {
    // A velocity-only limiter produces an instantaneous 0 -> vmax step at
    // 1 kHz. Ramp velocity as well, and use braking speed near the target.
    const double desired = hold_request
                               ? applied_position_[index]
                               : std::clamp(request->position[index], lower_limits_[index],
                                            upper_limits_[index]);
    const double error = desired - applied_position_[index];
    const double braking_speed = std::sqrt(
        std::max(0.0, 2.0 * acceleration_limits_[index] * std::abs(error)));
    const double requested_velocity = std::copysign(
        std::min(velocity_limits_[index], braking_speed), error);
    const double max_velocity_delta = acceleration_limits_[index] * period_seconds;
    applied_velocity_[index] += std::clamp(requested_velocity - applied_velocity_[index],
                                             -max_velocity_delta, max_velocity_delta);
    applied_position_[index] += applied_velocity_[index] * period_seconds;
    if (std::abs(error) < 1.0e-7 && std::abs(applied_velocity_[index]) < max_velocity_delta) {
      applied_position_[index] = desired;
      applied_velocity_[index] = 0.0;
    }
  }
  return applied_position_;
}

}  // namespace mios_ros2_control
