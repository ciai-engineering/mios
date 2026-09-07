#include "mios_ros2_control/mios_cartesian_velocity_control_core.hpp"

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

void MiosCartesianVelocityControlCore::configure(
    const CartesianVelocity& velocity_limits, const CartesianVelocity& acceleration_limits,
    const std::int64_t command_timeout_nanoseconds) {
  velocity_limits_ = velocity_limits;
  acceleration_limits_ = acceleration_limits;
  command_timeout_nanoseconds_ = command_timeout_nanoseconds;
  reset();
}

void MiosCartesianVelocityControlCore::reset() {
  applied_velocity_.fill(0.0);
}

MiosCartesianVelocityControlCore::CartesianVelocity
MiosCartesianVelocityControlCore::step(const JointVelocity& measured_joint_velocity,
                                       const bool user_stopped,
                                       const MiosCartesianVelocityRequest* request,
                                       const std::int64_t now_nanoseconds,
                                       const double period_seconds) {
  const bool valid_period = std::isfinite(period_seconds) && period_seconds > 0.0;
  const bool valid_request = request == nullptr || all_finite(request->velocity);
  const bool request_is_fresh = request != nullptr && request->received_nanoseconds > 0 &&
                                now_nanoseconds >= request->received_nanoseconds &&
                                now_nanoseconds - request->received_nanoseconds <=
                                    command_timeout_nanoseconds_;
  if (!all_finite(measured_joint_velocity) || user_stopped || !valid_period || !valid_request ||
      (request != nullptr && request->user_stopped) || !request_is_fresh) {
    applied_velocity_.fill(0.0);
    return applied_velocity_;
  }
  for (std::size_t index = 0; index < applied_velocity_.size(); ++index) {
    const double desired = std::clamp(request->velocity[index], -velocity_limits_[index],
                                      velocity_limits_[index]);
    const double max_delta = acceleration_limits_[index] * period_seconds;
    applied_velocity_[index] +=
        std::clamp(desired - applied_velocity_[index], -max_delta, max_delta);
  }
  return applied_velocity_;
}

}  // namespace mios_ros2_control
