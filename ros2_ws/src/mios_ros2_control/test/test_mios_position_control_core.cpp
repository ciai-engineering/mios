#include <cassert>
#include <cmath>
#include <cstdint>

#include "mios_ros2_control/mios_position_control_core.hpp"

namespace {

bool nearly_equal(const double left, const double right) {
  return std::abs(left - right) < 1.0e-12;
}

}  // namespace

int main() {
  using mios_ros2_control::MiosPositionControlCore;
  using mios_ros2_control::MiosPositionActivationState;
  using mios_ros2_control::MiosPositionRequest;

  MiosPositionActivationState activation_state;
  assert(mios_ros2_control::is_safe_position_activation(activation_state, 0.01, 0.001, 0.1));
  activation_state.desired_velocity[0] = 0.02;
  assert(!mios_ros2_control::is_safe_position_activation(activation_state, 0.01, 0.001, 0.1));
  activation_state.desired_velocity[0] = 0.0;
  activation_state.desired_position[1] = 0.002;
  assert(!mios_ros2_control::is_safe_position_activation(activation_state, 0.01, 0.001, 0.1));
  activation_state.desired_position[1] = 0.0;
  activation_state.desired_acceleration[2] = 0.2;
  assert(!mios_ros2_control::is_safe_position_activation(activation_state, 0.01, 0.001, 0.1));

  MiosPositionControlCore core;
  MiosPositionControlCore::Position lower{};
  MiosPositionControlCore::Position upper{};
  MiosPositionControlCore::Position velocity{};
  MiosPositionControlCore::Position acceleration{};
  lower.fill(-1.0);
  upper.fill(1.0);
  velocity.fill(0.2);
  acceleration.fill(0.1);
  core.configure(lower, upper, velocity, acceleration, 100'000'000);

  // Regression: q can differ from Franka's q_d by only a few microradians
  // while the arm is at rest. A no-target position controller must preserve
  // q_d; replacing it with q creates a discontinuous first 1 kHz command.
  MiosPositionControlCore::Position desired_reference{};
  desired_reference[0] = 0.4;
  MiosPositionControlCore::Position measured_for_hold = desired_reference;
  measured_for_hold[0] -= 4.5e-6;
  core.capture_hold_reference(desired_reference);
  MiosPositionRequest no_target;
  const auto held = core.step(measured_for_hold, false, &no_target, 1'000, 0.001);
  assert(nearly_equal(held[0], desired_reference[0]));
  assert(nearly_equal(core.hold_reference()[0], desired_reference[0]));

  MiosPositionControlCore::Position measured{};
  core.capture_hold_reference(measured);
  MiosPositionRequest request;
  request.position.fill(10.0);
  request.received_nanoseconds = 1'000;

  const auto first = core.step(measured, false, &request, 1'000, 0.1);
  assert(nearly_equal(first[0], 0.001));
  const auto second = core.step(measured, false, &request, 2'000, 0.1);
  assert(nearly_equal(second[0], 0.003));

  measured[0] = 0.37;
  const auto stale = core.step(measured, false, &request, 101'000'001, 0.1);
  assert(nearly_equal(stale[0], 0.004));

  request.received_nanoseconds = 101'000'002;
  request.user_stopped = true;
  measured[0] = 0.42;
  const auto stopped = core.step(measured, false, &request, 101'000'002, 0.1);
  assert(nearly_equal(stopped[0], 0.004));
  return 0;
}
