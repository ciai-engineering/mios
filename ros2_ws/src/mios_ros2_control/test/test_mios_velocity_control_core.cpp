#include <cassert>
#include <cmath>

#include "mios_ros2_control/mios_velocity_control_core.hpp"

namespace {
bool almost_equal(const double left, const double right) {
  return std::abs(left - right) < 1.0e-12;
}
}  // namespace

int main() {
  using mios_ros2_control::MiosVelocityControlCore;
  using mios_ros2_control::MiosVelocityRequest;

  MiosVelocityControlCore core;
  const MiosVelocityControlCore::Velocity limit = {0.05, 0.05, 0.05, 0.05,
                                                    0.05, 0.05, 0.05};
  const MiosVelocityControlCore::Velocity acceleration = {0.20, 0.20, 0.20, 0.20,
                                                           0.20, 0.20, 0.20};
  MiosVelocityControlCore::Position lower{};
  MiosVelocityControlCore::Position upper{};
  MiosVelocityControlCore::Position margin{};
  lower.fill(-1.0);
  upper.fill(1.0);
  margin.fill(0.1);
  core.configure(limit, acceleration, lower, upper, margin, 100000000);

  MiosVelocityControlCore::Position measured_position{};
  MiosVelocityControlCore::Velocity measured{};
  MiosVelocityRequest request;
  request.velocity[0] = 1.0;
  request.received_nanoseconds = 1000;
  const auto first = core.step(measured_position, measured, false, &request, 1000, 0.1);
  assert(almost_equal(first[0], 0.02));
  const auto second = core.step(measured_position, measured, false, &request, 2000, 0.1);
  assert(almost_equal(second[0], 0.04));

  // A stale request and a stop both bypass acceleration limiting and command
  // zero in the same update cycle.
  const auto stale = core.step(measured_position, measured, false, &request, 101000001, 0.1);
  assert(almost_equal(stale[0], 0.0));
  request.received_nanoseconds = 101000002;
  request.user_stopped = true;
  const auto stopped = core.step(measured_position, measured, false, &request, 101000002, 0.1);
  assert(almost_equal(stopped[0], 0.0));

  // A request farther into the configured limit margin is stopped in the
  // same cycle. A request toward the safe range is still rate limited and
  // accepted, which permits an operator to recover from that margin.
  request.user_stopped = false;
  request.velocity[0] = 0.05;
  request.received_nanoseconds = 101000003;
  measured_position[0] = 0.9;
  const auto outward_at_margin =
      core.step(measured_position, measured, false, &request, 101000003, 0.1);
  assert(almost_equal(outward_at_margin[0], 0.0));

  request.velocity[0] = -0.05;
  request.received_nanoseconds = 101000004;
  const auto inward_from_margin =
      core.step(measured_position, measured, false, &request, 101000004, 0.1);
  assert(almost_equal(inward_from_margin[0], -0.02));
}
