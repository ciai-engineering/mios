#include <cassert>
#include <cmath>

#include "mios_ros2_control/mios_cartesian_velocity_control_core.hpp"

namespace {
bool almost_equal(const double left, const double right) {
  return std::abs(left - right) < 1.0e-12;
}
}  // namespace

int main() {
  using mios_ros2_control::MiosCartesianVelocityControlCore;
  using mios_ros2_control::MiosCartesianVelocityRequest;

  MiosCartesianVelocityControlCore core;
  const MiosCartesianVelocityControlCore::CartesianVelocity limits =
      {0.01, 0.01, 0.01, 0.10, 0.10, 0.10};
  const MiosCartesianVelocityControlCore::CartesianVelocity accelerations =
      {0.05, 0.05, 0.05, 0.20, 0.20, 0.20};
  core.configure(limits, accelerations, 100000000);

  MiosCartesianVelocityControlCore::JointVelocity measured{};
  MiosCartesianVelocityRequest request;
  request.velocity[0] = 1.0;
  request.velocity[3] = 1.0;
  request.received_nanoseconds = 1000;
  const auto first = core.step(measured, false, &request, 1000, 0.1);
  assert(almost_equal(first[0], 0.005));
  assert(almost_equal(first[3], 0.02));
  const auto second = core.step(measured, false, &request, 2000, 0.1);
  assert(almost_equal(second[0], 0.01));
  assert(almost_equal(second[3], 0.04));

  const auto stale = core.step(measured, false, &request, 101000001, 0.1);
  assert(almost_equal(stale[0], 0.0));
  request.received_nanoseconds = 101000002;
  request.user_stopped = true;
  const auto stopped = core.step(measured, false, &request, 101000002, 0.1);
  assert(almost_equal(stopped[3], 0.0));
}
