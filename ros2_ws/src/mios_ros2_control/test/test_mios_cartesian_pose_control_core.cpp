#include <cassert>
#include <cmath>
#include <cstdint>

#include "mios_ros2_control/mios_cartesian_pose_control_core.hpp"

namespace {

using Core = mios_ros2_control::MiosCartesianPoseControlCore;

bool nearly_equal(const double left, const double right) {
  return std::abs(left - right) < 1.0e-10;
}

Core::Pose identity_pose() {
  return {1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0};
}

}  // namespace

int main() {
  Core core;
  Core::Translation lower{-1.0, -1.0, -1.0};
  Core::Translation upper{1.0, 1.0, 1.0};
  core.configure(lower, upper, 0.2, 0.2, 100'000'000);

  const auto measured = identity_pose();
  core.capture_hold_reference(measured);
  mios_ros2_control::MiosCartesianPoseRequest request;
  request.base_to_end_effector = {0.0, 1.0, 0.0, 0.0,
                                  -1.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 1.0, 0.0,
                                  10.0, 0.0, 0.0, 1.0};
  request.received_nanoseconds = 1'000;
  assert(Core::is_valid_pose(request.base_to_end_effector));

  const auto first = core.step(measured, false, &request, 1'000, 0.1);
  assert(Core::is_valid_pose(first));
  assert(nearly_equal(first[12], 0.02));
  assert(nearly_equal(std::atan2(first[1], first[0]), 0.02));

  auto stale_measurement = identity_pose();
  stale_measurement[12] = 0.37;
  const auto stale = core.step(stale_measurement, false, &request, 101'000'001, 0.1);
  assert(nearly_equal(stale[12], 0.37));

  request.base_to_end_effector[15] = 0.0;
  request.received_nanoseconds = 101'000'002;
  const auto invalid = core.step(stale_measurement, false, &request, 101'000'002, 0.1);
  assert(nearly_equal(invalid[12], 0.37));
  return 0;
}
