#include <cassert>
#include <cmath>
#include <limits>
#include <set>

#include "mios/controller_pipeline/joint_position_pipeline.hpp"
namespace {

constexpr double kEpsilon = 1.0e-12;

void expect_joint_values(const mios::control::ArmCommand& command,
                         const Eigen::Matrix<double, 7, 1>& expected) {
  assert(command.mode == mios::control::CommandMode::kJointPosition);
  for (std::size_t index = 0; index < command.joints.size(); ++index) {
    assert(std::abs(command.joints[index] - expected(static_cast<Eigen::Index>(index))) <
           kEpsilon);
  }
}

}  // namespace

int main() {
  Eigen::Matrix<double, 7, 1> current;
  current << -0.4, -0.3, -0.2, -0.1, 0.1, 0.2, 0.3;
  Eigen::Matrix<double, 7, 1> lower;
  lower << -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0;
  Eigen::Matrix<double, 7, 1> upper;
  upper << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  // A velocity profile begins at the measured pose and advances exactly one
  // 1 kHz control period.  This is the path emitted by MoveToJointPose.
  const auto first_position = mios::detail::integrate_joint_position_target(
      current, Eigen::Matrix<double, 7, 1>::Constant(0.05), lower, upper);
  const auto expected_first = current +
                              Eigen::Matrix<double, 7, 1>::Constant(0.00005);
  mios::control::ArmCommand first_command;
  first_command.mode = mios::control::CommandMode::kJointPosition;
  for (std::size_t index = 0; index < first_command.joints.size(); ++index) {
    first_command.joints[index] = first_position(static_cast<Eigen::Index>(index));
  }
  expect_joint_values(first_command, expected_first);
  assert(mios::detail::is_valid_joint_position_command(first_command));

  // An explicit position command is forwarded, but cannot escape the FR3
  // joint-position bounds held in the runtime configuration.
  auto requested_pose = upper;
  requested_pose(0) += 1.0;
  const auto bounded_pose =
      mios::detail::clamp_joint_position_target(requested_pose, lower, upper);
  for (std::size_t index = 0; index < first_command.joints.size(); ++index) {
    first_command.joints[index] = bounded_pose(static_cast<Eigen::Index>(index));
  }
  expect_joint_values(first_command, upper);

  // Any non-joint command pattern returns to the entry pose rather than
  // retaining the prior target from a completed primitive.
  const auto held_position = mios::detail::next_joint_position_target(
      {mios::CommandPatternIdle}, bounded_pose, current, requested_pose,
      Eigen::Matrix<double, 7, 1>::Constant(0.2), lower, upper);
  for (std::size_t index = 0; index < first_command.joints.size(); ++index) {
    first_command.joints[index] = held_position(static_cast<Eigen::Index>(index));
  }
  expect_joint_values(first_command, current);

  auto invalid = first_command;
  invalid.joints[3] = std::numeric_limits<double>::infinity();
  assert(!mios::detail::is_valid_joint_position_command(invalid));
  return 0;
}
