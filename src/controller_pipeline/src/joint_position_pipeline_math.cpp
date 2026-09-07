#include "mios/controller_pipeline/joint_position_pipeline.hpp"

#include <algorithm>
#include <cmath>

namespace mios::detail {
namespace {

constexpr double kControlPeriodSeconds = 0.001;

}  // namespace

Eigen::Matrix<double, 7, 1> clamp_joint_position_target(
    const Eigen::Matrix<double, 7, 1>& target,
    const Eigen::Matrix<double, 7, 1>& lower_limits,
    const Eigen::Matrix<double, 7, 1>& upper_limits) {
  Eigen::Matrix<double, 7, 1> bounded_target = target;
  for (Eigen::Index index = 0; index < bounded_target.size(); ++index) {
    bounded_target(index) =
        std::clamp(bounded_target(index), lower_limits(index), upper_limits(index));
  }
  return bounded_target;
}

Eigen::Matrix<double, 7, 1> integrate_joint_position_target(
    const Eigen::Matrix<double, 7, 1>& previous_target,
    const Eigen::Matrix<double, 7, 1>& desired_velocity,
    const Eigen::Matrix<double, 7, 1>& lower_limits,
    const Eigen::Matrix<double, 7, 1>& upper_limits) {
  return clamp_joint_position_target(previous_target + desired_velocity * kControlPeriodSeconds,
                                     lower_limits, upper_limits);
}

Eigen::Matrix<double, 7, 1> next_joint_position_target(
    const std::set<CommandPattern>& command_pattern,
    const Eigen::Matrix<double, 7, 1>& previous_target,
    const Eigen::Matrix<double, 7, 1>& entry_pose,
    const Eigen::Matrix<double, 7, 1>& requested_position,
    const Eigen::Matrix<double, 7, 1>& requested_velocity,
    const Eigen::Matrix<double, 7, 1>& lower_limits,
    const Eigen::Matrix<double, 7, 1>& upper_limits) {
  if (command_pattern.find(CommandPatternJointPose) != command_pattern.end()) {
    return clamp_joint_position_target(requested_position, lower_limits, upper_limits);
  }
  if (command_pattern.find(CommandPatternJointVelocities) != command_pattern.end()) {
    return integrate_joint_position_target(previous_target, requested_velocity, lower_limits,
                                           upper_limits);
  }
  return entry_pose;
}

bool is_valid_joint_position_command(const control::ArmCommand& command) {
  if (command.mode != control::CommandMode::kJointPosition) {
    return false;
  }
  for (const double joint : command.joints) {
    if (!std::isfinite(joint)) {
      return false;
    }
  }
  return true;
}

}  // namespace mios::detail
