#include "mios_ros2_control/mios_cartesian_pose_control_core.hpp"

#include <algorithm>
#include <cmath>

namespace mios_ros2_control {
namespace {

constexpr double kPoseTolerance = 1.0e-6;
constexpr double kSmallAngle = 1.0e-12;

struct Quaternion {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

bool is_finite(const double value) { return std::isfinite(value); }

bool all_finite(const MiosCartesianPoseControlCore::Pose& pose) {
  return std::all_of(pose.begin(), pose.end(), is_finite);
}

double rotation_at(const MiosCartesianPoseControlCore::Pose& pose,
                   const std::size_t row, const std::size_t column) {
  return pose[column * 4 + row];
}

void set_rotation(MiosCartesianPoseControlCore::Pose* pose, const std::size_t row,
                  const std::size_t column, const double value) {
  (*pose)[column * 4 + row] = value;
}

double dot(const Quaternion& left, const Quaternion& right) {
  return left.x * right.x + left.y * right.y + left.z * right.z + left.w * right.w;
}

Quaternion normalized(Quaternion quaternion) {
  const double norm = std::sqrt(dot(quaternion, quaternion));
  if (!is_finite(norm) || norm <= kSmallAngle) {
    return {};
  }
  quaternion.x /= norm;
  quaternion.y /= norm;
  quaternion.z /= norm;
  quaternion.w /= norm;
  return quaternion;
}

Quaternion quaternion_from_rotation(const MiosCartesianPoseControlCore::Pose& pose) {
  const double r00 = rotation_at(pose, 0, 0);
  const double r11 = rotation_at(pose, 1, 1);
  const double r22 = rotation_at(pose, 2, 2);
  const double trace = r00 + r11 + r22;
  Quaternion result;
  if (trace > 0.0) {
    const double scale = 2.0 * std::sqrt(trace + 1.0);
    result.w = 0.25 * scale;
    result.x = (rotation_at(pose, 2, 1) - rotation_at(pose, 1, 2)) / scale;
    result.y = (rotation_at(pose, 0, 2) - rotation_at(pose, 2, 0)) / scale;
    result.z = (rotation_at(pose, 1, 0) - rotation_at(pose, 0, 1)) / scale;
  } else if (r00 > r11 && r00 > r22) {
    const double scale = 2.0 * std::sqrt(1.0 + r00 - r11 - r22);
    result.w = (rotation_at(pose, 2, 1) - rotation_at(pose, 1, 2)) / scale;
    result.x = 0.25 * scale;
    result.y = (rotation_at(pose, 0, 1) + rotation_at(pose, 1, 0)) / scale;
    result.z = (rotation_at(pose, 0, 2) + rotation_at(pose, 2, 0)) / scale;
  } else if (r11 > r22) {
    const double scale = 2.0 * std::sqrt(1.0 + r11 - r00 - r22);
    result.w = (rotation_at(pose, 0, 2) - rotation_at(pose, 2, 0)) / scale;
    result.x = (rotation_at(pose, 0, 1) + rotation_at(pose, 1, 0)) / scale;
    result.y = 0.25 * scale;
    result.z = (rotation_at(pose, 1, 2) + rotation_at(pose, 2, 1)) / scale;
  } else {
    const double scale = 2.0 * std::sqrt(1.0 + r22 - r00 - r11);
    result.w = (rotation_at(pose, 1, 0) - rotation_at(pose, 0, 1)) / scale;
    result.x = (rotation_at(pose, 0, 2) + rotation_at(pose, 2, 0)) / scale;
    result.y = (rotation_at(pose, 1, 2) + rotation_at(pose, 2, 1)) / scale;
    result.z = 0.25 * scale;
  }
  return normalized(result);
}

void set_rotation_from_quaternion(MiosCartesianPoseControlCore::Pose* pose,
                                  const Quaternion& raw_quaternion) {
  const Quaternion quaternion = normalized(raw_quaternion);
  const double xx = quaternion.x * quaternion.x;
  const double yy = quaternion.y * quaternion.y;
  const double zz = quaternion.z * quaternion.z;
  const double xy = quaternion.x * quaternion.y;
  const double xz = quaternion.x * quaternion.z;
  const double yz = quaternion.y * quaternion.z;
  const double wx = quaternion.w * quaternion.x;
  const double wy = quaternion.w * quaternion.y;
  const double wz = quaternion.w * quaternion.z;
  set_rotation(pose, 0, 0, 1.0 - 2.0 * (yy + zz));
  set_rotation(pose, 0, 1, 2.0 * (xy - wz));
  set_rotation(pose, 0, 2, 2.0 * (xz + wy));
  set_rotation(pose, 1, 0, 2.0 * (xy + wz));
  set_rotation(pose, 1, 1, 1.0 - 2.0 * (xx + zz));
  set_rotation(pose, 1, 2, 2.0 * (yz - wx));
  set_rotation(pose, 2, 0, 2.0 * (xz - wy));
  set_rotation(pose, 2, 1, 2.0 * (yz + wx));
  set_rotation(pose, 2, 2, 1.0 - 2.0 * (xx + yy));
}

Quaternion slerp_limited(const Quaternion& current, Quaternion target,
                         const double max_angle) {
  Quaternion source = normalized(current);
  target = normalized(target);
  double cosine = dot(source, target);
  if (cosine < 0.0) {
    target.x = -target.x;
    target.y = -target.y;
    target.z = -target.z;
    target.w = -target.w;
    cosine = -cosine;
  }
  cosine = std::clamp(cosine, -1.0, 1.0);
  const double angle = 2.0 * std::acos(cosine);
  if (angle <= kSmallAngle || max_angle >= angle) {
    return target;
  }
  const double fraction = std::clamp(max_angle / angle, 0.0, 1.0);
  if (cosine > 1.0 - 1.0e-9) {
    return normalized({source.x + fraction * (target.x - source.x),
                       source.y + fraction * (target.y - source.y),
                       source.z + fraction * (target.z - source.z),
                       source.w + fraction * (target.w - source.w)});
  }
  const double sine = std::sqrt(1.0 - cosine * cosine);
  const double source_weight = std::sin((1.0 - fraction) * angle / 2.0) / sine;
  const double target_weight = std::sin(fraction * angle / 2.0) / sine;
  return normalized({source_weight * source.x + target_weight * target.x,
                     source_weight * source.y + target_weight * target.y,
                     source_weight * source.z + target_weight * target.z,
                     source_weight * source.w + target_weight * target.w});
}

}  // namespace

void MiosCartesianPoseControlCore::configure(
    const Translation& lower_workspace_limits, const Translation& upper_workspace_limits,
    const double linear_velocity_limit, const double angular_velocity_limit,
    const std::int64_t command_timeout_nanoseconds) {
  lower_workspace_limits_ = lower_workspace_limits;
  upper_workspace_limits_ = upper_workspace_limits;
  linear_velocity_limit_ = linear_velocity_limit;
  angular_velocity_limit_ = angular_velocity_limit;
  command_timeout_nanoseconds_ = command_timeout_nanoseconds;
  reset();
}

void MiosCartesianPoseControlCore::reset() {
  applied_pose_.fill(0.0);
  hold_reference_valid_ = false;
}

void MiosCartesianPoseControlCore::capture_hold_reference(const Pose& pose) {
  applied_pose_ = pose;
  hold_reference_valid_ = is_valid_pose(pose);
}

bool MiosCartesianPoseControlCore::is_valid_pose(const Pose& pose) {
  if (!all_finite(pose) || std::abs(pose[3]) > kPoseTolerance ||
      std::abs(pose[7]) > kPoseTolerance || std::abs(pose[11]) > kPoseTolerance ||
      std::abs(pose[15] - 1.0) > kPoseTolerance) {
    return false;
  }
  for (std::size_t column = 0; column < 3; ++column) {
    double norm_squared = 0.0;
    for (std::size_t row = 0; row < 3; ++row) {
      norm_squared += rotation_at(pose, row, column) * rotation_at(pose, row, column);
    }
    if (std::abs(norm_squared - 1.0) > kPoseTolerance) {
      return false;
    }
  }
  for (std::size_t left_column = 0; left_column < 3; ++left_column) {
    for (std::size_t right_column = left_column + 1; right_column < 3; ++right_column) {
      double column_dot = 0.0;
      for (std::size_t row = 0; row < 3; ++row) {
        column_dot += rotation_at(pose, row, left_column) * rotation_at(pose, row, right_column);
      }
      if (std::abs(column_dot) > kPoseTolerance) {
        return false;
      }
    }
  }
  const double determinant =
      rotation_at(pose, 0, 0) *
          (rotation_at(pose, 1, 1) * rotation_at(pose, 2, 2) -
           rotation_at(pose, 1, 2) * rotation_at(pose, 2, 1)) -
      rotation_at(pose, 0, 1) *
          (rotation_at(pose, 1, 0) * rotation_at(pose, 2, 2) -
           rotation_at(pose, 1, 2) * rotation_at(pose, 2, 0)) +
      rotation_at(pose, 0, 2) *
          (rotation_at(pose, 1, 0) * rotation_at(pose, 2, 1) -
           rotation_at(pose, 1, 1) * rotation_at(pose, 2, 0));
  return std::abs(determinant - 1.0) <= kPoseTolerance;
}

MiosCartesianPoseControlCore::Pose MiosCartesianPoseControlCore::step(
    const Pose& measured_pose, const bool user_stopped,
    const MiosCartesianPoseRequest* request, const std::int64_t now_nanoseconds,
    const double period_seconds) {
  const bool valid_period = is_finite(period_seconds) && period_seconds > 0.0;
  const bool request_is_fresh = request != nullptr && request->received_nanoseconds > 0 &&
                                now_nanoseconds >= request->received_nanoseconds &&
                                now_nanoseconds - request->received_nanoseconds <=
                                    command_timeout_nanoseconds_;
  if (!is_valid_pose(measured_pose) || user_stopped || !valid_period ||
      request == nullptr || request->user_stopped || !request_is_fresh ||
      !is_valid_pose(request->base_to_end_effector) || !hold_reference_valid_) {
    applied_pose_ = measured_pose;
    hold_reference_valid_ = is_valid_pose(measured_pose);
    return applied_pose_;
  }

  Pose desired_pose = request->base_to_end_effector;
  for (std::size_t index = 0; index < lower_workspace_limits_.size(); ++index) {
    desired_pose[12 + index] = std::clamp(desired_pose[12 + index], lower_workspace_limits_[index],
                                           upper_workspace_limits_[index]);
    const double difference = desired_pose[12 + index] - applied_pose_[12 + index];
    const double max_delta = linear_velocity_limit_ * period_seconds;
    applied_pose_[12 + index] += std::clamp(difference, -max_delta, max_delta);
  }
  set_rotation_from_quaternion(&applied_pose_,
                               slerp_limited(quaternion_from_rotation(applied_pose_),
                                             quaternion_from_rotation(desired_pose),
                                             angular_velocity_limit_ * period_seconds));
  applied_pose_[3] = 0.0;
  applied_pose_[7] = 0.0;
  applied_pose_[11] = 0.0;
  applied_pose_[15] = 1.0;
  return applied_pose_;
}

}  // namespace mios_ros2_control
