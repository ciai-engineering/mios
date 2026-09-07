#include "mios_ros2_runtime/robot_parameter_snapshot.hpp"

#include <algorithm>
#include <cmath>

namespace mios_ros2_runtime {
namespace {

template <std::size_t Size>
bool finite_values(const std::array<double, Size>& values) {
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

template <std::size_t Size>
bool nonnegative_values(const std::array<double, Size>& values) {
  return finite_values(values) &&
         std::all_of(values.cbegin(), values.cend(), [](const double value) { return value >= 0.0; });
}

bool valid_transform(const std::array<double, 16>& transform) {
  if (!finite_values(transform) || std::abs(transform[3]) > 1.0e-9 ||
      std::abs(transform[7]) > 1.0e-9 || std::abs(transform[11]) > 1.0e-9 ||
      std::abs(transform[15] - 1.0) > 1.0e-9) {
    return false;
  }
  // The first three columns are a column-major rotation matrix.
  for (std::size_t column = 0; column < 3; ++column) {
    double squared_norm = 0.0;
    for (std::size_t row = 0; row < 3; ++row) {
      squared_norm += transform[column * 4 + row] * transform[column * 4 + row];
    }
    if (std::abs(squared_norm - 1.0) > 1.0e-6) {
      return false;
    }
  }
  for (std::size_t left = 0; left < 3; ++left) {
    for (std::size_t right = left + 1; right < 3; ++right) {
      double dot_product = 0.0;
      for (std::size_t row = 0; row < 3; ++row) {
        dot_product += transform[left * 4 + row] * transform[right * 4 + row];
      }
      if (std::abs(dot_product) > 1.0e-6) {
        return false;
      }
    }
  }
  return true;
}

template <std::size_t Size>
bool ordered_thresholds(const std::array<double, Size>& lower,
                        const std::array<double, Size>& upper) {
  if (!nonnegative_values(lower) || !nonnegative_values(upper)) {
    return false;
  }
  for (std::size_t index = 0; index < Size; ++index) {
    if (lower[index] > upper[index]) {
      return false;
    }
  }
  return true;
}

}  // namespace

bool valid_robot_parameter_snapshot(const RobotParameterSnapshot& snapshot) {
  if (!std::isfinite(snapshot.load_mass) || snapshot.load_mass < 0.0 ||
      !finite_values(snapshot.load_center_of_mass) || !finite_values(snapshot.load_inertia) ||
      !valid_transform(snapshot.tcp_frame) || !valid_transform(snapshot.stiffness_frame) ||
      !nonnegative_values(snapshot.joint_stiffness) ||
      !nonnegative_values(snapshot.cartesian_stiffness) ||
      !ordered_thresholds(snapshot.lower_torque_thresholds, snapshot.upper_torque_thresholds) ||
      !ordered_thresholds(snapshot.lower_force_thresholds, snapshot.upper_force_thresholds)) {
    return false;
  }
  // A negative principal inertia cannot represent a physical payload. Full
  // positive-semidefinite validation is deliberately left to libfranka.
  return snapshot.load_inertia[0] >= 0.0 && snapshot.load_inertia[4] >= 0.0 &&
         snapshot.load_inertia[8] >= 0.0;
}

}  // namespace mios_ros2_runtime
