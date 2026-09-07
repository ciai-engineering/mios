#include "mios_ros2_control/mios_control_core.hpp"

#include <algorithm>
#include <cmath>

namespace {

template <std::size_t Size>
bool all_finite(const std::array<double, Size>& values) {
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}

using Effort = std::array<double, 7>;
using Cartesian = std::array<double, 6>;

Cartesian cartesian_velocity(const std::array<double, 42>& jacobian,
                             const std::array<double, 7>& joint_velocity) {
  Cartesian velocity{};
  for (std::size_t row = 0; row < velocity.size(); ++row) {
    for (std::size_t joint = 0; joint < joint_velocity.size(); ++joint) {
      // Franka's 6x7 Jacobians are column-major.
      velocity[row] += jacobian[row + 6 * joint] * joint_velocity[joint];
    }
  }
  return velocity;
}

Effort project_wrench(const std::array<double, 42>& jacobian, const Cartesian& wrench) {
  Effort effort{};
  for (std::size_t joint = 0; joint < effort.size(); ++joint) {
    for (std::size_t row = 0; row < wrench.size(); ++row) {
      effort[joint] += jacobian[row + 6 * joint] * wrench[row];
    }
  }
  return effort;
}

bool invert_damped_jacobian_product(const std::array<double, 42>& jacobian,
                                    const double damping,
                                    std::array<double, 36>* inverse) {
  std::array<double, 72> augmented{};
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      double value = row == column ? damping * damping : 0.0;
      for (std::size_t joint = 0; joint < 7; ++joint) {
        value += jacobian[row + 6 * joint] * jacobian[column + 6 * joint];
      }
      augmented[row * 12 + column] = value;
      augmented[row * 12 + 6 + column] = row == column ? 1.0 : 0.0;
    }
  }
  for (std::size_t pivot = 0; pivot < 6; ++pivot) {
    std::size_t best_row = pivot;
    for (std::size_t candidate = pivot + 1; candidate < 6; ++candidate) {
      if (std::abs(augmented[candidate * 12 + pivot]) >
          std::abs(augmented[best_row * 12 + pivot])) {
        best_row = candidate;
      }
    }
    if (!std::isfinite(augmented[best_row * 12 + pivot]) ||
        std::abs(augmented[best_row * 12 + pivot]) < 1.0e-12) {
      return false;
    }
    if (best_row != pivot) {
      for (std::size_t column = 0; column < 12; ++column) {
        std::swap(augmented[pivot * 12 + column], augmented[best_row * 12 + column]);
      }
    }
    const double divisor = augmented[pivot * 12 + pivot];
    for (std::size_t column = 0; column < 12; ++column) {
      augmented[pivot * 12 + column] /= divisor;
    }
    for (std::size_t row = 0; row < 6; ++row) {
      if (row == pivot) {
        continue;
      }
      const double factor = augmented[row * 12 + pivot];
      for (std::size_t column = 0; column < 12; ++column) {
        augmented[row * 12 + column] -= factor * augmented[pivot * 12 + column];
      }
    }
  }
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      (*inverse)[row * 6 + column] = augmented[row * 12 + 6 + column];
    }
  }
  return all_finite(*inverse);
}

bool valid_homogeneous_transform(const std::array<double, 16>& transform) {
  if (!all_finite(transform) || std::abs(transform[3]) > 1.0e-6 ||
      std::abs(transform[7]) > 1.0e-6 || std::abs(transform[11]) > 1.0e-6 ||
      std::abs(transform[15] - 1.0) > 1.0e-6) {
    return false;
  }
  for (std::size_t column = 0; column < 3; ++column) {
    double norm = 0.0;
    for (std::size_t row = 0; row < 3; ++row) {
      norm += transform[row + 4 * column] * transform[row + 4 * column];
    }
    if (std::abs(norm - 1.0) > 1.0e-3) {
      return false;
    }
    for (std::size_t other = column + 1; other < 3; ++other) {
      double dot = 0.0;
      for (std::size_t row = 0; row < 3; ++row) {
        dot += transform[row + 4 * column] * transform[row + 4 * other];
      }
      if (std::abs(dot) > 1.0e-3) {
        return false;
      }
    }
  }
  const double determinant =
      transform[0] * (transform[5] * transform[10] - transform[9] * transform[6]) -
      transform[4] * (transform[1] * transform[10] - transform[9] * transform[2]) +
      transform[8] * (transform[1] * transform[6] - transform[5] * transform[2]);
  return determinant > 0.999;
}

Cartesian orientation_error(const std::array<double, 16>& current,
                            const std::array<double, 16>& target) {
  Cartesian error{};
  for (std::size_t column = 0; column < 3; ++column) {
    const double cx = current[4 * column];
    const double cy = current[4 * column + 1];
    const double cz = current[4 * column + 2];
    const double tx = target[4 * column];
    const double ty = target[4 * column + 1];
    const double tz = target[4 * column + 2];
    error[3] += cy * tz - cz * ty;
    error[4] += cz * tx - cx * tz;
    error[5] += cx * ty - cy * tx;
  }
  error[3] *= 0.5;
  error[4] *= 0.5;
  error[5] *= 0.5;
  return error;
}

}  // namespace

namespace mios_ros2_control {

void MiosControlCore::configure(const Effort& effort_limits, double effort_rate_limit,
                                std::int64_t command_timeout_nanoseconds) {
  effort_limits_ = effort_limits;
  effort_rate_limit_ = effort_rate_limit;
  command_timeout_nanoseconds_ = command_timeout_nanoseconds;
  reset();
}

void MiosControlCore::reset() {
  applied_effort_.fill(0.0);
  workspace_safely_armed_ = false;
  position_hold_reference_valid_ = false;
  reset_cartesian_force();
  reset_cartesian_impedance_adaptation();
}

void MiosControlCore::configure_joint_safety(const Effort& lower_limits,
                                             const Effort& upper_limits,
                                             const Effort& stiffness,
                                             const Effort& damping,
                                             const Effort& max_torque, bool enabled) {
  lower_joint_limits_ = lower_limits;
  upper_joint_limits_ = upper_limits;
  wall_stiffness_ = stiffness;
  wall_damping_ = damping;
  wall_max_torque_ = max_torque;
  joint_safety_enabled_ = enabled;
}

void MiosControlCore::configure_position_hold(const Effort& stiffness, const Effort& damping,
                                              const Effort& max_torque, bool enabled) {
  hold_stiffness_ = stiffness;
  hold_damping_ = damping;
  hold_max_torque_ = max_torque;
  position_hold_enabled_ = enabled;
  position_hold_reference_valid_ = false;
}

void MiosControlCore::capture_position_hold_reference(const Effort& position) {
  hold_reference_ = position;
  position_hold_reference_valid_ = all_finite(position);
}

void MiosControlCore::configure_cartesian_velocity_damping(const Cartesian& threshold,
                                                            const Cartesian& damping,
                                                            const Effort& max_torque,
                                                            bool enabled) {
  cartesian_velocity_threshold_ = threshold;
  cartesian_velocity_damping_ = damping;
  cartesian_damping_max_torque_ = max_torque;
  cartesian_velocity_damping_enabled_ = enabled;
}

void MiosControlCore::configure_cartesian_workspace(const Position& lower_limits,
                                                     const Position& upper_limits,
                                                     const Position& stiffness,
                                                     const Position& damping,
                                                     const Position& max_force,
                                                     bool enabled) {
  workspace_lower_limits_ = lower_limits;
  workspace_upper_limits_ = upper_limits;
  workspace_stiffness_ = stiffness;
  workspace_damping_ = damping;
  workspace_max_force_ = max_force;
  cartesian_workspace_enabled_ = enabled;
  workspace_safely_armed_ = false;
}

void MiosControlCore::configure_cartesian_force(
    const Cartesian& proportional_gain, const Cartesian& integral_gain,
    const Cartesian& integral_wrench_limit, const Cartesian& derivative_gain,
    const Cartesian& derivative_filter_time_constant,
    const Cartesian& derivative_wrench_limit, const Cartesian& output_wrench_limit,
    const Cartesian& output_wrench_rate_limit, const bool enabled) {
  force_proportional_gain_ = proportional_gain;
  force_integral_gain_ = integral_gain;
  force_integral_wrench_limit_ = integral_wrench_limit;
  force_derivative_gain_ = derivative_gain;
  force_derivative_filter_time_constant_ = derivative_filter_time_constant;
  force_derivative_wrench_limit_ = derivative_wrench_limit;
  force_output_wrench_limit_ = output_wrench_limit;
  force_output_wrench_rate_limit_ = output_wrench_rate_limit;
  cartesian_force_enabled_ = enabled;
  reset_cartesian_force();
}

void MiosControlCore::reset_cartesian_force() {
  force_integral_wrench_.fill(0.0);
  force_previous_error_.fill(0.0);
  force_filtered_error_rate_.fill(0.0);
  force_applied_wrench_.fill(0.0);
}

bool MiosControlCore::evaluate_joint_impedance(
    const MiosRobotState& state, const MiosJointImpedanceRequest& request,
    Effort& effort) const {
  if (!all_finite(state.position) || !all_finite(state.velocity) ||
      !all_finite(request.position) || !all_finite(request.velocity) ||
      !all_finite(request.feedforward_effort) || !all_finite(request.stiffness) ||
      !all_finite(request.damping)) {
    return false;
  }
  for (std::size_t index = 0; index < effort.size(); ++index) {
    if (request.stiffness[index] < 0.0 || request.damping[index] < 0.0) {
      return false;
    }
    effort[index] = request.feedforward_effort[index] +
                    request.stiffness[index] * (request.position[index] - state.position[index]) +
                    request.damping[index] * (request.velocity[index] - state.velocity[index]);
  }
  return all_finite(effort);
}

void MiosControlCore::configure_cartesian_impedance_adaptation(
    const Cartesian& contact_stiffness_gain, const Cartesian& stiffness_lower_limit,
    const Cartesian& stiffness_upper_limit, const Cartesian& stiffness_rate_limit,
    const bool enabled) {
  adaptation_contact_stiffness_gain_ = contact_stiffness_gain;
  adaptation_stiffness_lower_limit_ = stiffness_lower_limit;
  adaptation_stiffness_upper_limit_ = stiffness_upper_limit;
  adaptation_stiffness_rate_limit_ = stiffness_rate_limit;
  cartesian_impedance_adaptation_enabled_ = enabled;
  reset_cartesian_impedance_adaptation();
}

void MiosControlCore::reset_cartesian_impedance_adaptation() {
  adapted_stiffness_.fill(0.0);
  adapted_stiffness_valid_ = false;
}

bool MiosControlCore::evaluate_cartesian_impedance(
    const MiosRobotState& state, const MiosRobotModel& model,
    const MiosCartesianImpedanceRequest& request, const double period_seconds, Effort& effort) {
  if (!all_finite(state.velocity) || !valid_homogeneous_transform(state.base_to_end_effector) ||
      !valid_homogeneous_transform(request.target_base_to_end_effector) ||
      !all_finite(model.zero_jacobian) || !all_finite(request.target_velocity) ||
      !all_finite(request.feedforward_wrench) || !all_finite(request.stiffness) ||
      !all_finite(request.damping) ||
      (request.use_coriolis_compensation && !all_finite(model.coriolis))) {
    return false;
  }
  if (cartesian_impedance_adaptation_enabled_ &&
      (!std::isfinite(period_seconds) || period_seconds <= 0.0 ||
       !all_finite(state.external_wrench_base) || !all_finite(adaptation_contact_stiffness_gain_) ||
       !all_finite(adaptation_stiffness_lower_limit_) ||
       !all_finite(adaptation_stiffness_upper_limit_) ||
       !all_finite(adaptation_stiffness_rate_limit_))) {
    return false;
  }
  Cartesian active_stiffness = request.stiffness;
  for (std::size_t index = 0; index < request.stiffness.size(); ++index) {
    if (request.stiffness[index] < 0.0 || request.damping[index] < 0.0) {
      return false;
    }
    if (cartesian_impedance_adaptation_enabled_) {
      if (adaptation_contact_stiffness_gain_[index] < 0.0 ||
          adaptation_stiffness_lower_limit_[index] < 0.0 ||
          adaptation_stiffness_lower_limit_[index] > adaptation_stiffness_upper_limit_[index] ||
          adaptation_stiffness_rate_limit_[index] <= 0.0) {
        return false;
      }
      const double requested_stiffness = std::clamp(
          request.stiffness[index] + adaptation_contact_stiffness_gain_[index] *
                                         std::abs(state.external_wrench_base[index]),
          adaptation_stiffness_lower_limit_[index], adaptation_stiffness_upper_limit_[index]);
      if (!adapted_stiffness_valid_) {
        adapted_stiffness_[index] = std::clamp(request.stiffness[index],
                                                adaptation_stiffness_lower_limit_[index],
                                                adaptation_stiffness_upper_limit_[index]);
      }
      const double max_delta = adaptation_stiffness_rate_limit_[index] * period_seconds;
      adapted_stiffness_[index] += std::clamp(requested_stiffness - adapted_stiffness_[index],
                                               -max_delta, max_delta);
      active_stiffness[index] = adapted_stiffness_[index];
    }
  }
  adapted_stiffness_valid_ = cartesian_impedance_adaptation_enabled_;

  Cartesian pose_error = orientation_error(state.base_to_end_effector,
                                            request.target_base_to_end_effector);
  for (std::size_t index = 0; index < 3; ++index) {
    pose_error[index] = request.target_base_to_end_effector[12 + index] -
                        state.base_to_end_effector[12 + index];
  }
  const Cartesian measured_velocity = cartesian_velocity(model.zero_jacobian, state.velocity);
  Cartesian wrench{};
  for (std::size_t index = 0; index < wrench.size(); ++index) {
    wrench[index] = request.feedforward_wrench[index] + active_stiffness[index] * pose_error[index] +
                     request.damping[index] * (request.target_velocity[index] - measured_velocity[index]);
  }
  effort = project_wrench(model.zero_jacobian, wrench);
  if (request.use_coriolis_compensation) {
    for (std::size_t index = 0; index < effort.size(); ++index) {
      effort[index] += model.coriolis[index];
    }
  }
  return all_finite(effort);
}

bool MiosControlCore::evaluate_cartesian_force(
    const MiosRobotState& state, const MiosRobotModel& model,
    const MiosCartesianForceRequest& request, const double period_seconds,
    Effort& effort) {
  if (!cartesian_force_enabled_ || !std::isfinite(period_seconds) || period_seconds <= 0.0 ||
      !all_finite(state.external_wrench_base) || !all_finite(model.zero_jacobian) ||
      !all_finite(request.target_external_wrench) || !all_finite(force_proportional_gain_) ||
      !all_finite(force_integral_gain_) || !all_finite(force_integral_wrench_limit_) ||
      !all_finite(force_derivative_gain_) ||
      !all_finite(force_derivative_filter_time_constant_) ||
      !all_finite(force_derivative_wrench_limit_) ||
      !all_finite(force_output_wrench_limit_) || !all_finite(force_output_wrench_rate_limit_)) {
    return false;
  }
  Cartesian corrective_wrench{};
  for (std::size_t index = 0; index < corrective_wrench.size(); ++index) {
    if (force_proportional_gain_[index] < 0.0 || force_integral_gain_[index] < 0.0 ||
        force_integral_wrench_limit_[index] < 0.0 || force_output_wrench_limit_[index] < 0.0 ||
        force_derivative_gain_[index] < 0.0 ||
        force_derivative_filter_time_constant_[index] < 0.0 ||
        force_derivative_wrench_limit_[index] < 0.0 ||
        force_output_wrench_rate_limit_[index] <= 0.0) {
      return false;
    }
    const double target = std::clamp(request.target_external_wrench[index],
                                     -force_output_wrench_limit_[index],
                                     force_output_wrench_limit_[index]);
    const double error = target - state.external_wrench_base[index];
    force_integral_wrench_[index] = std::clamp(
        force_integral_wrench_[index] + force_integral_gain_[index] * error * period_seconds,
        -force_integral_wrench_limit_[index], force_integral_wrench_limit_[index]);
    const double raw_error_rate = (error - force_previous_error_[index]) / period_seconds;
    const double filter_time = force_derivative_filter_time_constant_[index];
    const double filter_alpha = filter_time == 0.0 ? 1.0 : period_seconds / (filter_time + period_seconds);
    force_filtered_error_rate_[index] +=
        filter_alpha * (raw_error_rate - force_filtered_error_rate_[index]);
    const double derivative_wrench = std::clamp(
        force_derivative_gain_[index] * force_filtered_error_rate_[index],
        -force_derivative_wrench_limit_[index], force_derivative_wrench_limit_[index]);
    force_previous_error_[index] = error;
    corrective_wrench[index] = std::clamp(force_proportional_gain_[index] * error +
                                               force_integral_wrench_[index] + derivative_wrench,
                                           -force_output_wrench_limit_[index],
                                           force_output_wrench_limit_[index]);
    const double max_delta = force_output_wrench_rate_limit_[index] * period_seconds;
    force_applied_wrench_[index] += std::clamp(corrective_wrench[index] - force_applied_wrench_[index],
                                                -max_delta, max_delta);
  }
  effort = project_wrench(model.zero_jacobian, force_applied_wrench_);
  return all_finite(effort);
}

void MiosControlCore::configure_nullspace(const double singularity_damping,
                                          const Effort& effort_limit,
                                          const bool enabled) {
  nullspace_singularity_damping_ = singularity_damping;
  nullspace_effort_limit_ = effort_limit;
  nullspace_enabled_ = enabled;
}

bool MiosControlCore::evaluate_nullspace(const MiosRobotState& state,
                                         const MiosRobotModel& model,
                                         const MiosNullspaceRequest& request,
                                         Effort& effort) const {
  if (!nullspace_enabled_ || !std::isfinite(nullspace_singularity_damping_) ||
      nullspace_singularity_damping_ <= 0.0 || !all_finite(state.position) ||
      !all_finite(state.velocity) || !all_finite(model.zero_jacobian) ||
      !all_finite(request.position) || !all_finite(request.stiffness) ||
      !all_finite(request.damping) || !all_finite(nullspace_effort_limit_)) {
    return false;
  }
  Effort candidate{};
  for (std::size_t joint = 0; joint < candidate.size(); ++joint) {
    if (request.stiffness[joint] < 0.0 || request.damping[joint] < 0.0 ||
        nullspace_effort_limit_[joint] < 0.0) {
      return false;
    }
    candidate[joint] = std::clamp(request.stiffness[joint] *
                                       (request.position[joint] - state.position[joint]) -
                                   request.damping[joint] * state.velocity[joint],
                                   -nullspace_effort_limit_[joint],
                                   nullspace_effort_limit_[joint]);
  }
  std::array<double, 36> inverse{};
  if (!invert_damped_jacobian_product(model.zero_jacobian, nullspace_singularity_damping_,
                                      &inverse)) {
    return false;
  }
  Cartesian task_component{};
  for (std::size_t row = 0; row < task_component.size(); ++row) {
    for (std::size_t joint = 0; joint < candidate.size(); ++joint) {
      task_component[row] += model.zero_jacobian[row + 6 * joint] * candidate[joint];
    }
  }
  Cartesian solved_component{};
  for (std::size_t row = 0; row < solved_component.size(); ++row) {
    for (std::size_t column = 0; column < task_component.size(); ++column) {
      solved_component[row] += inverse[row * 6 + column] * task_component[column];
    }
  }
  for (std::size_t joint = 0; joint < effort.size(); ++joint) {
    effort[joint] = candidate[joint];
    for (std::size_t row = 0; row < solved_component.size(); ++row) {
      effort[joint] -= model.zero_jacobian[row + 6 * joint] * solved_component[row];
    }
    effort[joint] = std::clamp(effort[joint], -nullspace_effort_limit_[joint],
                                nullspace_effort_limit_[joint]);
  }
  return all_finite(effort);
}

MiosControlCore::Effort MiosControlCore::step(const MiosRobotState& state,
                                               const MiosRobotModel& model,
                                               const MiosTorqueRequest* request,
                                               std::int64_t now_nanoseconds,
                                               double period_seconds) {
  Effort desired{};
  const bool valid_period = std::isfinite(period_seconds) && period_seconds > 0.0;
  const bool valid_state = all_finite(state.position) && all_finite(state.velocity) &&
                           all_finite(state.effort);
  const bool valid_model =
      (!cartesian_velocity_damping_enabled_ || all_finite(model.body_jacobian)) &&
      (!cartesian_workspace_enabled_ || all_finite(model.zero_jacobian));
  const bool request_is_fresh = request != nullptr && request->received_nanoseconds > 0 &&
                                now_nanoseconds >= request->received_nanoseconds &&
                                now_nanoseconds - request->received_nanoseconds <=
                                    command_timeout_nanoseconds_;
  const bool valid_request = request == nullptr || all_finite(request->effort);
  if (state.user_stopped || !valid_period || !valid_state || !valid_model || !valid_request ||
      (request != nullptr && request->user_stopped) ||
      (position_hold_enabled_ && !position_hold_reference_valid_)) {
    // These conditions are fail-safe, not command transitions: do not let a
    // torque-rate limiter delay removal of an unsafe command.
    applied_effort_.fill(0.0);
    return applied_effort_;
  }

  if (position_hold_enabled_) {
    for (std::size_t index = 0; index < desired.size(); ++index) {
      const double hold_torque = hold_stiffness_[index] *
                                     (hold_reference_[index] - state.position[index]) -
                                 hold_damping_[index] * state.velocity[index];
      desired[index] = std::clamp(hold_torque, -hold_max_torque_[index],
                                  hold_max_torque_[index]);
    }
  }

  if (request_is_fresh) {
    for (std::size_t index = 0; index < desired.size(); ++index) {
      desired[index] += request->effort[index];
    }
  }

  if (cartesian_velocity_damping_enabled_) {
    const Cartesian velocity = cartesian_velocity(model.body_jacobian, state.velocity);
    Cartesian damping_wrench{};
    for (std::size_t index = 0; index < damping_wrench.size(); ++index) {
      const double speed = std::abs(velocity[index]);
      if (speed > cartesian_velocity_threshold_[index]) {
        damping_wrench[index] = -std::copysign(
            (speed - cartesian_velocity_threshold_[index]) * cartesian_velocity_damping_[index],
            velocity[index]);
      }
    }
    const Effort damping_effort = project_wrench(model.body_jacobian, damping_wrench);
    for (std::size_t index = 0; index < desired.size(); ++index) {
      desired[index] += std::clamp(damping_effort[index], -cartesian_damping_max_torque_[index],
                                   cartesian_damping_max_torque_[index]);
    }
  }

  if (cartesian_workspace_enabled_) {
    const bool valid_pose = all_finite(state.base_to_end_effector);
    if (!valid_pose) {
      applied_effort_.fill(0.0);
      return applied_effort_;
    }
    const Position position = {state.base_to_end_effector[12], state.base_to_end_effector[13],
                               state.base_to_end_effector[14]};
    bool inside_workspace = true;
    for (std::size_t index = 0; index < position.size(); ++index) {
      inside_workspace = inside_workspace && position[index] >= workspace_lower_limits_[index] &&
                         position[index] <= workspace_upper_limits_[index];
    }
    workspace_safely_armed_ = workspace_safely_armed_ || inside_workspace;
    if (workspace_safely_armed_) {
      const Cartesian base_velocity = cartesian_velocity(model.zero_jacobian, state.velocity);
      Cartesian workspace_wrench{};
      for (std::size_t index = 0; index < position.size(); ++index) {
        double force = 0.0;
        if (position[index] < workspace_lower_limits_[index]) {
          force = workspace_stiffness_[index] * (workspace_lower_limits_[index] - position[index]) -
                  workspace_damping_[index] * base_velocity[index];
        } else if (position[index] > workspace_upper_limits_[index]) {
          force = workspace_stiffness_[index] * (workspace_upper_limits_[index] - position[index]) -
                  workspace_damping_[index] * base_velocity[index];
        }
        workspace_wrench[index] =
            std::clamp(force, -workspace_max_force_[index], workspace_max_force_[index]);
      }
      const Effort workspace_effort = project_wrench(model.zero_jacobian, workspace_wrench);
      for (std::size_t index = 0; index < desired.size(); ++index) {
        desired[index] += workspace_effort[index];
      }
    }
  }

  if (joint_safety_enabled_) {
    for (std::size_t index = 0; index < desired.size(); ++index) {
      double wall_torque = 0.0;
      if (state.position[index] < lower_joint_limits_[index]) {
        wall_torque = wall_stiffness_[index] * (lower_joint_limits_[index] - state.position[index]) -
                      wall_damping_[index] * state.velocity[index];
      } else if (state.position[index] > upper_joint_limits_[index]) {
        wall_torque = wall_stiffness_[index] * (upper_joint_limits_[index] - state.position[index]) -
                      wall_damping_[index] * state.velocity[index];
      }
      desired[index] += std::clamp(wall_torque, -wall_max_torque_[index], wall_max_torque_[index]);
    }
  }

  const double max_delta = valid_period ? effort_rate_limit_ * period_seconds : 0.0;
  for (std::size_t index = 0; index < applied_effort_.size(); ++index) {
    const double limited = std::clamp(desired[index], -effort_limits_[index], effort_limits_[index]);
    applied_effort_[index] +=
        std::clamp(limited - applied_effort_[index], -max_delta, max_delta);
  }
  return applied_effort_;
}

}  // namespace mios_ros2_control
