#pragma once

#include "mios/controller_pipeline/controller_pipeline.hpp"

#include "Eigen/Core"

#include <set>

namespace mios {

namespace detail {

// These deterministic helpers are kept separate from ROS transport and from
// the larger legacy controller library so their safety-critical trajectory
// conversion can be tested without a robot or FCI connection.
Eigen::Matrix<double, 7, 1> integrate_joint_position_target(
    const Eigen::Matrix<double, 7, 1>& previous_target,
    const Eigen::Matrix<double, 7, 1>& desired_velocity,
    const Eigen::Matrix<double, 7, 1>& lower_limits,
    const Eigen::Matrix<double, 7, 1>& upper_limits);

Eigen::Matrix<double, 7, 1> clamp_joint_position_target(
    const Eigen::Matrix<double, 7, 1>& target,
    const Eigen::Matrix<double, 7, 1>& lower_limits,
    const Eigen::Matrix<double, 7, 1>& upper_limits);

Eigen::Matrix<double, 7, 1> next_joint_position_target(
    const std::set<CommandPattern>& command_pattern,
    const Eigen::Matrix<double, 7, 1>& previous_target,
    const Eigen::Matrix<double, 7, 1>& entry_pose,
    const Eigen::Matrix<double, 7, 1>& requested_position,
    const Eigen::Matrix<double, 7, 1>& requested_velocity,
    const Eigen::Matrix<double, 7, 1>& lower_limits,
    const Eigen::Matrix<double, 7, 1>& upper_limits);

bool is_valid_joint_position_command(const control::ArmCommand& command);

}  // namespace detail

// Converts the existing MIOS joint-pose and joint-velocity strategy outputs
// into a continuous joint-position command.  MoveToJointPose emits a bounded
// velocity profile, so this pipeline integrates that profile from the current
// measured pose rather than sending a discontinuous target position.
class JointPositionControllerPipeline : public ControllerPipeline {
 public:
  JointPositionControllerPipeline();

  void initialize(const Percept& p_0, const control::ControlRuntimeConfig& config) override;
  control::ArmCommand step(const Percept& p, const Actuator& cmd) override;
  bool is_valid_command(const control::ArmCommand& cmd) const override;
  void update_percept(Percept::Controller& p) override;
  void terminate() override;
  void context_switch(const Percept& p) override;

 private:
  control::ArmCommand m_command{};
  Eigen::Matrix<double, 7, 1> m_q_d;
  Eigen::Matrix<double, 7, 1> m_q_0;
  Eigen::Matrix<double, 7, 1> m_q_lower;
  Eigen::Matrix<double, 7, 1> m_q_upper;
};

}  // namespace mios
