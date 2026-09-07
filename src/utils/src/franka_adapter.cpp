#include "mios/control/franka_adapter.hpp"

#include <stdexcept>

namespace mios::control {
namespace {

void require_mode(const ArmCommand& command, CommandMode expected) {
  if (command.mode != expected) {
    throw std::invalid_argument("MIOS arm command mode does not match the requested backend interface");
  }
}

RobotMode from_franka_robot_mode(const franka::RobotMode mode) {
  switch (mode) {
    case franka::RobotMode::kIdle:
      return RobotMode::kIdle;
    case franka::RobotMode::kMove:
      return RobotMode::kMove;
    case franka::RobotMode::kGuiding:
      return RobotMode::kGuiding;
    case franka::RobotMode::kReflex:
      return RobotMode::kReflex;
    case franka::RobotMode::kUserStopped:
      return RobotMode::kUserStopped;
    case franka::RobotMode::kAutomaticErrorRecovery:
      return RobotMode::kAutomaticErrorRecovery;
    case franka::RobotMode::kOther:
      return RobotMode::kOther;
  }
  return RobotMode::kOther;
}

}  // namespace

RobotState from_franka_state(const franka::RobotState& state) {
  RobotState result;
  result.position = state.q;
  result.velocity = state.dq;
  result.motor_position = state.theta;
  result.motor_velocity = state.dtheta;
  result.effort = state.tau_J;
  result.external_effort = state.tau_ext_hat_filtered;
  result.base_to_end_effector = state.O_T_EE;
  result.external_wrench_origin = state.O_F_ext_hat_K;
  result.external_wrench_stiffness = state.K_F_ext_hat_K;
  result.robot_mode = from_franka_robot_mode(state.robot_mode);
  result.user_stopped = result.robot_mode == RobotMode::kUserStopped;
  return result;
}

GripperState from_franka_state(const franka::GripperState& state) {
  return {state.width, state.max_width, static_cast<double>(state.temperature), state.is_grasped};
}

RobotModel from_franka_model(const franka::Model& model, const franka::RobotState& state) {
  RobotModel result;
  result.mass = model.mass(state);
  result.coriolis = model.coriolis(state);
  result.gravity = model.gravity(state);
  result.body_jacobian = model.bodyJacobian(franka::Frame::kEndEffector, state);
  result.zero_jacobian = model.zeroJacobian(franka::Frame::kEndEffector, state);
  return result;
}

ArmCommand from_franka_command(const franka::Torques& command) {
  ArmCommand result;
  result.mode = CommandMode::kTorque;
  result.joints = command.tau_J;
  result.motion_finished = command.motion_finished;
  return result;
}

ArmCommand from_franka_command(const franka::JointVelocities& command) {
  ArmCommand result;
  result.mode = CommandMode::kJointVelocity;
  result.joints = command.dq;
  result.motion_finished = command.motion_finished;
  return result;
}

ArmCommand from_franka_command(const franka::CartesianVelocities& command) {
  ArmCommand result;
  result.mode = CommandMode::kCartesianVelocity;
  result.cartesian = command.O_dP_EE;
  result.motion_finished = command.motion_finished;
  return result;
}

franka::Torques to_franka_torques(const ArmCommand& command) {
  require_mode(command, CommandMode::kTorque);
  auto result = franka::Torques(command.joints);
  result.motion_finished = command.motion_finished;
  return result;
}

franka::JointVelocities to_franka_joint_velocities(const ArmCommand& command) {
  require_mode(command, CommandMode::kJointVelocity);
  auto result = franka::JointVelocities(command.joints);
  result.motion_finished = command.motion_finished;
  return result;
}

franka::CartesianVelocities to_franka_cartesian_velocities(const ArmCommand& command) {
  require_mode(command, CommandMode::kCartesianVelocity);
  auto result = franka::CartesianVelocities(command.cartesian);
  result.motion_finished = command.motion_finished;
  return result;
}

franka::JointPositions to_franka_joint_positions(const ArmCommand& command) {
  require_mode(command, CommandMode::kJointPosition);
  auto result = franka::JointPositions(command.joints);
  result.motion_finished = command.motion_finished;
  return result;
}

franka::CartesianPose to_franka_cartesian_pose(const ArmCommand& command) {
  require_mode(command, CommandMode::kCartesianPose);
  auto result = franka::CartesianPose(command.pose);
  result.motion_finished = command.motion_finished;
  return result;
}

}  // namespace mios::control
