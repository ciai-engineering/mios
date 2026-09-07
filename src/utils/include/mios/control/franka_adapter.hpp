#pragma once

#include "franka/control_types.h"
#include "franka/gripper_state.h"
#include "franka/model.h"
#include "franka/robot_state.h"

#include "mios/control/control_types.hpp"

namespace mios::control {

RobotState from_franka_state(const franka::RobotState& state);
GripperState from_franka_state(const franka::GripperState& state);
RobotModel from_franka_model(const franka::Model& model, const franka::RobotState& state);
ArmCommand from_franka_command(const franka::Torques& command);
ArmCommand from_franka_command(const franka::JointVelocities& command);
ArmCommand from_franka_command(const franka::CartesianVelocities& command);
franka::Torques to_franka_torques(const ArmCommand& command);
franka::JointVelocities to_franka_joint_velocities(const ArmCommand& command);
franka::CartesianVelocities to_franka_cartesian_velocities(const ArmCommand& command);
franka::JointPositions to_franka_joint_positions(const ArmCommand& command);
franka::CartesianPose to_franka_cartesian_pose(const ArmCommand& command);

}  // namespace mios::control
