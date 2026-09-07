#pragma once

#include <array>
#include <cstdint>

namespace mios::control {

// Transport-neutral data exchanged between MIOS algorithms and a robot backend.
// ROS 2 adapters and legacy libfranka adapters convert at this boundary.
inline constexpr std::size_t kJointCount = 7;

enum class CommandMode : std::uint8_t {
  kTorque,
  kJointVelocity,
  kCartesianVelocity,
  kJointPosition,
  kCartesianPose,
};

enum class RobotMode : std::uint8_t {
  kOther,
  kIdle,
  kMove,
  kGuiding,
  kReflex,
  kUserStopped,
  kAutomaticErrorRecovery,
};

struct RobotState {
  std::array<double, kJointCount> position{};
  std::array<double, kJointCount> velocity{};
  std::array<double, kJointCount> motor_position{};
  std::array<double, kJointCount> motor_velocity{};
  std::array<double, kJointCount> effort{};
  std::array<double, kJointCount> external_effort{};
  std::array<double, 16> base_to_end_effector{};
  std::array<double, 6> external_wrench_origin{};
  std::array<double, 6> external_wrench_stiffness{};
  RobotMode robot_mode{RobotMode::kOther};
  bool user_stopped{false};
};

struct GripperState {
  double width{0.0};
  double max_width{0.0};
  double temperature{0.0};
  bool is_grasped{false};
};

struct RobotModel {
  std::array<double, 49> mass{};
  std::array<double, kJointCount> coriolis{};
  std::array<double, kJointCount> gravity{};
  std::array<double, 42> body_jacobian{};
  std::array<double, 42> zero_jacobian{};
};

// Robot configuration requested by MIOS Core outside the control cycle. A
// transport backend owns how these values reach the robot.
struct RobotParameters {
  double load_mass{0.0};
  std::array<double, 3> load_center_of_mass{};
  std::array<double, 9> load_inertia{};
  std::array<double, 16> tcp_frame{};
  std::array<double, 16> stiffness_frame{};
  std::array<double, kJointCount> joint_stiffness{};
  std::array<double, 6> cartesian_stiffness{};
  std::array<double, kJointCount> lower_torque_thresholds{};
  std::array<double, kJointCount> upper_torque_thresholds{};
  std::array<double, 6> lower_force_thresholds{};
  std::array<double, 6> upper_force_thresholds{};
};

struct ArmCommand {
  CommandMode mode{CommandMode::kTorque};
  std::array<double, kJointCount> joints{};
  std::array<double, 6> cartesian{};
  std::array<double, 16> pose{};
  bool motion_finished{false};
};

}  // namespace mios::control
