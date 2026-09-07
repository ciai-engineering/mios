#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "franka_msgs/msg/franka_robot_state.hpp"
#include "mios_msgs/msg/mios_actuator_command.hpp"
#include "mios_ros2_control/mios_cartesian_pose_control_core.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace mios_ros2_control {

// ROS-control position-mode boundary for MIOS base-frame Cartesian pose
// commands. It claims Franka's 16 native Cartesian-pose interfaces.
class MiosCartesianPoseController : public controller_interface::ControllerInterface {
 public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                            const rclcpp::Duration& period) override;

 private:
  using Pose = MiosCartesianPoseControlCore::Pose;
  using Translation = MiosCartesianPoseControlCore::Translation;
  using JointVelocity = std::array<double, 7>;

  struct RobotSafetyState {
    std::int64_t received_nanoseconds{0};
    bool ready_for_control{false};
    Pose base_to_end_effector{};
    JointVelocity velocity{};
  };

  void receive_runtime_command(const mios_msgs::msg::MiosActuatorCommand::SharedPtr message);
  void receive_robot_state(const franka_msgs::msg::FrankaRobotState::SharedPtr message);
  bool configure_parameters();

  std::vector<std::string> joints_;
  std::vector<std::string> cartesian_pose_interfaces_;
  Translation lower_workspace_limits_{};
  Translation upper_workspace_limits_{};
  double linear_velocity_limit_{0.01};
  double angular_velocity_limit_{0.10};
  double command_timeout_{0.1};
  std::string robot_state_topic_{"/franka_robot_state_broadcaster/robot_state"};
  double robot_state_timeout_{0.1};
  double activation_velocity_threshold_{0.01};
  bool allow_cartesian_pose_activation_{false};
  bool allow_runtime_cartesian_pose_commands_{false};
  MiosCartesianPoseControlCore control_core_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<MiosCartesianPoseRequest>> desired_pose_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<RobotSafetyState>> robot_safety_state_;
  rclcpp::Subscription<mios_msgs::msg::MiosActuatorCommand>::SharedPtr runtime_subscription_;
  rclcpp::Subscription<franka_msgs::msg::FrankaRobotState>::SharedPtr robot_state_subscription_;
};

}  // namespace mios_ros2_control
