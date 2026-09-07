#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "franka_msgs/msg/franka_robot_state.hpp"
#include "mios_msgs/msg/mios_actuator_command.hpp"
#include "mios_ros2_control/mios_velocity_control_core.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace mios_ros2_control {

// ROS-control implementation of MIOS' legacy JointVelocityControllerPipeline.
// It claims only velocity command interfaces, so controller_manager performs
// the required mode switch away from every effort controller.
class MiosJointVelocityController : public controller_interface::ControllerInterface {
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
  using Velocity = MiosVelocityControlCore::Velocity;

  struct RobotSafetyState {
    std::int64_t received_nanoseconds{0};
    bool ready_for_control{false};
    Velocity velocity{};
  };

  void receive_runtime_command(const mios_msgs::msg::MiosActuatorCommand::SharedPtr message);
  void receive_robot_state(const franka_msgs::msg::FrankaRobotState::SharedPtr message);
  bool configure_parameters();

  std::vector<std::string> joints_;
  Velocity velocity_limits_{};
  Velocity acceleration_limits_{};
  Velocity joint_lower_limits_{};
  Velocity joint_upper_limits_{};
  Velocity joint_limit_margins_{};
  double command_timeout_{0.1};
  std::string robot_state_topic_{"/franka_robot_state_broadcaster/robot_state"};
  double robot_state_timeout_{0.1};
  double activation_velocity_threshold_{0.01};
  bool allow_velocity_activation_{false};
  bool allow_runtime_joint_velocity_commands_{false};
  MiosVelocityControlCore control_core_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<MiosVelocityRequest>> desired_velocity_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<RobotSafetyState>> robot_safety_state_;
  rclcpp::Subscription<mios_msgs::msg::MiosActuatorCommand>::SharedPtr runtime_subscription_;
  rclcpp::Subscription<franka_msgs::msg::FrankaRobotState>::SharedPtr robot_state_subscription_;
};

}  // namespace mios_ros2_control
