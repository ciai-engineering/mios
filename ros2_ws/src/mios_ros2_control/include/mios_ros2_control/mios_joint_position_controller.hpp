#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "franka/robot_state.h"
#include "franka_msgs/msg/franka_robot_state.hpp"
#include "mios_msgs/msg/mios_actuator_command.hpp"
#include "mios_ros2_control/mios_position_control_core.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

namespace mios_ros2_control {

// ROS-control position-mode boundary for MIOS joint position commands. It is
// deliberately separate from torque and velocity controllers so controller
// manager owns all mode switching.
class MiosJointPositionController : public controller_interface::ControllerInterface {
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
  using Position = MiosPositionControlCore::Position;

  struct RobotSafetyState {
    std::int64_t received_nanoseconds{0};
    bool ready_for_control{false};
    Position position{};
    Position velocity{};
    Position desired_position{};
    Position desired_velocity{};
    Position desired_acceleration{};
  };

  void receive_runtime_command(const mios_msgs::msg::MiosActuatorCommand::SharedPtr message);
  void receive_robot_state(const franka_msgs::msg::FrankaRobotState::SharedPtr message);
  bool configure_parameters();

  std::vector<std::string> joints_;
  std::string arm_id_{"fr3"};
  Position lower_limits_{};
  Position upper_limits_{};
  Position velocity_limits_{};
  double command_timeout_{0.1};
  // On a real Franka, get the state directly from the ros2_control hardware
  // instead of deserializing a complete 1 kHz RobotState message in the
  // controller-manager process.  The topic fallback supports GenericSystem
  // lifecycle tests, which do not export Franka's state box.
  std::string robot_state_source_{"topic"};
  std::string robot_state_topic_{"/franka_robot_state_broadcaster/robot_state"};
  double robot_state_timeout_{0.1};
  double activation_velocity_threshold_{0.01};
  double activation_position_tolerance_{0.001};
  double activation_acceleration_threshold_{0.1};
  bool allow_position_activation_{false};
  bool allow_runtime_joint_position_commands_{false};
  MiosPositionControlCore control_core_;
  realtime_tools::RealtimeThreadSafeBox<franka::RobotState>* robot_state_box_{nullptr};
  realtime_tools::RealtimeBuffer<std::shared_ptr<MiosPositionRequest>> desired_position_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<RobotSafetyState>> robot_safety_state_;
  rclcpp::Subscription<mios_msgs::msg::MiosActuatorCommand>::SharedPtr runtime_subscription_;
  rclcpp::Subscription<franka_msgs::msg::FrankaRobotState>::SharedPtr robot_state_subscription_;
};

}  // namespace mios_ros2_control
