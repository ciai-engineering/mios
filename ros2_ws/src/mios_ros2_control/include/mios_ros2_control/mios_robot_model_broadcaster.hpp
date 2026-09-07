#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "franka_semantic_components/franka_robot_model.hpp"
#include "mios_msgs/msg/mios_robot_model.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace mios_ros2_control {

// Read-only ros2_control controller that exposes Franka model values to the
// non-real-time MIOS runtime. franka_hardware remains the sole FCI/libfranka
// owner. No command interface is claimed or written.
class MiosRobotModelBroadcaster : public controller_interface::ControllerInterface {
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
  std::string robot_type_{"fr3"};
  std::string arm_prefix_;
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_model_;
  rclcpp_lifecycle::LifecyclePublisher<mios_msgs::msg::MiosRobotModel>::SharedPtr
      lifecycle_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<mios_msgs::msg::MiosRobotModel>> publisher_;
  std::atomic<bool> active_{false};
};

}  // namespace mios_ros2_control
