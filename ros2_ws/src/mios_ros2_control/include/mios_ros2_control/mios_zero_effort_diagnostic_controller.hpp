#pragma once

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"

namespace mios_ros2_control {

// A deliberately minimal commissioning controller. It owns only the seven
// effort command interfaces and writes exactly zero external torque in every
// update. Keeping it independent of ROS subscriptions, clocks, model access,
// and MIOS command handling makes it suitable for isolating the real-time
// behavior of the controller-manager/FCI path.
class MiosZeroEffortDiagnosticController : public controller_interface::ControllerInterface {
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
  std::vector<std::string> joints_;
  double activation_velocity_threshold_{0.01};
  bool allow_zero_effort_activation_{false};
};

}  // namespace mios_ros2_control
