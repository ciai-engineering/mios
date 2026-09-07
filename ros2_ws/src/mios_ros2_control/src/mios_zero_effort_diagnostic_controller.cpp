#include "mios_ros2_control/mios_zero_effort_diagnostic_controller.hpp"

#include <cmath>
#include <exception>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace mios_ros2_control {

controller_interface::CallbackReturn MiosZeroEffortDiagnosticController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<bool>("allow_zero_effort_activation", false);
    auto_declare<double>("activation_velocity_threshold", activation_velocity_threshold_);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to declare parameters: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MiosZeroEffortDiagnosticController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_) {
    config.names.push_back(joint + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
MiosZeroEffortDiagnosticController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
  }
  return config;
}

controller_interface::CallbackReturn MiosZeroEffortDiagnosticController::on_configure(
    const rclcpp_lifecycle::State&) {
  try {
    joints_ = get_node()->get_parameter("joints").as_string_array();
    allow_zero_effort_activation_ =
        get_node()->get_parameter("allow_zero_effort_activation").as_bool();
    activation_velocity_threshold_ =
        get_node()->get_parameter("activation_velocity_threshold").as_double();
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to read parameters: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (joints_.size() != 7 || !std::isfinite(activation_velocity_threshold_) ||
      activation_velocity_threshold_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected exactly seven joints and a positive finite activation velocity threshold.");
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MiosZeroEffortDiagnosticController::on_activate(
    const rclcpp_lifecycle::State&) {
  bool allowed = false;
  try {
    allowed = get_node()->get_parameter("allow_zero_effort_activation").as_bool();
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to read activation gate: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!allowed || command_interfaces_.size() != joints_.size() ||
      state_interfaces_.size() != joints_.size() * 2) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Refusing zero-effort diagnostic activation: gate or interfaces are invalid.");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    const auto& position = state_interfaces_[index * 2];
    const auto& velocity = state_interfaces_[index * 2 + 1];
    if (command_interfaces_[index].get_name() != joints_[index] + "/effort" ||
        position.get_name() != joints_[index] + "/position" ||
        velocity.get_name() != joints_[index] + "/velocity" ||
        !std::isfinite(position.get_value()) || !std::isfinite(velocity.get_value()) ||
        std::abs(velocity.get_value()) > activation_velocity_threshold_ ||
        !command_interfaces_[index].set_value(0.0)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Refusing zero-effort diagnostic activation: joint %zu is invalid or moving.", index);
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MiosZeroEffortDiagnosticController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  for (auto& command_interface : command_interfaces_) {
    if (!command_interface.set_value(0.0)) {
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MiosZeroEffortDiagnosticController::update(
    const rclcpp::Time&, const rclcpp::Duration&) {
  for (auto& command_interface : command_interfaces_) {
    if (!command_interface.set_value(0.0)) {
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace mios_ros2_control

PLUGINLIB_EXPORT_CLASS(mios_ros2_control::MiosZeroEffortDiagnosticController,
                       controller_interface::ControllerInterface)
