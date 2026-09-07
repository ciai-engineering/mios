#include "mios_ros2_control/mios_robot_model_broadcaster.hpp"

#include <exception>

#include "franka/robot.h"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace mios_ros2_control {

controller_interface::CallbackReturn MiosRobotModelBroadcaster::on_init() {
  try {
    auto_declare<std::string>("robot_type", robot_type_);
    auto_declare<std::string>("arm_prefix", arm_prefix_);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to declare model-broadcaster parameters: %s",
                 error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MiosRobotModelBroadcaster::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
MiosRobotModelBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  if (!franka_model_) {
    return config;
  }
  config.names = franka_model_->get_state_interface_names();
  return config;
}

controller_interface::CallbackReturn MiosRobotModelBroadcaster::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_type_ = get_node()->get_parameter("robot_type").as_string();
  arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
  if (robot_type_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_type must be set.");
    return controller_interface::CallbackReturn::ERROR;
  }

  const std::string prefix = arm_prefix_.empty() ? "" : arm_prefix_ + "_";
  franka_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      prefix + robot_type_ + "/robot_model", prefix + robot_type_ + "/robot_state");
  lifecycle_publisher_ = get_node()->create_publisher<mios_msgs::msg::MiosRobotModel>(
      "~/robot_model", rclcpp::QoS(1).best_effort());
  publisher_ = std::make_unique<realtime_tools::RealtimePublisher<mios_msgs::msg::MiosRobotModel>>(
      lifecycle_publisher_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MiosRobotModelBroadcaster::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!franka_model_ || !lifecycle_publisher_ || !publisher_) {
    return controller_interface::CallbackReturn::ERROR;
  }
  franka_model_->assign_loaned_state_interfaces(state_interfaces_);
  lifecycle_publisher_->on_activate();
  active_.store(true, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MiosRobotModelBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  active_.store(false, std::memory_order_release);
  if (lifecycle_publisher_) {
    lifecycle_publisher_->on_deactivate();
  }
  if (franka_model_) {
    franka_model_->release_interfaces();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MiosRobotModelBroadcaster::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  if (!active_.load(std::memory_order_acquire) || !franka_model_) {
    return controller_interface::return_type::OK;
  }

  // RealtimePublisher::trylock() never blocks. If the non-real-time publish
  // thread is busy, this model frame is intentionally dropped rather than
  // delaying the ros2_control update cycle.
  if (!publisher_->trylock()) {
    return controller_interface::return_type::OK;
  }
  try {
    auto& model = publisher_->msg_;
    model.stamp = time;
    model.mass = franka_model_->getMassMatrix();
    model.coriolis = franka_model_->getCoriolisForceVector();
    model.gravity = franka_model_->getGravityForceVector();
    model.body_jacobian = franka_model_->getBodyJacobian(franka::Frame::kEndEffector);
    model.zero_jacobian = franka_model_->getZeroJacobian(franka::Frame::kEndEffector);
  } catch (const std::exception& error) {
    publisher_->unlock();
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to read Franka model interfaces: %s",
                 error.what());
    return controller_interface::return_type::ERROR;
  }
  publisher_->unlockAndPublish();
  return controller_interface::return_type::OK;
}

}  // namespace mios_ros2_control

PLUGINLIB_EXPORT_CLASS(mios_ros2_control::MiosRobotModelBroadcaster,
                       controller_interface::ControllerInterface)
