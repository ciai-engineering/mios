#include "mios_franka_ros2/cartesian_controllers.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

namespace mios_franka_ros2 {
namespace {

std::string normalize_prefix(std::string prefix) {
  if (!prefix.empty() && prefix.back() != '_') {
    prefix.push_back('_');
  }
  return prefix;
}

Eigen::Vector3d limited_vector(const Eigen::Vector3d& value, double maximum_norm) {
  const double norm = value.norm();
  return norm > maximum_norm && norm > 0.0 ? value * (maximum_norm / norm) : value;
}

bool frame_is_valid(const std::string& actual, const std::string& expected) {
  return actual.empty() || expected.empty() || actual == expected;
}

}  // namespace

controller_interface::CallbackReturn CartesianPoseController::on_init() {
  try {
    auto_declare<std::string>("arm_prefix", "");
    auto_declare<std::string>("base_frame", "fr3_link0");
    auto_declare<double>("max_linear_velocity", 0.1);
    auto_declare<double>("max_angular_velocity", 0.5);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPoseController::on_configure(
    const rclcpp_lifecycle::State&) {
  arm_prefix_ = normalize_prefix(get_node()->get_parameter("arm_prefix").as_string());
  base_frame_ = get_node()->get_parameter("base_frame").as_string();
  max_linear_velocity_ = get_node()->get_parameter("max_linear_velocity").as_double();
  max_angular_velocity_ = get_node()->get_parameter("max_angular_velocity").as_double();
  if (max_linear_velocity_ <= 0.0 || max_angular_velocity_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cartesian velocity limits must be positive");
    return controller_interface::CallbackReturn::ERROR;
  }

  interface_ = std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
      arm_prefix_, false);
  subscription_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/target_pose", rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        if (!frame_is_valid(message->header.frame_id, base_frame_)) {
          RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                                "Rejected pose in frame '%s'; expected '%s'",
                                message->header.frame_id.c_str(), base_frame_.c_str());
          return;
        }
        Eigen::Quaterniond orientation(message->pose.orientation.w, message->pose.orientation.x,
                                       message->pose.orientation.y, message->pose.orientation.z);
        if (!std::isfinite(orientation.squaredNorm()) || orientation.norm() < 1e-9) {
          RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                                "Rejected invalid pose quaternion");
          return;
        }
        const Eigen::Vector3d position(message->pose.position.x, message->pose.position.y,
                                       message->pose.position.z);
        if (!position.allFinite()) {
          return;
        }
        target_.writeFromNonRT(PoseTarget{orientation.normalized(), position, true});
      });
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CartesianPoseController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          interface_->get_command_interface_names()};
}

controller_interface::InterfaceConfiguration
CartesianPoseController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          interface_->get_state_interface_names()};
}

controller_interface::CallbackReturn CartesianPoseController::on_activate(
    const rclcpp_lifecycle::State&) {
  interface_->assign_loaned_command_interfaces(command_interfaces_);
  interface_->assign_loaned_state_interfaces(state_interfaces_);
  std::tie(commanded_orientation_, commanded_position_) =
      interface_->getCurrentOrientationAndTranslation();
  target_.writeFromNonRT(PoseTarget{commanded_orientation_, commanded_position_, true});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianPoseController::update(
    const rclcpp::Time&, const rclcpp::Duration& period) {
  const PoseTarget* target = target_.readFromRT();
  if (target == nullptr || !target->valid) {
    return controller_interface::return_type::OK;
  }

  const double dt = std::max(0.0, period.seconds());
  const Eigen::Vector3d delta = target->position - commanded_position_;
  commanded_position_ += limited_vector(delta, max_linear_velocity_ * dt);

  Eigen::Quaterniond destination = target->orientation;
  if (commanded_orientation_.dot(destination) < 0.0) {
    destination.coeffs() *= -1.0;
  }
  const double angular_distance = commanded_orientation_.angularDistance(destination);
  const double fraction = angular_distance > 1e-12
                              ? std::min(1.0, max_angular_velocity_ * dt / angular_distance)
                              : 1.0;
  commanded_orientation_ = commanded_orientation_.slerp(fraction, destination).normalized();

  return interface_->setCommand(commanded_orientation_, commanded_position_)
             ? controller_interface::return_type::OK
             : controller_interface::return_type::ERROR;
}

controller_interface::CallbackReturn CartesianPoseController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  interface_->release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityController::on_init() {
  try {
    auto_declare<std::string>("arm_prefix", "");
    auto_declare<std::string>("base_frame", "fr3_link0");
    auto_declare<double>("command_timeout", 0.1);
    auto_declare<double>("max_linear_velocity", 0.1);
    auto_declare<double>("max_angular_velocity", 0.5);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityController::on_configure(
    const rclcpp_lifecycle::State&) {
  arm_prefix_ = normalize_prefix(get_node()->get_parameter("arm_prefix").as_string());
  base_frame_ = get_node()->get_parameter("base_frame").as_string();
  command_timeout_ = get_node()->get_parameter("command_timeout").as_double();
  max_linear_velocity_ = get_node()->get_parameter("max_linear_velocity").as_double();
  max_angular_velocity_ = get_node()->get_parameter("max_angular_velocity").as_double();
  if (command_timeout_ <= 0.0 || max_linear_velocity_ <= 0.0 || max_angular_velocity_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Timeout and Cartesian velocity limits must be positive");
    return controller_interface::CallbackReturn::ERROR;
  }

  interface_ = std::make_unique<franka_semantic_components::FrankaCartesianVelocityInterface>(
      arm_prefix_, false);
  subscription_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
      "~/target_twist", rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr message) {
        if (!frame_is_valid(message->header.frame_id, base_frame_)) {
          RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                                "Rejected twist in frame '%s'; expected '%s'",
                                message->header.frame_id.c_str(), base_frame_.c_str());
          return;
        }
        Eigen::Vector3d linear(message->twist.linear.x, message->twist.linear.y,
                               message->twist.linear.z);
        Eigen::Vector3d angular(message->twist.angular.x, message->twist.angular.y,
                                message->twist.angular.z);
        if (!linear.allFinite() || !angular.allFinite()) {
          return;
        }
        linear = limited_vector(linear, max_linear_velocity_);
        angular = limited_vector(angular, max_angular_velocity_);
        target_.writeFromNonRT(
            VelocityTarget{linear, angular, get_node()->now().seconds(), true});
      });
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CartesianVelocityController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          interface_->get_command_interface_names()};
}

controller_interface::InterfaceConfiguration
CartesianVelocityController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn CartesianVelocityController::on_activate(
    const rclcpp_lifecycle::State&) {
  interface_->assign_loaned_command_interfaces(command_interfaces_);
  target_.writeFromNonRT(VelocityTarget{});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianVelocityController::update(
    const rclcpp::Time& time, const rclcpp::Duration&) {
  const VelocityTarget* target = target_.readFromRT();
  Eigen::Vector3d linear = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular = Eigen::Vector3d::Zero();
  if (target != nullptr && target->valid && time.seconds() - target->received_at <= command_timeout_) {
    linear = target->linear;
    angular = target->angular;
  }
  return interface_->setCommand(linear, angular) ? controller_interface::return_type::OK
                                                  : controller_interface::return_type::ERROR;
}

controller_interface::CallbackReturn CartesianVelocityController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  interface_->setCommand(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  interface_->release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace mios_franka_ros2

PLUGINLIB_EXPORT_CLASS(mios_franka_ros2::CartesianPoseController,
                       controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(mios_franka_ros2::CartesianVelocityController,
                       controller_interface::ControllerInterface)
