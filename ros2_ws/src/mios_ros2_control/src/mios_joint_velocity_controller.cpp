#include "mios_ros2_control/mios_joint_velocity_controller.hpp"

#include <cmath>
#include <functional>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace mios_ros2_control {

controller_interface::CallbackReturn MiosJointVelocityController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<double>>("velocity_limits", std::vector<double>(7, 0.05));
    auto_declare<std::vector<double>>("acceleration_limits", std::vector<double>(7, 0.10));
    auto_declare<std::vector<double>>("joint_lower_limits", std::vector<double>(7, -3.0));
    auto_declare<std::vector<double>>("joint_upper_limits", std::vector<double>(7, 3.0));
    auto_declare<std::vector<double>>("joint_limit_margins", std::vector<double>(7, 0.10));
    auto_declare<double>("command_timeout", command_timeout_);
    auto_declare<std::string>("robot_state_topic", robot_state_topic_);
    auto_declare<double>("robot_state_timeout", robot_state_timeout_);
    auto_declare<double>("activation_velocity_threshold", activation_velocity_threshold_);
    auto_declare<bool>("allow_velocity_activation", allow_velocity_activation_);
    auto_declare<bool>("allow_runtime_joint_velocity_commands",
                       allow_runtime_joint_velocity_commands_);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to declare parameters: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MiosJointVelocityController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_) {
    config.names.push_back(joint + "/velocity");
  }
  return config;
}

controller_interface::InterfaceConfiguration
MiosJointVelocityController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
  }
  return config;
}

bool MiosJointVelocityController::configure_parameters() {
  joints_ = get_node()->get_parameter("joints").as_string_array();
  const auto velocity_limits = get_node()->get_parameter("velocity_limits").as_double_array();
  const auto acceleration_limits =
      get_node()->get_parameter("acceleration_limits").as_double_array();
  const auto lower_limits = get_node()->get_parameter("joint_lower_limits").as_double_array();
  const auto upper_limits = get_node()->get_parameter("joint_upper_limits").as_double_array();
  const auto joint_limit_margins =
      get_node()->get_parameter("joint_limit_margins").as_double_array();
  command_timeout_ = get_node()->get_parameter("command_timeout").as_double();
  robot_state_topic_ = get_node()->get_parameter("robot_state_topic").as_string();
  robot_state_timeout_ = get_node()->get_parameter("robot_state_timeout").as_double();
  activation_velocity_threshold_ =
      get_node()->get_parameter("activation_velocity_threshold").as_double();
  allow_velocity_activation_ = get_node()->get_parameter("allow_velocity_activation").as_bool();
  allow_runtime_joint_velocity_commands_ =
      get_node()->get_parameter("allow_runtime_joint_velocity_commands").as_bool();
  if (joints_.size() != velocity_limits_.size() ||
      velocity_limits.size() != velocity_limits_.size() ||
      acceleration_limits.size() != acceleration_limits_.size() ||
      lower_limits.size() != joint_lower_limits_.size() ||
      upper_limits.size() != joint_upper_limits_.size() ||
      joint_limit_margins.size() != joint_limit_margins_.size() || robot_state_topic_.empty() ||
      !std::isfinite(command_timeout_) || command_timeout_ <= 0.0 ||
      !std::isfinite(robot_state_timeout_) || robot_state_timeout_ <= 0.0 ||
      !std::isfinite(activation_velocity_threshold_) || activation_velocity_threshold_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected seven joints/limits and positive finite timeouts and activation threshold.");
    return false;
  }
  for (std::size_t index = 0; index < velocity_limits_.size(); ++index) {
    if (!std::isfinite(velocity_limits[index]) || velocity_limits[index] <= 0.0 ||
        !std::isfinite(acceleration_limits[index]) || acceleration_limits[index] <= 0.0 ||
        !std::isfinite(lower_limits[index]) || !std::isfinite(upper_limits[index]) ||
        !std::isfinite(joint_limit_margins[index]) || joint_limit_margins[index] <= 0.0 ||
        lower_limits[index] + joint_limit_margins[index] >=
            upper_limits[index] - joint_limit_margins[index]) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Velocity limits, joint bounds, and joint-limit margins must be finite and valid.");
      return false;
    }
    velocity_limits_[index] = velocity_limits[index];
    acceleration_limits_[index] = acceleration_limits[index];
    joint_lower_limits_[index] = lower_limits[index];
    joint_upper_limits_[index] = upper_limits[index];
    joint_limit_margins_[index] = joint_limit_margins[index];
  }
  control_core_.configure(velocity_limits_, acceleration_limits_, joint_lower_limits_,
                          joint_upper_limits_, joint_limit_margins_,
                          static_cast<std::int64_t>(command_timeout_ * 1.0e9));
  return true;
}

controller_interface::CallbackReturn MiosJointVelocityController::on_configure(
    const rclcpp_lifecycle::State&) {
  if (!configure_parameters()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  runtime_subscription_ = get_node()->create_subscription<mios_msgs::msg::MiosActuatorCommand>(
      "~/mios_actuator_command", rclcpp::QoS(1).best_effort(),
      std::bind(&MiosJointVelocityController::receive_runtime_command, this,
                std::placeholders::_1));
  robot_state_subscription_ = get_node()->create_subscription<franka_msgs::msg::FrankaRobotState>(
      robot_state_topic_, rclcpp::QoS(1).best_effort(),
      std::bind(&MiosJointVelocityController::receive_robot_state, this,
                std::placeholders::_1));
  desired_velocity_.writeFromNonRT(std::make_shared<MiosVelocityRequest>());
  robot_safety_state_.writeFromNonRT(std::make_shared<RobotSafetyState>());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MiosJointVelocityController::on_activate(
    const rclcpp_lifecycle::State&) {
  control_core_.reset();
  if (!allow_velocity_activation_) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Refusing velocity activation: allow_velocity_activation is false.");
    return controller_interface::CallbackReturn::ERROR;
  }
  const auto robot_safety = *robot_safety_state_.readFromRT();
  const auto now_nanoseconds = get_node()->now().nanoseconds();
  if (robot_safety == nullptr || !robot_safety->ready_for_control ||
      robot_safety->received_nanoseconds <= 0 || now_nanoseconds < robot_safety->received_nanoseconds ||
      now_nanoseconds - robot_safety->received_nanoseconds >
          static_cast<std::int64_t>(robot_state_timeout_ * 1.0e9)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Refusing velocity activation without a fresh IDLE-or-MOVE robot state.");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t index = 0; index < velocity_limits_.size(); ++index) {
    const double live_position = state_interfaces_[2 * index].get_value();
    const double live_velocity = state_interfaces_[2 * index + 1].get_value();
    if (!std::isfinite(live_position) || !std::isfinite(robot_safety->velocity[index]) ||
        !std::isfinite(live_velocity) ||
        std::abs(robot_safety->velocity[index]) > activation_velocity_threshold_ ||
        std::abs(live_velocity) > activation_velocity_threshold_) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Refusing velocity activation: joint %zu is not stationary within %.4f rad/s.",
                   index, activation_velocity_threshold_);
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  for (auto& command_interface : command_interfaces_) {
    if (!command_interface.set_value(0.0)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to write a zero velocity command on activation.");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MiosJointVelocityController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  control_core_.reset();
  for (auto& command_interface : command_interfaces_) {
    if (!command_interface.set_value(0.0)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to write a zero velocity command on deactivation.");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

void MiosJointVelocityController::receive_runtime_command(
    const mios_msgs::msg::MiosActuatorCommand::SharedPtr message) {
  if (!allow_runtime_joint_velocity_commands_) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                         "Ignoring MIOS joint-velocity command: runtime commands are disabled.");
    return;
  }
  auto desired = std::make_shared<MiosVelocityRequest>();
  desired->received_nanoseconds = get_node()->now().nanoseconds();
  desired->user_stopped = message->user_stopped ||
                          message->mode == mios_msgs::msg::MiosActuatorCommand::MODE_STOP;
  if (desired->user_stopped) {
    desired_velocity_.writeFromNonRT(std::move(desired));
    return;
  }
  if (message->mode != mios_msgs::msg::MiosActuatorCommand::MODE_JOINT_VELOCITY) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Ignoring unsupported MIOS velocity-controller actuator mode %u.",
                static_cast<unsigned>(message->mode));
    desired->user_stopped = true;
    desired_velocity_.writeFromNonRT(std::move(desired));
    return;
  }
  for (std::size_t index = 0; index < desired->velocity.size(); ++index) {
    if (!std::isfinite(message->joint_velocity[index])) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Ignoring non-finite MIOS joint-velocity command; clearing the active request.");
      desired->user_stopped = true;
      desired_velocity_.writeFromNonRT(std::move(desired));
      return;
    }
    desired->velocity[index] = message->joint_velocity[index];
  }
  desired_velocity_.writeFromNonRT(std::move(desired));
}

void MiosJointVelocityController::receive_robot_state(
    const franka_msgs::msg::FrankaRobotState::SharedPtr message) {
  if (message->measured_joint_state.velocity.size() != velocity_limits_.size()) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                         "Ignoring incomplete Franka robot-state message for velocity activation safety.");
    return;
  }
  auto safety = std::make_shared<RobotSafetyState>();
  safety->received_nanoseconds = get_node()->now().nanoseconds();
  safety->ready_for_control =
      message->robot_mode == franka_msgs::msg::FrankaRobotState::ROBOT_MODE_IDLE ||
      message->robot_mode == franka_msgs::msg::FrankaRobotState::ROBOT_MODE_MOVE;
  for (std::size_t index = 0; index < safety->velocity.size(); ++index) {
    if (!std::isfinite(message->measured_joint_state.velocity[index])) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                           "Ignoring non-finite Franka robot-state velocity for activation safety.");
      return;
    }
    safety->velocity[index] = message->measured_joint_state.velocity[index];
  }
  robot_safety_state_.writeFromNonRT(std::move(safety));
}

controller_interface::return_type MiosJointVelocityController::update(
    const rclcpp::Time&, const rclcpp::Duration& period) {
  Velocity measured_position{};
  Velocity measured_velocity{};
  for (std::size_t index = 0; index < measured_velocity.size(); ++index) {
    measured_position[index] = state_interfaces_[2 * index].get_value();
    measured_velocity[index] = state_interfaces_[2 * index + 1].get_value();
    if (!std::isfinite(measured_position[index]) || !std::isfinite(measured_velocity[index])) {
      RCLCPP_ERROR(get_node()->get_logger(), "Non-finite robot state for joint %zu.", index);
      return controller_interface::return_type::ERROR;
    }
  }
  const auto request = *desired_velocity_.readFromRT();
  const auto robot_safety = *robot_safety_state_.readFromRT();
  const auto now_nanoseconds = get_node()->now().nanoseconds();
  const bool user_stopped = robot_safety == nullptr || !robot_safety->ready_for_control ||
                            robot_safety->received_nanoseconds <= 0 ||
                            now_nanoseconds < robot_safety->received_nanoseconds ||
                            now_nanoseconds - robot_safety->received_nanoseconds >
                                static_cast<std::int64_t>(robot_state_timeout_ * 1.0e9);
  const Velocity command = control_core_.step(measured_position, measured_velocity,
                                               user_stopped, request.get(), now_nanoseconds,
                                               period.seconds());
  for (std::size_t index = 0; index < command.size(); ++index) {
    if (!command_interfaces_[index].set_value(command[index])) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to write desired velocity for joint %zu.", index);
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace mios_ros2_control

PLUGINLIB_EXPORT_CLASS(mios_ros2_control::MiosJointVelocityController,
                       controller_interface::ControllerInterface)
