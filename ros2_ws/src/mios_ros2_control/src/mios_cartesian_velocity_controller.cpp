#include "mios_ros2_control/mios_cartesian_velocity_controller.hpp"

#include <cmath>
#include <functional>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace mios_ros2_control {

controller_interface::CallbackReturn MiosCartesianVelocityController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<std::string>>(
        "cartesian_velocity_interfaces",
        {"vx/cartesian_velocity", "vy/cartesian_velocity", "vz/cartesian_velocity",
         "wx/cartesian_velocity", "wy/cartesian_velocity", "wz/cartesian_velocity"});
    auto_declare<std::vector<double>>("velocity_limits", std::vector<double>(6, 0.01));
    auto_declare<std::vector<double>>("acceleration_limits", std::vector<double>(6, 0.05));
    auto_declare<double>("command_timeout", command_timeout_);
    auto_declare<std::string>("robot_state_topic", robot_state_topic_);
    auto_declare<double>("robot_state_timeout", robot_state_timeout_);
    auto_declare<double>("activation_velocity_threshold", activation_velocity_threshold_);
    auto_declare<bool>("allow_cartesian_velocity_activation",
                       allow_cartesian_velocity_activation_);
    auto_declare<bool>("allow_runtime_cartesian_velocity_commands",
                       allow_runtime_cartesian_velocity_commands_);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to declare parameters: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MiosCartesianVelocityController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = cartesian_velocity_interfaces_;
  return config;
}

controller_interface::InterfaceConfiguration
MiosCartesianVelocityController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_) {
    config.names.push_back(joint + "/velocity");
  }
  return config;
}

bool MiosCartesianVelocityController::configure_parameters() {
  joints_ = get_node()->get_parameter("joints").as_string_array();
  cartesian_velocity_interfaces_ =
      get_node()->get_parameter("cartesian_velocity_interfaces").as_string_array();
  const auto velocity_limits = get_node()->get_parameter("velocity_limits").as_double_array();
  const auto acceleration_limits =
      get_node()->get_parameter("acceleration_limits").as_double_array();
  command_timeout_ = get_node()->get_parameter("command_timeout").as_double();
  robot_state_topic_ = get_node()->get_parameter("robot_state_topic").as_string();
  robot_state_timeout_ = get_node()->get_parameter("robot_state_timeout").as_double();
  activation_velocity_threshold_ =
      get_node()->get_parameter("activation_velocity_threshold").as_double();
  allow_cartesian_velocity_activation_ =
      get_node()->get_parameter("allow_cartesian_velocity_activation").as_bool();
  allow_runtime_cartesian_velocity_commands_ =
      get_node()->get_parameter("allow_runtime_cartesian_velocity_commands").as_bool();
  if (joints_.size() != JointVelocity{}.size() ||
      cartesian_velocity_interfaces_.size() != CartesianVelocity{}.size() ||
      velocity_limits.size() != velocity_limits_.size() ||
      acceleration_limits.size() != acceleration_limits_.size() || robot_state_topic_.empty() ||
      !std::isfinite(command_timeout_) || command_timeout_ <= 0.0 ||
      !std::isfinite(robot_state_timeout_) || robot_state_timeout_ <= 0.0 ||
      !std::isfinite(activation_velocity_threshold_) || activation_velocity_threshold_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected seven joints, six Cartesian interfaces/limits, and positive finite timing values.");
    return false;
  }
  for (std::size_t index = 0; index < velocity_limits_.size(); ++index) {
    if (!std::isfinite(velocity_limits[index]) || velocity_limits[index] <= 0.0 ||
        !std::isfinite(acceleration_limits[index]) || acceleration_limits[index] <= 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Cartesian velocity_limits and acceleration_limits must be positive finite values.");
      return false;
    }
    velocity_limits_[index] = velocity_limits[index];
    acceleration_limits_[index] = acceleration_limits[index];
  }
  control_core_.configure(velocity_limits_, acceleration_limits_,
                          static_cast<std::int64_t>(command_timeout_ * 1.0e9));
  return true;
}

controller_interface::CallbackReturn MiosCartesianVelocityController::on_configure(
    const rclcpp_lifecycle::State&) {
  if (!configure_parameters()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  runtime_subscription_ = get_node()->create_subscription<mios_msgs::msg::MiosActuatorCommand>(
      "~/mios_actuator_command", rclcpp::QoS(1).best_effort(),
      std::bind(&MiosCartesianVelocityController::receive_runtime_command, this,
                std::placeholders::_1));
  robot_state_subscription_ = get_node()->create_subscription<franka_msgs::msg::FrankaRobotState>(
      robot_state_topic_, rclcpp::QoS(1).best_effort(),
      std::bind(&MiosCartesianVelocityController::receive_robot_state, this,
                std::placeholders::_1));
  desired_velocity_.writeFromNonRT(std::make_shared<MiosCartesianVelocityRequest>());
  robot_safety_state_.writeFromNonRT(std::make_shared<RobotSafetyState>());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MiosCartesianVelocityController::on_activate(
    const rclcpp_lifecycle::State&) {
  control_core_.reset();
  if (!allow_cartesian_velocity_activation_) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Refusing Cartesian velocity activation: allow_cartesian_velocity_activation is false.");
    return controller_interface::CallbackReturn::ERROR;
  }
  const auto robot_safety = *robot_safety_state_.readFromRT();
  const auto now_nanoseconds = get_node()->now().nanoseconds();
  if (robot_safety == nullptr || !robot_safety->ready_for_control ||
      robot_safety->received_nanoseconds <= 0 || now_nanoseconds < robot_safety->received_nanoseconds ||
      now_nanoseconds - robot_safety->received_nanoseconds >
          static_cast<std::int64_t>(robot_state_timeout_ * 1.0e9)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Refusing Cartesian velocity activation without a fresh IDLE-or-MOVE robot state.");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t index = 0; index < robot_safety->velocity.size(); ++index) {
    const double live_velocity = state_interfaces_[index].get_value();
    if (!std::isfinite(robot_safety->velocity[index]) || !std::isfinite(live_velocity) ||
        std::abs(robot_safety->velocity[index]) > activation_velocity_threshold_ ||
        std::abs(live_velocity) > activation_velocity_threshold_) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Refusing Cartesian velocity activation: joint %zu is not stationary within %.4f rad/s.",
                   index, activation_velocity_threshold_);
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  for (auto& command_interface : command_interfaces_) {
    if (!command_interface.set_value(0.0)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to write a zero Cartesian velocity command on activation.");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MiosCartesianVelocityController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  control_core_.reset();
  for (auto& command_interface : command_interfaces_) {
    if (!command_interface.set_value(0.0)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to write a zero Cartesian velocity command on deactivation.");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

void MiosCartesianVelocityController::receive_runtime_command(
    const mios_msgs::msg::MiosActuatorCommand::SharedPtr message) {
  if (!allow_runtime_cartesian_velocity_commands_) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                         "Ignoring MIOS Cartesian velocity command: runtime commands are disabled.");
    return;
  }
  auto desired = std::make_shared<MiosCartesianVelocityRequest>();
  desired->received_nanoseconds = get_node()->now().nanoseconds();
  desired->user_stopped = message->user_stopped ||
                          message->mode == mios_msgs::msg::MiosActuatorCommand::MODE_STOP;
  if (desired->user_stopped) {
    desired_velocity_.writeFromNonRT(std::move(desired));
    return;
  }
  if (message->mode != mios_msgs::msg::MiosActuatorCommand::MODE_CARTESIAN_VELOCITY) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Ignoring unsupported MIOS Cartesian velocity actuator mode %u.",
                static_cast<unsigned>(message->mode));
    desired->user_stopped = true;
    desired_velocity_.writeFromNonRT(std::move(desired));
    return;
  }
  for (std::size_t index = 0; index < desired->velocity.size(); ++index) {
    if (!std::isfinite(message->target_cartesian_velocity[index])) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Ignoring non-finite MIOS Cartesian velocity command; clearing the active request.");
      desired->user_stopped = true;
      desired_velocity_.writeFromNonRT(std::move(desired));
      return;
    }
    desired->velocity[index] = message->target_cartesian_velocity[index];
  }
  desired_velocity_.writeFromNonRT(std::move(desired));
}

void MiosCartesianVelocityController::receive_robot_state(
    const franka_msgs::msg::FrankaRobotState::SharedPtr message) {
  if (message->measured_joint_state.velocity.size() != JointVelocity{}.size()) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                         "Ignoring incomplete Franka robot-state message for Cartesian velocity safety.");
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
                           "Ignoring non-finite Franka robot-state velocity for Cartesian safety.");
      return;
    }
    safety->velocity[index] = message->measured_joint_state.velocity[index];
  }
  robot_safety_state_.writeFromNonRT(std::move(safety));
}

controller_interface::return_type MiosCartesianVelocityController::update(
    const rclcpp::Time&, const rclcpp::Duration& period) {
  JointVelocity measured_joint_velocity{};
  for (std::size_t index = 0; index < measured_joint_velocity.size(); ++index) {
    measured_joint_velocity[index] = state_interfaces_[index].get_value();
    if (!std::isfinite(measured_joint_velocity[index])) {
      RCLCPP_ERROR(get_node()->get_logger(), "Non-finite robot velocity for joint %zu.", index);
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
  const CartesianVelocity command = control_core_.step(measured_joint_velocity, user_stopped,
                                                        request.get(), now_nanoseconds,
                                                        period.seconds());
  for (std::size_t index = 0; index < command.size(); ++index) {
    if (!command_interfaces_[index].set_value(command[index])) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to write desired Cartesian velocity component %zu.", index);
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace mios_ros2_control

PLUGINLIB_EXPORT_CLASS(mios_ros2_control::MiosCartesianVelocityController,
                       controller_interface::ControllerInterface)
