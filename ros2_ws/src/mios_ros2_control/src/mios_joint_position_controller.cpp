#include "mios_ros2_control/mios_joint_position_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <optional>
#include <type_traits>
#include <utility>

#include "franka/robot.h"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace mios_ros2_control {

namespace {

template <class To, class From>
std::enable_if_t<sizeof(To) == sizeof(From) && std::is_trivially_copyable<From>::value &&
                     std::is_trivially_copyable<To>::value,
                 To>
bit_cast(const From& source) noexcept {
  To destination;
  std::memcpy(&destination, &source, sizeof(To));
  return destination;
}

bool ready_for_position_control(const franka::RobotMode mode) {
  return mode == franka::RobotMode::kIdle || mode == franka::RobotMode::kMove;
}

}  // namespace

controller_interface::CallbackReturn MiosJointPositionController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("arm_id", arm_id_);
    auto_declare<std::vector<double>>("joint_lower_limits", std::vector<double>(7, -3.0));
    auto_declare<std::vector<double>>("joint_upper_limits", std::vector<double>(7, 3.0));
    auto_declare<std::vector<double>>("position_velocity_limits", std::vector<double>(7, 0.05));
    auto_declare<std::vector<double>>("position_acceleration_limits", std::vector<double>(7, 0.05));
    auto_declare<double>("command_timeout", command_timeout_);
    auto_declare<std::string>("robot_state_source", robot_state_source_);
    auto_declare<std::string>("robot_state_topic", robot_state_topic_);
    auto_declare<double>("robot_state_timeout", robot_state_timeout_);
    auto_declare<double>("activation_velocity_threshold", activation_velocity_threshold_);
    auto_declare<double>("activation_position_tolerance", activation_position_tolerance_);
    auto_declare<double>("activation_acceleration_threshold",
                         activation_acceleration_threshold_);
    auto_declare<bool>("allow_position_activation", allow_position_activation_);
    auto_declare<bool>("allow_runtime_joint_position_commands",
                       allow_runtime_joint_position_commands_);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to declare parameters: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MiosJointPositionController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_) {
    config.names.push_back(joint + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration
MiosJointPositionController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
  }
  if (robot_state_source_ == "hardware") {
    config.names.push_back(arm_id_ + "/robot_state");
  }
  return config;
}

bool MiosJointPositionController::configure_parameters() {
  joints_ = get_node()->get_parameter("joints").as_string_array();
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  const auto lower_limits = get_node()->get_parameter("joint_lower_limits").as_double_array();
  const auto upper_limits = get_node()->get_parameter("joint_upper_limits").as_double_array();
  const auto velocity_limits =
      get_node()->get_parameter("position_velocity_limits").as_double_array();
  const auto acceleration_limits =
      get_node()->get_parameter("position_acceleration_limits").as_double_array();
  command_timeout_ = get_node()->get_parameter("command_timeout").as_double();
  robot_state_source_ = get_node()->get_parameter("robot_state_source").as_string();
  robot_state_topic_ = get_node()->get_parameter("robot_state_topic").as_string();
  robot_state_timeout_ = get_node()->get_parameter("robot_state_timeout").as_double();
  activation_velocity_threshold_ =
      get_node()->get_parameter("activation_velocity_threshold").as_double();
  activation_position_tolerance_ =
      get_node()->get_parameter("activation_position_tolerance").as_double();
  activation_acceleration_threshold_ =
      get_node()->get_parameter("activation_acceleration_threshold").as_double();
  allow_position_activation_ = get_node()->get_parameter("allow_position_activation").as_bool();
  allow_runtime_joint_position_commands_ =
      get_node()->get_parameter("allow_runtime_joint_position_commands").as_bool();

  if (joints_.size() != lower_limits_.size() || lower_limits.size() != lower_limits_.size() ||
      upper_limits.size() != upper_limits_.size() ||
      velocity_limits.size() != velocity_limits_.size() ||
      acceleration_limits.size() != lower_limits_.size() || arm_id_.empty() ||
      !std::isfinite(command_timeout_) || command_timeout_ <= 0.0 ||
      !std::isfinite(robot_state_timeout_) || robot_state_timeout_ <= 0.0 ||
      !std::isfinite(activation_velocity_threshold_) || activation_velocity_threshold_ <= 0.0 ||
      !std::isfinite(activation_position_tolerance_) || activation_position_tolerance_ <= 0.0 ||
      !std::isfinite(activation_acceleration_threshold_) ||
          activation_acceleration_threshold_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected seven joints/limits and positive finite safety timeouts and thresholds.");
    return false;
  }
  if (robot_state_source_ != "hardware" && robot_state_source_ != "topic") {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "robot_state_source must be either 'hardware' or 'topic'.");
    return false;
  }
  if (robot_state_source_ == "topic" && robot_state_topic_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "robot_state_topic must be set when robot_state_source is 'topic'.");
    return false;
  }
  for (std::size_t index = 0; index < lower_limits_.size(); ++index) {
    if (!std::isfinite(lower_limits[index]) || !std::isfinite(upper_limits[index]) ||
        !std::isfinite(velocity_limits[index]) || !std::isfinite(acceleration_limits[index]) ||
        lower_limits[index] >= upper_limits[index] || velocity_limits[index] <= 0.0 ||
        acceleration_limits[index] <= 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Joint position bounds and position_velocity_limits must be finite and valid.");
      return false;
    }
    lower_limits_[index] = lower_limits[index];
    upper_limits_[index] = upper_limits[index];
    velocity_limits_[index] = velocity_limits[index];
  }
  Position acceleration_limit_array{};
  for (std::size_t index = 0; index < acceleration_limit_array.size(); ++index) {
    acceleration_limit_array[index] = acceleration_limits[index];
  }
  control_core_.configure(lower_limits_, upper_limits_, velocity_limits_, acceleration_limit_array,
                          static_cast<std::int64_t>(command_timeout_ * 1.0e9));
  return true;
}

controller_interface::CallbackReturn MiosJointPositionController::on_configure(
    const rclcpp_lifecycle::State&) {
  if (!configure_parameters()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  runtime_subscription_ = get_node()->create_subscription<mios_msgs::msg::MiosActuatorCommand>(
      "~/mios_actuator_command", rclcpp::QoS(1).best_effort(),
      std::bind(&MiosJointPositionController::receive_runtime_command, this,
                std::placeholders::_1));
  if (robot_state_source_ == "topic") {
    robot_state_subscription_ = get_node()->create_subscription<franka_msgs::msg::FrankaRobotState>(
        robot_state_topic_, rclcpp::QoS(1).best_effort(),
        std::bind(&MiosJointPositionController::receive_robot_state, this,
                  std::placeholders::_1));
  } else {
    robot_state_subscription_.reset();
  }
  desired_position_.writeFromNonRT(std::make_shared<MiosPositionRequest>());
  robot_safety_state_.writeFromNonRT(std::make_shared<RobotSafetyState>());
  robot_state_box_ = nullptr;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MiosJointPositionController::on_activate(
    const rclcpp_lifecycle::State&) {
  control_core_.reset();
  if (!allow_position_activation_) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Refusing joint-position activation: allow_position_activation is false.");
    return controller_interface::CallbackReturn::ERROR;
  }
  MiosPositionActivationState activation_state;
  const auto now_nanoseconds = get_node()->now().nanoseconds();
  const RobotSafetyState* robot_safety = nullptr;
  std::optional<franka::RobotState> hardware_state;
  if (robot_state_source_ == "hardware") {
    const std::string robot_state_interface = arm_id_ + "/robot_state";
    const auto interface = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(), [&](const auto& state_interface) {
          return state_interface.get_name() == robot_state_interface;
        });
    if (interface == state_interfaces_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Refusing joint-position activation: %s was not loaned by the hardware.",
                   robot_state_interface.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    const auto interface_value = interface->get_optional();
    if (!interface_value.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Refusing joint-position activation: unable to read %s.",
                   robot_state_interface.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    robot_state_box_ = bit_cast<realtime_tools::RealtimeThreadSafeBox<franka::RobotState>*>(
        interface_value.value());
    if (robot_state_box_ == nullptr) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Refusing joint-position activation: %s supplied a null state box.",
                   robot_state_interface.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    hardware_state = robot_state_box_->get();
    if (!ready_for_position_control(hardware_state->robot_mode)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Refusing joint-position activation: Franka is not in IDLE or MOVE mode.");
      robot_state_box_ = nullptr;
      return controller_interface::CallbackReturn::ERROR;
    }
  } else {
    const auto safety = *robot_safety_state_.readFromRT();
    if (safety == nullptr || !safety->ready_for_control || safety->received_nanoseconds <= 0 ||
        now_nanoseconds < safety->received_nanoseconds ||
        now_nanoseconds - safety->received_nanoseconds >
            static_cast<std::int64_t>(robot_state_timeout_ * 1.0e9)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Refusing joint-position activation without a fresh IDLE-or-MOVE robot state.");
      return controller_interface::CallbackReturn::ERROR;
    }
    robot_safety = safety.get();
  }
  for (std::size_t index = 0; index < activation_state.live_position.size(); ++index) {
    const double live_position = state_interfaces_[2 * index].get_value();
    const double live_velocity = state_interfaces_[2 * index + 1].get_value();
    activation_state.measured_position[index] =
        hardware_state.has_value() ? hardware_state->q[index] : robot_safety->position[index];
    activation_state.measured_velocity[index] =
        hardware_state.has_value() ? hardware_state->dq[index] : robot_safety->velocity[index];
    activation_state.live_position[index] = live_position;
    activation_state.live_velocity[index] = live_velocity;
    activation_state.desired_position[index] = hardware_state.has_value()
                                                   ? hardware_state->q_d[index]
                                                   : robot_safety->desired_position[index];
    activation_state.desired_velocity[index] = hardware_state.has_value()
                                                   ? hardware_state->dq_d[index]
                                                   : robot_safety->desired_velocity[index];
    activation_state.desired_acceleration[index] = hardware_state.has_value()
                                                       ? hardware_state->ddq_d[index]
                                                       : robot_safety->desired_acceleration[index];
  }
  if (!is_safe_position_activation(activation_state, activation_velocity_threshold_,
                                   activation_position_tolerance_,
                                   activation_acceleration_threshold_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Refusing joint-position activation until measured, live, and desired joint "
                 "references are aligned and stationary.");
    return controller_interface::CallbackReturn::ERROR;
  }
  // Preserve Franka's desired reference (q_d), not q. q can lag q_d by a few
  // microradians while the arm is stationary; using q for the first position
  // command creates an artificial 1 kHz velocity/acceleration step that
  // Franka correctly rejects as a motion-generator discontinuity.
  const Position initial_position = activation_state.desired_position;
  control_core_.capture_hold_reference(initial_position);
  for (std::size_t index = 0; index < initial_position.size(); ++index) {
    if (!command_interfaces_[index].set_value(initial_position[index])) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to write the desired position command on activation.");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MiosJointPositionController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  const Position last_command = control_core_.hold_reference();
  for (std::size_t index = 0; index < command_interfaces_.size(); ++index) {
    if (!std::isfinite(last_command[index]) ||
        !command_interfaces_[index].set_value(last_command[index])) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to preserve the last desired position command on deactivation.");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  control_core_.reset();
  robot_state_box_ = nullptr;
  return controller_interface::CallbackReturn::SUCCESS;
}

void MiosJointPositionController::receive_runtime_command(
    const mios_msgs::msg::MiosActuatorCommand::SharedPtr message) {
  if (!allow_runtime_joint_position_commands_) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                         "Ignoring MIOS joint-position command: runtime commands are disabled.");
    return;
  }
  auto desired = std::make_shared<MiosPositionRequest>();
  desired->received_nanoseconds = get_node()->now().nanoseconds();
  desired->user_stopped = message->user_stopped ||
                          message->mode == mios_msgs::msg::MiosActuatorCommand::MODE_STOP;
  if (desired->user_stopped) {
    desired_position_.writeFromNonRT(std::move(desired));
    return;
  }
  if (message->mode != mios_msgs::msg::MiosActuatorCommand::MODE_JOINT_POSITION) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Ignoring unsupported MIOS joint-position actuator mode %u.",
                static_cast<unsigned>(message->mode));
    desired->user_stopped = true;
    desired_position_.writeFromNonRT(std::move(desired));
    return;
  }
  for (std::size_t index = 0; index < desired->position.size(); ++index) {
    if (!std::isfinite(message->joint_position[index])) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Ignoring non-finite MIOS joint-position command; clearing the active request.");
      desired->user_stopped = true;
      desired_position_.writeFromNonRT(std::move(desired));
      return;
    }
    desired->position[index] = message->joint_position[index];
  }
  desired_position_.writeFromNonRT(std::move(desired));
}

void MiosJointPositionController::receive_robot_state(
    const franka_msgs::msg::FrankaRobotState::SharedPtr message) {
  if (message->measured_joint_state.position.size() != lower_limits_.size() ||
      message->measured_joint_state.velocity.size() != velocity_limits_.size() ||
      message->desired_joint_state.position.size() != lower_limits_.size() ||
      message->desired_joint_state.velocity.size() != velocity_limits_.size()) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                         "Ignoring incomplete Franka robot-state message for joint-position safety.");
    return;
  }
  auto safety = std::make_shared<RobotSafetyState>();
  safety->received_nanoseconds = get_node()->now().nanoseconds();
  safety->ready_for_control =
      message->robot_mode == franka_msgs::msg::FrankaRobotState::ROBOT_MODE_IDLE ||
      message->robot_mode == franka_msgs::msg::FrankaRobotState::ROBOT_MODE_MOVE;
  for (std::size_t index = 0; index < safety->position.size(); ++index) {
    if (!std::isfinite(message->measured_joint_state.position[index]) ||
        !std::isfinite(message->measured_joint_state.velocity[index]) ||
        !std::isfinite(message->desired_joint_state.position[index]) ||
        !std::isfinite(message->desired_joint_state.velocity[index]) ||
        !std::isfinite(message->ddq_d[index])) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                           "Ignoring non-finite Franka robot state for joint-position safety.");
      return;
    }
    safety->position[index] = message->measured_joint_state.position[index];
    safety->velocity[index] = message->measured_joint_state.velocity[index];
    safety->desired_position[index] = message->desired_joint_state.position[index];
    safety->desired_velocity[index] = message->desired_joint_state.velocity[index];
    safety->desired_acceleration[index] = message->ddq_d[index];
  }
  robot_safety_state_.writeFromNonRT(std::move(safety));
}

controller_interface::return_type MiosJointPositionController::update(
    const rclcpp::Time&, const rclcpp::Duration& period) {
  Position measured_position{};
  for (std::size_t index = 0; index < measured_position.size(); ++index) {
    measured_position[index] = state_interfaces_[2 * index].get_value();
    const double measured_velocity = state_interfaces_[2 * index + 1].get_value();
    if (!std::isfinite(measured_position[index]) || !std::isfinite(measured_velocity)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Non-finite robot state for joint %zu.", index);
      return controller_interface::return_type::ERROR;
    }
  }
  const auto request = *desired_position_.readFromRT();
  const auto now_nanoseconds = get_node()->now().nanoseconds();
  bool user_stopped = false;
  if (robot_state_source_ == "hardware") {
    const auto robot_state = robot_state_box_ == nullptr
                                 ? std::optional<franka::RobotState>{}
                                 : robot_state_box_->try_get();
    user_stopped = !robot_state.has_value() || !ready_for_position_control(robot_state->robot_mode);
  } else {
    const auto robot_safety = *robot_safety_state_.readFromRT();
    user_stopped = robot_safety == nullptr || !robot_safety->ready_for_control ||
                   robot_safety->received_nanoseconds <= 0 ||
                   now_nanoseconds < robot_safety->received_nanoseconds ||
                   now_nanoseconds - robot_safety->received_nanoseconds >
                       static_cast<std::int64_t>(robot_state_timeout_ * 1.0e9);
  }
  const Position command = control_core_.step(measured_position, user_stopped, request.get(),
                                              now_nanoseconds, period.seconds());
  for (std::size_t index = 0; index < command.size(); ++index) {
    if (!command_interfaces_[index].set_value(command[index])) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to write desired position for joint %zu.", index);
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace mios_ros2_control

PLUGINLIB_EXPORT_CLASS(mios_ros2_control::MiosJointPositionController,
                       controller_interface::ControllerInterface)
