#include "mios_ros2_control/mios_cartesian_pose_controller.hpp"

#include <cmath>
#include <functional>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace mios_ros2_control {
namespace {

std::vector<std::string> default_cartesian_pose_interfaces() {
  std::vector<std::string> result;
  result.reserve(16);
  for (std::size_t index = 0; index < 16; ++index) {
    result.push_back(std::to_string(index) + "/cartesian_pose_command");
  }
  return result;
}

bool pose_from_robot_state(const franka_msgs::msg::FrankaRobotState& message,
                           MiosCartesianPoseControlCore::Pose* pose) {
  const auto& source = message.o_t_ee.pose;
  if (!std::isfinite(source.position.x) || !std::isfinite(source.position.y) ||
      !std::isfinite(source.position.z) || !std::isfinite(source.orientation.x) ||
      !std::isfinite(source.orientation.y) || !std::isfinite(source.orientation.z) ||
      !std::isfinite(source.orientation.w)) {
    return false;
  }
  const double norm = std::sqrt(source.orientation.x * source.orientation.x +
                                source.orientation.y * source.orientation.y +
                                source.orientation.z * source.orientation.z +
                                source.orientation.w * source.orientation.w);
  if (!std::isfinite(norm) || norm <= 1.0e-12) {
    return false;
  }
  const double x = source.orientation.x / norm;
  const double y = source.orientation.y / norm;
  const double z = source.orientation.z / norm;
  const double w = source.orientation.w / norm;
  *pose = {1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + z * w),
           2.0 * (x * z - y * w),       0.0,
           2.0 * (x * y - z * w),       1.0 - 2.0 * (x * x + z * z),
           2.0 * (y * z + x * w),       0.0,
           2.0 * (x * z + y * w),       2.0 * (y * z - x * w),
           1.0 - 2.0 * (x * x + y * y), 0.0,
           source.position.x,            source.position.y,
           source.position.z,            1.0};
  return MiosCartesianPoseControlCore::is_valid_pose(*pose);
}

}  // namespace

controller_interface::CallbackReturn MiosCartesianPoseController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<std::string>>("cartesian_pose_interfaces",
                                           default_cartesian_pose_interfaces());
    auto_declare<std::vector<double>>("workspace_lower_limits", std::vector<double>(3, -1.0));
    auto_declare<std::vector<double>>("workspace_upper_limits", std::vector<double>(3, 1.0));
    auto_declare<double>("linear_velocity_limit", linear_velocity_limit_);
    auto_declare<double>("angular_velocity_limit", angular_velocity_limit_);
    auto_declare<double>("command_timeout", command_timeout_);
    auto_declare<std::string>("robot_state_topic", robot_state_topic_);
    auto_declare<double>("robot_state_timeout", robot_state_timeout_);
    auto_declare<double>("activation_velocity_threshold", activation_velocity_threshold_);
    auto_declare<bool>("allow_cartesian_pose_activation", allow_cartesian_pose_activation_);
    auto_declare<bool>("allow_runtime_cartesian_pose_commands",
                       allow_runtime_cartesian_pose_commands_);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to declare parameters: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MiosCartesianPoseController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = cartesian_pose_interfaces_;
  return config;
}

controller_interface::InterfaceConfiguration
MiosCartesianPoseController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_) {
    config.names.push_back(joint + "/velocity");
  }
  return config;
}

bool MiosCartesianPoseController::configure_parameters() {
  joints_ = get_node()->get_parameter("joints").as_string_array();
  cartesian_pose_interfaces_ =
      get_node()->get_parameter("cartesian_pose_interfaces").as_string_array();
  const auto lower_workspace_limits =
      get_node()->get_parameter("workspace_lower_limits").as_double_array();
  const auto upper_workspace_limits =
      get_node()->get_parameter("workspace_upper_limits").as_double_array();
  linear_velocity_limit_ = get_node()->get_parameter("linear_velocity_limit").as_double();
  angular_velocity_limit_ = get_node()->get_parameter("angular_velocity_limit").as_double();
  command_timeout_ = get_node()->get_parameter("command_timeout").as_double();
  robot_state_topic_ = get_node()->get_parameter("robot_state_topic").as_string();
  robot_state_timeout_ = get_node()->get_parameter("robot_state_timeout").as_double();
  activation_velocity_threshold_ =
      get_node()->get_parameter("activation_velocity_threshold").as_double();
  allow_cartesian_pose_activation_ =
      get_node()->get_parameter("allow_cartesian_pose_activation").as_bool();
  allow_runtime_cartesian_pose_commands_ =
      get_node()->get_parameter("allow_runtime_cartesian_pose_commands").as_bool();
  if (joints_.size() != JointVelocity{}.size() || cartesian_pose_interfaces_.size() != Pose{}.size() ||
      lower_workspace_limits.size() != lower_workspace_limits_.size() ||
      upper_workspace_limits.size() != upper_workspace_limits_.size() || robot_state_topic_.empty() ||
      !std::isfinite(linear_velocity_limit_) || linear_velocity_limit_ <= 0.0 ||
      !std::isfinite(angular_velocity_limit_) || angular_velocity_limit_ <= 0.0 ||
      !std::isfinite(command_timeout_) || command_timeout_ <= 0.0 ||
      !std::isfinite(robot_state_timeout_) || robot_state_timeout_ <= 0.0 ||
      !std::isfinite(activation_velocity_threshold_) || activation_velocity_threshold_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected seven joints, 16 Cartesian pose interfaces, and positive finite timing values.");
    return false;
  }
  for (std::size_t index = 0; index < lower_workspace_limits_.size(); ++index) {
    if (!std::isfinite(lower_workspace_limits[index]) ||
        !std::isfinite(upper_workspace_limits[index]) ||
        lower_workspace_limits[index] >= upper_workspace_limits[index]) {
      RCLCPP_ERROR(get_node()->get_logger(), "Workspace limits must be finite and ordered.");
      return false;
    }
    lower_workspace_limits_[index] = lower_workspace_limits[index];
    upper_workspace_limits_[index] = upper_workspace_limits[index];
  }
  control_core_.configure(lower_workspace_limits_, upper_workspace_limits_, linear_velocity_limit_,
                          angular_velocity_limit_,
                          static_cast<std::int64_t>(command_timeout_ * 1.0e9));
  return true;
}

controller_interface::CallbackReturn MiosCartesianPoseController::on_configure(
    const rclcpp_lifecycle::State&) {
  if (!configure_parameters()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  runtime_subscription_ = get_node()->create_subscription<mios_msgs::msg::MiosActuatorCommand>(
      "~/mios_actuator_command", rclcpp::QoS(1).best_effort(),
      std::bind(&MiosCartesianPoseController::receive_runtime_command, this,
                std::placeholders::_1));
  robot_state_subscription_ = get_node()->create_subscription<franka_msgs::msg::FrankaRobotState>(
      robot_state_topic_, rclcpp::QoS(1).best_effort(),
      std::bind(&MiosCartesianPoseController::receive_robot_state, this,
                std::placeholders::_1));
  desired_pose_.writeFromNonRT(std::make_shared<MiosCartesianPoseRequest>());
  robot_safety_state_.writeFromNonRT(std::make_shared<RobotSafetyState>());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MiosCartesianPoseController::on_activate(
    const rclcpp_lifecycle::State&) {
  control_core_.reset();
  if (!allow_cartesian_pose_activation_) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Refusing Cartesian-pose activation: allow_cartesian_pose_activation is false.");
    return controller_interface::CallbackReturn::ERROR;
  }
  const auto robot_safety = *robot_safety_state_.readFromRT();
  const auto now_nanoseconds = get_node()->now().nanoseconds();
  if (robot_safety == nullptr || !robot_safety->ready_for_control ||
      !MiosCartesianPoseControlCore::is_valid_pose(robot_safety->base_to_end_effector) ||
      robot_safety->received_nanoseconds <= 0 || now_nanoseconds < robot_safety->received_nanoseconds ||
      now_nanoseconds - robot_safety->received_nanoseconds >
          static_cast<std::int64_t>(robot_state_timeout_ * 1.0e9)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Refusing Cartesian-pose activation without a fresh IDLE-or-MOVE robot state.");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t index = 0; index < robot_safety->velocity.size(); ++index) {
    const double live_velocity = state_interfaces_[index].get_value();
    if (!std::isfinite(robot_safety->velocity[index]) || !std::isfinite(live_velocity) ||
        std::abs(robot_safety->velocity[index]) > activation_velocity_threshold_ ||
        std::abs(live_velocity) > activation_velocity_threshold_) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Refusing Cartesian-pose activation: joint %zu is not stationary within %.4f rad/s.",
                   index, activation_velocity_threshold_);
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  control_core_.capture_hold_reference(robot_safety->base_to_end_effector);
  for (std::size_t index = 0; index < command_interfaces_.size(); ++index) {
    if (!command_interfaces_[index].set_value(robot_safety->base_to_end_effector[index])) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to write the current Cartesian pose command on activation.");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MiosCartesianPoseController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  control_core_.reset();
  const auto robot_safety = *robot_safety_state_.readFromRT();
  if (robot_safety == nullptr ||
      !MiosCartesianPoseControlCore::is_valid_pose(robot_safety->base_to_end_effector)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Refusing to write an unknown Cartesian pose on deactivation.");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t index = 0; index < command_interfaces_.size(); ++index) {
    if (!command_interfaces_[index].set_value(robot_safety->base_to_end_effector[index])) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to write the current Cartesian pose command on deactivation.");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

void MiosCartesianPoseController::receive_runtime_command(
    const mios_msgs::msg::MiosActuatorCommand::SharedPtr message) {
  if (!allow_runtime_cartesian_pose_commands_) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                         "Ignoring MIOS Cartesian-pose command: runtime commands are disabled.");
    return;
  }
  auto desired = std::make_shared<MiosCartesianPoseRequest>();
  desired->received_nanoseconds = get_node()->now().nanoseconds();
  desired->user_stopped = message->user_stopped ||
                          message->mode == mios_msgs::msg::MiosActuatorCommand::MODE_STOP;
  if (desired->user_stopped) {
    desired_pose_.writeFromNonRT(std::move(desired));
    return;
  }
  if (message->mode != mios_msgs::msg::MiosActuatorCommand::MODE_CARTESIAN_POSE) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Ignoring unsupported MIOS Cartesian-pose actuator mode %u.",
                static_cast<unsigned>(message->mode));
    desired->user_stopped = true;
    desired_pose_.writeFromNonRT(std::move(desired));
    return;
  }
  desired->base_to_end_effector = message->target_base_to_end_effector;
  if (!MiosCartesianPoseControlCore::is_valid_pose(desired->base_to_end_effector)) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Ignoring invalid MIOS Cartesian-pose command; clearing the active request.");
    desired->user_stopped = true;
  }
  desired_pose_.writeFromNonRT(std::move(desired));
}

void MiosCartesianPoseController::receive_robot_state(
    const franka_msgs::msg::FrankaRobotState::SharedPtr message) {
  if (message->measured_joint_state.velocity.size() != JointVelocity{}.size()) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                         "Ignoring incomplete Franka robot-state message for Cartesian-pose safety.");
    return;
  }
  auto safety = std::make_shared<RobotSafetyState>();
  if (!pose_from_robot_state(*message, &safety->base_to_end_effector)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                         "Ignoring invalid Franka end-effector pose for Cartesian-pose safety.");
    return;
  }
  safety->received_nanoseconds = get_node()->now().nanoseconds();
  safety->ready_for_control =
      message->robot_mode == franka_msgs::msg::FrankaRobotState::ROBOT_MODE_IDLE ||
      message->robot_mode == franka_msgs::msg::FrankaRobotState::ROBOT_MODE_MOVE;
  for (std::size_t index = 0; index < safety->velocity.size(); ++index) {
    if (!std::isfinite(message->measured_joint_state.velocity[index])) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                           "Ignoring non-finite Franka robot-state velocity for Cartesian-pose safety.");
      return;
    }
    safety->velocity[index] = message->measured_joint_state.velocity[index];
  }
  robot_safety_state_.writeFromNonRT(std::move(safety));
}

controller_interface::return_type MiosCartesianPoseController::update(
    const rclcpp::Time&, const rclcpp::Duration& period) {
  for (std::size_t index = 0; index < JointVelocity{}.size(); ++index) {
    const double measured_velocity = state_interfaces_[index].get_value();
    if (!std::isfinite(measured_velocity)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Non-finite robot velocity for joint %zu.", index);
      return controller_interface::return_type::ERROR;
    }
  }
  const auto robot_safety = *robot_safety_state_.readFromRT();
  const auto now_nanoseconds = get_node()->now().nanoseconds();
  if (robot_safety == nullptr || !robot_safety->ready_for_control ||
      !MiosCartesianPoseControlCore::is_valid_pose(robot_safety->base_to_end_effector) ||
      robot_safety->received_nanoseconds <= 0 || now_nanoseconds < robot_safety->received_nanoseconds ||
      now_nanoseconds - robot_safety->received_nanoseconds >
          static_cast<std::int64_t>(robot_state_timeout_ * 1.0e9)) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                          "Failing Cartesian-pose update because live robot pose is unavailable.");
    return controller_interface::return_type::ERROR;
  }
  const auto request = *desired_pose_.readFromRT();
  const Pose command = control_core_.step(robot_safety->base_to_end_effector, false, request.get(),
                                          now_nanoseconds, period.seconds());
  for (std::size_t index = 0; index < command.size(); ++index) {
    if (!command_interfaces_[index].set_value(command[index])) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to write desired Cartesian pose element %zu.", index);
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace mios_ros2_control

PLUGINLIB_EXPORT_CLASS(mios_ros2_control::MiosCartesianPoseController,
                       controller_interface::ControllerInterface)
