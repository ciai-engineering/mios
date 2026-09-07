#include "mios_ros2_control/mios_effort_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <functional>
#include <limits>
#include <optional>
#include <type_traits>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "franka/robot.h"

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

bool ready_for_effort_control(franka::RobotMode mode) {
  return mode == franka::RobotMode::kIdle || mode == franka::RobotMode::kMove;
}

}  // namespace

controller_interface::CallbackReturn MiosEffortController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("arm_id", arm_id_);
    auto_declare<std::vector<double>>("effort_limits", std::vector<double>(7, 5.0));
    auto_declare<double>("effort_rate_limit", effort_rate_limit_);
    auto_declare<double>("command_timeout", command_timeout_);
    auto_declare<std::string>("robot_state_source", robot_state_source_);
    auto_declare<std::string>("robot_state_topic", robot_state_topic_);
    auto_declare<double>("robot_state_timeout", robot_state_timeout_);
    auto_declare<bool>("robot_state_safety_enabled", robot_state_safety_enabled_);
    auto_declare<double>("activation_velocity_threshold", activation_velocity_threshold_);
    auto_declare<bool>("allow_effort_activation", allow_effort_activation_);
    // In this mode MIOS writes exactly zero external torque. Franka's master
    // controller supplies gravity compensation, matching Franka's official
    // gravity-compensation example. It is deliberately gated separately from
    // an active MIOS position hold or any runtime command source.
    auto_declare<bool>("allow_zero_effort_activation", false);
    auto_declare<bool>("allow_external_effort_commands", allow_external_effort_commands_);
    auto_declare<bool>("allow_runtime_effort_commands", allow_runtime_effort_commands_);
    auto_declare<bool>("allow_runtime_actuator_commands", allow_runtime_actuator_commands_);
    auto_declare<bool>("allow_runtime_cartesian_actuator_commands",
                       allow_runtime_cartesian_actuator_commands_);
    auto_declare<bool>("allow_runtime_cartesian_force_commands",
                       allow_runtime_cartesian_force_commands_);
    auto_declare<std::vector<double>>("joint_lower_limits", std::vector<double>(7, -3.0));
    auto_declare<std::vector<double>>("joint_upper_limits", std::vector<double>(7, 3.0));
    auto_declare<std::vector<double>>("joint_wall_stiffness", std::vector<double>(7, 0.0));
    auto_declare<std::vector<double>>("joint_wall_damping", std::vector<double>(7, 0.0));
    auto_declare<std::vector<double>>("joint_wall_max_torque", std::vector<double>(7, 0.0));
    auto_declare<bool>("joint_safety_enabled", joint_safety_enabled_);
    auto_declare<std::vector<double>>("hold_stiffness", std::vector<double>(7, 0.0));
    auto_declare<std::vector<double>>("hold_damping", std::vector<double>(7, 0.0));
    auto_declare<std::vector<double>>("hold_max_torque", std::vector<double>(7, 0.0));
    auto_declare<bool>("position_hold_enabled", position_hold_enabled_);
    auto_declare<double>("effort_diagnostic_publish_rate", effort_diagnostic_publish_rate_);
    auto_declare<std::vector<double>>("cartesian_velocity_threshold", std::vector<double>(6, 0.0));
    auto_declare<std::vector<double>>("cartesian_velocity_damping", std::vector<double>(6, 0.0));
    auto_declare<std::vector<double>>("cartesian_damping_max_torque", std::vector<double>(7, 0.0));
    auto_declare<bool>("cartesian_velocity_damping_enabled",
                       cartesian_velocity_damping_enabled_);
    auto_declare<std::vector<double>>("cartesian_workspace_lower_limits",
                                      std::vector<double>(3, -1.0));
    auto_declare<std::vector<double>>("cartesian_workspace_upper_limits",
                                      std::vector<double>(3, 1.0));
    auto_declare<std::vector<double>>("cartesian_workspace_stiffness", std::vector<double>(3, 0.0));
    auto_declare<std::vector<double>>("cartesian_workspace_damping", std::vector<double>(3, 0.0));
    auto_declare<std::vector<double>>("cartesian_workspace_max_force", std::vector<double>(3, 0.0));
    auto_declare<bool>("cartesian_workspace_enabled", cartesian_workspace_enabled_);
    auto_declare<std::vector<double>>("force_proportional_gain", std::vector<double>(6, 0.0));
    auto_declare<std::vector<double>>("force_integral_gain", std::vector<double>(6, 0.0));
    auto_declare<std::vector<double>>("force_integral_wrench_limit", std::vector<double>(6, 0.0));
    auto_declare<std::vector<double>>("force_derivative_gain", std::vector<double>(6, 0.0));
    auto_declare<std::vector<double>>("force_derivative_filter_time_constant", std::vector<double>(6, 0.0));
    auto_declare<std::vector<double>>("force_derivative_wrench_limit", std::vector<double>(6, 0.0));
    auto_declare<std::vector<double>>("force_output_wrench_limit", std::vector<double>(6, 0.0));
    auto_declare<std::vector<double>>("force_output_wrench_rate_limit", std::vector<double>(6, 1.0));
    auto_declare<bool>("cartesian_force_enabled", cartesian_force_enabled_);
    auto_declare<std::vector<double>>("adaptation_contact_stiffness_gain", std::vector<double>(6, 0.0));
    auto_declare<std::vector<double>>("adaptation_stiffness_lower_limit", std::vector<double>(6, 0.0));
    auto_declare<std::vector<double>>("adaptation_stiffness_upper_limit", std::vector<double>(6, 1.0));
    auto_declare<std::vector<double>>("adaptation_stiffness_rate_limit", std::vector<double>(6, 1.0));
    auto_declare<bool>("cartesian_impedance_adaptation_enabled",
                       cartesian_impedance_adaptation_enabled_);
    auto_declare<double>("nullspace_singularity_damping", nullspace_singularity_damping_);
    auto_declare<std::vector<double>>("nullspace_effort_limit", std::vector<double>(7, 0.0));
    auto_declare<bool>("nullspace_enabled", nullspace_enabled_);
    auto_declare<bool>("allow_runtime_nullspace_commands", allow_runtime_nullspace_commands_);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to declare parameters: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MiosEffortController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_) {
    config.names.push_back(joint + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
MiosEffortController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
    config.names.push_back(joint + "/effort");
  }
  if (franka_model_) {
    const auto model_interfaces = franka_model_->get_state_interface_names();
    config.names.insert(config.names.end(), model_interfaces.begin(), model_interfaces.end());
  }
  // FrankaRobotModel already claims this interface when model terms are
  // enabled.  Do not request it twice from controller_manager.
  if (robot_state_safety_enabled_ && robot_state_source_ == "hardware" && !franka_model_) {
    config.names.push_back(arm_id_ + "/robot_state");
  }
  return config;
}

bool MiosEffortController::configure_parameters() {
  joints_ = get_node()->get_parameter("joints").as_string_array();
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  const auto limits = get_node()->get_parameter("effort_limits").as_double_array();
  effort_rate_limit_ = get_node()->get_parameter("effort_rate_limit").as_double();
  command_timeout_ = get_node()->get_parameter("command_timeout").as_double();
  robot_state_source_ = get_node()->get_parameter("robot_state_source").as_string();
  robot_state_topic_ = get_node()->get_parameter("robot_state_topic").as_string();
  robot_state_timeout_ = get_node()->get_parameter("robot_state_timeout").as_double();
  robot_state_safety_enabled_ =
      get_node()->get_parameter("robot_state_safety_enabled").as_bool();
  activation_velocity_threshold_ =
      get_node()->get_parameter("activation_velocity_threshold").as_double();
  allow_effort_activation_ = get_node()->get_parameter("allow_effort_activation").as_bool();
  allow_external_effort_commands_ =
      get_node()->get_parameter("allow_external_effort_commands").as_bool();
  allow_runtime_effort_commands_ =
      get_node()->get_parameter("allow_runtime_effort_commands").as_bool();
  allow_runtime_actuator_commands_ =
      get_node()->get_parameter("allow_runtime_actuator_commands").as_bool();
  allow_runtime_cartesian_actuator_commands_ =
      get_node()->get_parameter("allow_runtime_cartesian_actuator_commands").as_bool();
  allow_runtime_cartesian_force_commands_ =
      get_node()->get_parameter("allow_runtime_cartesian_force_commands").as_bool();
  const auto lower_limits = get_node()->get_parameter("joint_lower_limits").as_double_array();
  const auto upper_limits = get_node()->get_parameter("joint_upper_limits").as_double_array();
  const auto wall_stiffness = get_node()->get_parameter("joint_wall_stiffness").as_double_array();
  const auto wall_damping = get_node()->get_parameter("joint_wall_damping").as_double_array();
  const auto wall_max_torque =
      get_node()->get_parameter("joint_wall_max_torque").as_double_array();
  joint_safety_enabled_ = get_node()->get_parameter("joint_safety_enabled").as_bool();
  const auto hold_stiffness = get_node()->get_parameter("hold_stiffness").as_double_array();
  const auto hold_damping = get_node()->get_parameter("hold_damping").as_double_array();
  const auto hold_max_torque = get_node()->get_parameter("hold_max_torque").as_double_array();
  position_hold_enabled_ = get_node()->get_parameter("position_hold_enabled").as_bool();
  effort_diagnostic_publish_rate_ =
      get_node()->get_parameter("effort_diagnostic_publish_rate").as_double();
  const auto cartesian_velocity_threshold =
      get_node()->get_parameter("cartesian_velocity_threshold").as_double_array();
  const auto cartesian_velocity_damping =
      get_node()->get_parameter("cartesian_velocity_damping").as_double_array();
  const auto cartesian_damping_max_torque =
      get_node()->get_parameter("cartesian_damping_max_torque").as_double_array();
  cartesian_velocity_damping_enabled_ =
      get_node()->get_parameter("cartesian_velocity_damping_enabled").as_bool();
  const auto workspace_lower_limits =
      get_node()->get_parameter("cartesian_workspace_lower_limits").as_double_array();
  const auto workspace_upper_limits =
      get_node()->get_parameter("cartesian_workspace_upper_limits").as_double_array();
  const auto workspace_stiffness =
      get_node()->get_parameter("cartesian_workspace_stiffness").as_double_array();
  const auto workspace_damping =
      get_node()->get_parameter("cartesian_workspace_damping").as_double_array();
  const auto workspace_max_force =
      get_node()->get_parameter("cartesian_workspace_max_force").as_double_array();
  cartesian_workspace_enabled_ =
      get_node()->get_parameter("cartesian_workspace_enabled").as_bool();
  const auto force_proportional_gain =
      get_node()->get_parameter("force_proportional_gain").as_double_array();
  const auto force_integral_gain = get_node()->get_parameter("force_integral_gain").as_double_array();
  const auto force_integral_wrench_limit =
      get_node()->get_parameter("force_integral_wrench_limit").as_double_array();
  const auto force_derivative_gain =
      get_node()->get_parameter("force_derivative_gain").as_double_array();
  const auto force_derivative_filter_time_constant =
      get_node()->get_parameter("force_derivative_filter_time_constant").as_double_array();
  const auto force_derivative_wrench_limit =
      get_node()->get_parameter("force_derivative_wrench_limit").as_double_array();
  const auto force_output_wrench_limit =
      get_node()->get_parameter("force_output_wrench_limit").as_double_array();
  const auto force_output_wrench_rate_limit =
      get_node()->get_parameter("force_output_wrench_rate_limit").as_double_array();
  cartesian_force_enabled_ = get_node()->get_parameter("cartesian_force_enabled").as_bool();
  const auto adaptation_contact_stiffness_gain =
      get_node()->get_parameter("adaptation_contact_stiffness_gain").as_double_array();
  const auto adaptation_stiffness_lower_limit =
      get_node()->get_parameter("adaptation_stiffness_lower_limit").as_double_array();
  const auto adaptation_stiffness_upper_limit =
      get_node()->get_parameter("adaptation_stiffness_upper_limit").as_double_array();
  const auto adaptation_stiffness_rate_limit =
      get_node()->get_parameter("adaptation_stiffness_rate_limit").as_double_array();
  cartesian_impedance_adaptation_enabled_ =
      get_node()->get_parameter("cartesian_impedance_adaptation_enabled").as_bool();
  nullspace_singularity_damping_ =
      get_node()->get_parameter("nullspace_singularity_damping").as_double();
  const auto nullspace_effort_limit =
      get_node()->get_parameter("nullspace_effort_limit").as_double_array();
  nullspace_enabled_ = get_node()->get_parameter("nullspace_enabled").as_bool();
  allow_runtime_nullspace_commands_ =
      get_node()->get_parameter("allow_runtime_nullspace_commands").as_bool();
  if (joints_.size() != effort_limits_.size() || limits.size() != effort_limits_.size() ||
      arm_id_.empty() || robot_state_source_.empty() || robot_state_topic_.empty() ||
      lower_limits.size() != effort_limits_.size() ||
      upper_limits.size() != effort_limits_.size() ||
      wall_stiffness.size() != effort_limits_.size() ||
      wall_damping.size() != effort_limits_.size() ||
      wall_max_torque.size() != effort_limits_.size() ||
      hold_stiffness.size() != effort_limits_.size() ||
      hold_damping.size() != effort_limits_.size() ||
      hold_max_torque.size() != effort_limits_.size() ||
      cartesian_velocity_threshold.size() != cartesian_velocity_threshold_.size() ||
      cartesian_velocity_damping.size() != cartesian_velocity_damping_.size() ||
      cartesian_damping_max_torque.size() != effort_limits_.size() ||
      workspace_lower_limits.size() != workspace_lower_limits_.size() ||
      workspace_upper_limits.size() != workspace_upper_limits_.size() ||
      workspace_stiffness.size() != workspace_stiffness_.size() ||
      workspace_damping.size() != workspace_damping_.size() ||
      workspace_max_force.size() != workspace_max_force_.size() ||
      force_proportional_gain.size() != force_proportional_gain_.size() ||
      force_integral_gain.size() != force_integral_gain_.size() ||
      force_integral_wrench_limit.size() != force_integral_wrench_limit_.size() ||
      force_derivative_gain.size() != force_derivative_gain_.size() ||
      force_derivative_filter_time_constant.size() != force_derivative_filter_time_constant_.size() ||
      force_derivative_wrench_limit.size() != force_derivative_wrench_limit_.size() ||
      force_output_wrench_limit.size() != force_output_wrench_limit_.size() ||
      force_output_wrench_rate_limit.size() != force_output_wrench_rate_limit_.size() ||
      adaptation_contact_stiffness_gain.size() != adaptation_contact_stiffness_gain_.size() ||
      adaptation_stiffness_lower_limit.size() != adaptation_stiffness_lower_limit_.size() ||
      adaptation_stiffness_upper_limit.size() != adaptation_stiffness_upper_limit_.size() ||
      adaptation_stiffness_rate_limit.size() != adaptation_stiffness_rate_limit_.size() ||
      nullspace_effort_limit.size() != nullspace_effort_limit_.size() ||
      !std::isfinite(effort_rate_limit_) || effort_rate_limit_ <= 0.0 ||
      !std::isfinite(command_timeout_) || command_timeout_ <= 0.0 ||
      !std::isfinite(robot_state_timeout_) || robot_state_timeout_ <= 0.0 ||
      !std::isfinite(effort_diagnostic_publish_rate_) || effort_diagnostic_publish_rate_ < 0.0 ||
      !std::isfinite(activation_velocity_threshold_) || activation_velocity_threshold_ <= 0.0 ||
      !std::isfinite(nullspace_singularity_damping_) || nullspace_singularity_damping_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected exactly 7 joints and effort limits; rate limits and timeouts must be positive.");
    return false;
  }
  if (robot_state_source_ != "hardware" && robot_state_source_ != "topic") {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "robot_state_source must be either 'hardware' or 'topic'.");
    return false;
  }
  for (size_t index = 0; index < limits.size(); ++index) {
    if (!std::isfinite(limits[index]) || limits[index] <= 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "effort_limits must contain positive finite values.");
      return false;
    }
    effort_limits_[index] = limits[index];
    if (!std::isfinite(lower_limits[index]) || !std::isfinite(upper_limits[index]) ||
        lower_limits[index] >= upper_limits[index] || !std::isfinite(wall_stiffness[index]) ||
        !std::isfinite(wall_damping[index]) || !std::isfinite(wall_max_torque[index]) ||
        wall_stiffness[index] < 0.0 || wall_damping[index] < 0.0 ||
        wall_max_torque[index] < 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid joint safety configuration.");
      return false;
    }
    joint_lower_limits_[index] = lower_limits[index];
    joint_upper_limits_[index] = upper_limits[index];
    joint_wall_stiffness_[index] = wall_stiffness[index];
    joint_wall_damping_[index] = wall_damping[index];
    joint_wall_max_torque_[index] = wall_max_torque[index];
    if (!std::isfinite(hold_stiffness[index]) || !std::isfinite(hold_damping[index]) ||
        !std::isfinite(hold_max_torque[index]) || hold_stiffness[index] < 0.0 ||
        hold_damping[index] < 0.0 || hold_max_torque[index] < 0.0 ||
        (position_hold_enabled_ &&
         (hold_max_torque[index] <= 0.0 || hold_max_torque[index] > limits[index]))) {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid position-hold configuration.");
      return false;
    }
    hold_stiffness_[index] = hold_stiffness[index];
    hold_damping_[index] = hold_damping[index];
    hold_max_torque_[index] = hold_max_torque[index];
  }
  for (size_t index = 0; index < cartesian_velocity_threshold_.size(); ++index) {
    if (!std::isfinite(cartesian_velocity_threshold[index]) ||
        !std::isfinite(cartesian_velocity_damping[index]) ||
        cartesian_velocity_threshold[index] < 0.0 || cartesian_velocity_damping[index] < 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid Cartesian velocity-damping configuration.");
      return false;
    }
    cartesian_velocity_threshold_[index] = cartesian_velocity_threshold[index];
    cartesian_velocity_damping_[index] = cartesian_velocity_damping[index];
  }
  for (size_t index = 0; index < cartesian_damping_max_torque.size(); ++index) {
    if (!std::isfinite(cartesian_damping_max_torque[index]) ||
        cartesian_damping_max_torque[index] < 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid Cartesian damping torque limit.");
      return false;
    }
    cartesian_damping_max_torque_[index] = cartesian_damping_max_torque[index];
  }
  for (size_t index = 0; index < workspace_lower_limits_.size(); ++index) {
    if (!std::isfinite(workspace_lower_limits[index]) ||
        !std::isfinite(workspace_upper_limits[index]) ||
        workspace_lower_limits[index] >= workspace_upper_limits[index] ||
        !std::isfinite(workspace_stiffness[index]) || !std::isfinite(workspace_damping[index]) ||
        !std::isfinite(workspace_max_force[index]) || workspace_stiffness[index] < 0.0 ||
        workspace_damping[index] < 0.0 || workspace_max_force[index] < 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid Cartesian workspace configuration.");
      return false;
    }
    workspace_lower_limits_[index] = workspace_lower_limits[index];
    workspace_upper_limits_[index] = workspace_upper_limits[index];
    workspace_stiffness_[index] = workspace_stiffness[index];
    workspace_damping_[index] = workspace_damping[index];
    workspace_max_force_[index] = workspace_max_force[index];
  }
  for (size_t index = 0; index < force_proportional_gain_.size(); ++index) {
    if (!std::isfinite(force_proportional_gain[index]) || !std::isfinite(force_integral_gain[index]) ||
        !std::isfinite(force_integral_wrench_limit[index]) ||
        !std::isfinite(force_derivative_gain[index]) ||
        !std::isfinite(force_derivative_filter_time_constant[index]) ||
        !std::isfinite(force_derivative_wrench_limit[index]) ||
        !std::isfinite(force_output_wrench_limit[index]) ||
        !std::isfinite(force_output_wrench_rate_limit[index]) ||
        force_proportional_gain[index] < 0.0 || force_integral_gain[index] < 0.0 ||
        force_integral_wrench_limit[index] < 0.0 || force_derivative_gain[index] < 0.0 ||
        force_derivative_filter_time_constant[index] < 0.0 ||
        force_derivative_wrench_limit[index] < 0.0 || force_output_wrench_limit[index] < 0.0 ||
        force_output_wrench_rate_limit[index] <= 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid Cartesian-force configuration.");
      return false;
    }
    force_proportional_gain_[index] = force_proportional_gain[index];
    force_integral_gain_[index] = force_integral_gain[index];
    force_integral_wrench_limit_[index] = force_integral_wrench_limit[index];
    force_derivative_gain_[index] = force_derivative_gain[index];
    force_derivative_filter_time_constant_[index] = force_derivative_filter_time_constant[index];
    force_derivative_wrench_limit_[index] = force_derivative_wrench_limit[index];
    force_output_wrench_limit_[index] = force_output_wrench_limit[index];
    force_output_wrench_rate_limit_[index] = force_output_wrench_rate_limit[index];
    if (!std::isfinite(adaptation_contact_stiffness_gain[index]) ||
        !std::isfinite(adaptation_stiffness_lower_limit[index]) ||
        !std::isfinite(adaptation_stiffness_upper_limit[index]) ||
        !std::isfinite(adaptation_stiffness_rate_limit[index]) ||
        adaptation_contact_stiffness_gain[index] < 0.0 ||
        adaptation_stiffness_lower_limit[index] < 0.0 ||
        adaptation_stiffness_lower_limit[index] > adaptation_stiffness_upper_limit[index] ||
        adaptation_stiffness_rate_limit[index] <= 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid Cartesian impedance-adaptation configuration.");
      return false;
    }
    adaptation_contact_stiffness_gain_[index] = adaptation_contact_stiffness_gain[index];
    adaptation_stiffness_lower_limit_[index] = adaptation_stiffness_lower_limit[index];
    adaptation_stiffness_upper_limit_[index] = adaptation_stiffness_upper_limit[index];
    adaptation_stiffness_rate_limit_[index] = adaptation_stiffness_rate_limit[index];
  }
  for (size_t index = 0; index < nullspace_effort_limit_.size(); ++index) {
    if (!std::isfinite(nullspace_effort_limit[index]) || nullspace_effort_limit[index] < 0.0 ||
        nullspace_effort_limit[index] > effort_limits_[index]) {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid null-space effort limit.");
      return false;
    }
    nullspace_effort_limit_[index] = nullspace_effort_limit[index];
  }
  const auto timeout_nanoseconds = static_cast<std::int64_t>(command_timeout_ * 1.0e9);
  control_core_.configure(effort_limits_, effort_rate_limit_, timeout_nanoseconds);
  control_core_.configure_joint_safety(
      joint_lower_limits_, joint_upper_limits_, joint_wall_stiffness_, joint_wall_damping_,
      joint_wall_max_torque_, joint_safety_enabled_);
  control_core_.configure_position_hold(hold_stiffness_, hold_damping_, hold_max_torque_,
                                        position_hold_enabled_);
  control_core_.configure_cartesian_velocity_damping(
      cartesian_velocity_threshold_, cartesian_velocity_damping_, cartesian_damping_max_torque_,
      cartesian_velocity_damping_enabled_);
  control_core_.configure_cartesian_workspace(
      workspace_lower_limits_, workspace_upper_limits_, workspace_stiffness_, workspace_damping_,
      workspace_max_force_, cartesian_workspace_enabled_);
  control_core_.configure_cartesian_force(
      force_proportional_gain_, force_integral_gain_, force_integral_wrench_limit_,
      force_derivative_gain_, force_derivative_filter_time_constant_,
      force_derivative_wrench_limit_, force_output_wrench_limit_, force_output_wrench_rate_limit_,
      cartesian_force_enabled_);
  control_core_.configure_cartesian_impedance_adaptation(
      adaptation_contact_stiffness_gain_, adaptation_stiffness_lower_limit_,
      adaptation_stiffness_upper_limit_, adaptation_stiffness_rate_limit_,
      cartesian_impedance_adaptation_enabled_);
  control_core_.configure_nullspace(nullspace_singularity_damping_, nullspace_effort_limit_,
                                    nullspace_enabled_);
  requires_franka_model_ = cartesian_velocity_damping_enabled_ || cartesian_workspace_enabled_ ||
                           allow_runtime_cartesian_actuator_commands_ ||
                           allow_runtime_cartesian_force_commands_ ||
                           allow_runtime_nullspace_commands_;
  // The diagnostic baseline must have no non-zero torque source and no
  // high-rate transport work in the controller-manager update thread. Direct
  // loaned joint state checks at activation remain in force below.
  zero_effort_baseline_ = !robot_state_safety_enabled_;
  if (zero_effort_baseline_ &&
      (position_hold_enabled_ || allow_external_effort_commands_ ||
       allow_runtime_effort_commands_ || allow_runtime_actuator_commands_ ||
       allow_runtime_cartesian_actuator_commands_ ||
       allow_runtime_cartesian_force_commands_ || allow_runtime_nullspace_commands_ ||
       cartesian_velocity_damping_enabled_ || cartesian_workspace_enabled_ ||
       cartesian_force_enabled_ || nullspace_enabled_ || requires_franka_model_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "robot_state_safety_enabled may be false only for the exact-zero "
                 "commissioning baseline with every non-zero command path disabled.");
    return false;
  }
  if (requires_franka_model_) {
    franka_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
        arm_id_ + "/robot_model", arm_id_ + "/robot_state");
  } else {
    franka_model_.reset();
  }
  return true;
}

controller_interface::CallbackReturn MiosEffortController::on_configure(
    const rclcpp_lifecycle::State&) {
  if (!configure_parameters()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  effort_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/desired_effort", rclcpp::QoS(1).best_effort(),
      std::bind(&MiosEffortController::receive_effort, this, std::placeholders::_1));
  runtime_effort_subscription_ =
      get_node()->create_subscription<mios_msgs::msg::MiosEffortCommand>(
          "~/mios_effort_command", rclcpp::QoS(1).best_effort(),
          std::bind(&MiosEffortController::receive_runtime_effort, this,
                    std::placeholders::_1));
  runtime_actuator_subscription_ =
      get_node()->create_subscription<mios_msgs::msg::MiosActuatorCommand>(
          "~/mios_actuator_command", rclcpp::QoS(1).best_effort(),
          std::bind(&MiosEffortController::receive_runtime_actuator, this,
                    std::placeholders::_1));
  if (robot_state_safety_enabled_ && robot_state_source_ == "topic") {
    robot_state_subscription_ = get_node()->create_subscription<franka_msgs::msg::FrankaRobotState>(
        robot_state_topic_, rclcpp::QoS(1).best_effort(),
        std::bind(&MiosEffortController::receive_robot_state, this, std::placeholders::_1));
  } else {
    robot_state_subscription_.reset();
  }
  effort_diagnostic_publisher_.reset();
  effort_diagnostic_lifecycle_publisher_.reset();
  effort_diagnostic_elapsed_seconds_ = 0.0;
  if (effort_diagnostic_publish_rate_ > 0.0) {
    effort_diagnostic_lifecycle_publisher_ =
        get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
            "~/effort_diagnostic", rclcpp::QoS(1).best_effort());
    effort_diagnostic_publisher_ =
        std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
            effort_diagnostic_lifecycle_publisher_);
    // Fixed layout, populated without allocation in update(): commanded
    // effort, measured effort, position, velocity, hold reference (7 each).
    effort_diagnostic_publisher_->msg_.data.resize(35, 0.0);
  }
  desired_effort_.writeFromNonRT(std::make_shared<MiosTorqueRequest>());
  desired_actuator_.writeFromNonRT(std::make_shared<MiosJointImpedanceRequest>());
  desired_cartesian_actuator_.writeFromNonRT(
      std::make_shared<MiosCartesianImpedanceRequest>());
  desired_cartesian_force_.writeFromNonRT(std::make_shared<MiosCartesianForceRequest>());
  desired_nullspace_.writeFromNonRT(std::make_shared<MiosNullspaceRequest>());
  robot_safety_state_.writeFromNonRT(std::make_shared<RobotSafetyState>());
  robot_state_box_ = nullptr;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MiosEffortController::on_activate(
    const rclcpp_lifecycle::State&) {
  control_core_.reset();
  // This is a commissioning interlock, not a control-loop parameter.  Query
  // it at the lifecycle boundary so an operator can relock an already
  // configured inactive controller before any later activation attempt.
  bool allow_effort_activation = false;
  try {
    allow_effort_activation =
        get_node()->get_parameter("allow_effort_activation").as_bool();
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Refusing effort activation: cannot read allow_effort_activation: %s",
                 error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!allow_effort_activation) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Refusing effort activation: allow_effort_activation is false. "
                 "Complete hardware commissioning before explicitly enabling it.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!position_hold_enabled_) {
    bool allow_zero_effort_activation = false;
    try {
      // Query the gate at the lifecycle boundary so an operator can relock an
      // already configured controller before a later activation attempt.
      allow_zero_effort_activation =
          get_node()->get_parameter("allow_zero_effort_activation").as_bool();
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Refusing zero-effort activation: cannot read "
                   "allow_zero_effort_activation: %s",
                   error.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    if (!allow_zero_effort_activation) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Refusing zero-effort activation: "
                   "allow_zero_effort_activation is false. This mode must "
                   "first be commissioned against Franka's official "
                   "gravity-compensation baseline.");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  if (command_interfaces_.size() != joints_.size() ||
      state_interfaces_.size() < joints_.size() * 3) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Refusing effort activation: unexpected count of loaned joint interfaces.");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    const std::string expected_command = joints_[index] + "/effort";
    const std::string expected_position = joints_[index] + "/position";
    const std::string expected_velocity = joints_[index] + "/velocity";
    const std::string expected_effort = joints_[index] + "/effort";
    if (command_interfaces_[index].get_name() != expected_command ||
        state_interfaces_[index * 3].get_name() != expected_position ||
        state_interfaces_[index * 3 + 1].get_name() != expected_velocity ||
        state_interfaces_[index * 3 + 2].get_name() != expected_effort) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Refusing effort activation: loaned joint interface order does not match "
                   "the configured position/velocity/effort layout.");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  std::optional<franka::RobotState> activation_robot_state;
  if (robot_state_safety_enabled_ && robot_state_source_ == "hardware") {
    const std::string robot_state_interface = arm_id_ + "/robot_state";
    const auto interface = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(), [&](const auto& state_interface) {
          return state_interface.get_name() == robot_state_interface;
        });
    if (interface == state_interfaces_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Refusing effort activation: %s was not loaned by the hardware.",
                   robot_state_interface.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    const auto interface_value = interface->get_optional();
    if (!interface_value.has_value()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Refusing effort activation: unable to read %s.",
                   robot_state_interface.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    robot_state_box_ =
        bit_cast<realtime_tools::RealtimeThreadSafeBox<franka::RobotState>*>(interface_value.value());
    if (robot_state_box_ == nullptr) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Refusing effort activation: %s supplied a null state box.",
                   robot_state_interface.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    activation_robot_state = robot_state_box_->get();
    if (!ready_for_effort_control(activation_robot_state->robot_mode)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Refusing effort activation: Franka is not in IDLE or MOVE mode.");
      robot_state_box_ = nullptr;
      return controller_interface::CallbackReturn::ERROR;
    }
    for (std::size_t index = 0; index < activation_robot_state->q.size(); ++index) {
      if (!std::isfinite(activation_robot_state->q[index]) ||
          !std::isfinite(activation_robot_state->dq[index]) ||
          std::abs(activation_robot_state->dq[index]) > activation_velocity_threshold_) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Refusing effort activation: joint %zu is not stationary within %.4f rad/s.", index,
                     activation_velocity_threshold_);
        robot_state_box_ = nullptr;
        return controller_interface::CallbackReturn::ERROR;
      }
    }
  } else if (robot_state_safety_enabled_) {
    const auto robot_safety = *robot_safety_state_.readFromRT();
    const auto now_nanoseconds = get_node()->now().nanoseconds();
    if (robot_safety == nullptr || !robot_safety->ready_for_control ||
        robot_safety->received_nanoseconds <= 0 || now_nanoseconds < robot_safety->received_nanoseconds ||
        now_nanoseconds - robot_safety->received_nanoseconds >
            static_cast<std::int64_t>(robot_state_timeout_ * 1.0e9)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Refusing effort activation without a fresh IDLE-or-MOVE robot state.");
      return controller_interface::CallbackReturn::ERROR;
    }
    for (std::size_t index = 0; index < robot_safety->velocity.size(); ++index) {
      if (!std::isfinite(robot_safety->position[index]) ||
          !std::isfinite(robot_safety->velocity[index]) ||
          std::abs(robot_safety->velocity[index]) > activation_velocity_threshold_) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Refusing effort activation: joint %zu is not stationary within %.4f rad/s.", index,
                     activation_velocity_threshold_);
        return controller_interface::CallbackReturn::ERROR;
      }
    }
  }
  if (franka_model_ && !franka_model_->assign_loaned_state_interfaces(state_interfaces_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to claim Franka model state interfaces.");
    return controller_interface::CallbackReturn::ERROR;
  }
  Effort current_position{};
  for (std::size_t index = 0; index < current_position.size(); ++index) {
    current_position[index] = state_interfaces_[index * 3].get_value();
    const double current_velocity = state_interfaces_[index * 3 + 1].get_value();
    if (!std::isfinite(current_position[index]) || !std::isfinite(current_velocity) ||
        std::abs(current_velocity) > activation_velocity_threshold_) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Refusing effort activation: live joint %zu is not stationary within %.4f rad/s.",
                   index, activation_velocity_threshold_);
      if (franka_model_) {
        franka_model_->release_interfaces();
      }
      robot_state_box_ = nullptr;
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  if (position_hold_enabled_) {
    control_core_.capture_position_hold_reference(current_position);
  }
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    if (!command_interfaces_[index].set_value(0.0)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to write a zero effort command on activation.");
      if (franka_model_) {
        franka_model_->release_interfaces();
      }
      robot_state_box_ = nullptr;
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  if (effort_diagnostic_lifecycle_publisher_) {
    effort_diagnostic_lifecycle_publisher_->on_activate();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MiosEffortController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  control_core_.reset();
  if (effort_diagnostic_lifecycle_publisher_) {
    effort_diagnostic_lifecycle_publisher_->on_deactivate();
  }
  if (franka_model_) {
    franka_model_->release_interfaces();
  }
  robot_state_box_ = nullptr;
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    if (!command_interfaces_[index].set_value(0.0)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to write a zero effort command on deactivation.");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

void MiosEffortController::receive_effort(
    const std_msgs::msg::Float64MultiArray::SharedPtr message) {
  if (!allow_external_effort_commands_) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                         "Ignoring desired_effort: external torque commands are disabled.");
    return;
  }
  if (message->data.size() != effort_limits_.size()) {
    RCLCPP_WARN(get_node()->get_logger(), "Ignoring desired_effort with %zu values; expected 7.",
                message->data.size());
    return;
  }
  auto desired = std::make_shared<MiosTorqueRequest>();
  for (size_t index = 0; index < desired->effort.size(); ++index) {
    if (!std::isfinite(message->data[index])) {
      RCLCPP_WARN(get_node()->get_logger(), "Ignoring desired_effort containing a non-finite value.");
      return;
    }
    desired->effort[index] = message->data[index];
  }
  desired->received_nanoseconds = get_node()->now().nanoseconds();
  desired_effort_.writeFromNonRT(std::move(desired));
}

void MiosEffortController::receive_runtime_effort(
    const mios_msgs::msg::MiosEffortCommand::SharedPtr message) {
  if (!allow_runtime_effort_commands_) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                         "Ignoring MIOS runtime effort command: runtime commands are disabled.");
    return;
  }
  auto desired = std::make_shared<MiosTorqueRequest>();
  for (size_t index = 0; index < desired->effort.size(); ++index) {
    if (!std::isfinite(message->effort[index])) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Ignoring MIOS runtime effort command containing a non-finite value.");
      return;
    }
    desired->effort[index] = message->effort[index];
  }
  desired->user_stopped = message->user_stopped;
  // The runtime timestamp is diagnostic metadata. Freshness is measured at
  // receipt using this controller's clock, which avoids cross-host clock skew.
  desired->received_nanoseconds = get_node()->now().nanoseconds();
  desired_effort_.writeFromNonRT(std::move(desired));
}

void MiosEffortController::receive_runtime_actuator(
    const mios_msgs::msg::MiosActuatorCommand::SharedPtr message) {
  if (message->mode == mios_msgs::msg::MiosActuatorCommand::MODE_NULLSPACE) {
    if (!allow_runtime_nullspace_commands_) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                           "Ignoring MIOS null-space command: null-space commands are disabled.");
      return;
    }
    auto nullspace = std::make_shared<MiosNullspaceRequest>();
    nullspace->received_nanoseconds = get_node()->now().nanoseconds();
    nullspace->user_stopped = message->user_stopped;
    for (std::size_t index = 0; index < nullspace->position.size(); ++index) {
      const double position = message->joint_position[index];
      const double stiffness = message->joint_stiffness[index];
      const double damping = message->joint_damping[index];
      if (!std::isfinite(position) || !std::isfinite(stiffness) || !std::isfinite(damping) ||
          stiffness < 0.0 || damping < 0.0) {
        RCLCPP_WARN(get_node()->get_logger(),
                    "Ignoring invalid MIOS null-space command; clearing the active request.");
        nullspace->user_stopped = true;
        desired_nullspace_.writeFromNonRT(std::move(nullspace));
        return;
      }
      nullspace->position[index] = position;
      nullspace->stiffness[index] = stiffness;
      nullspace->damping[index] = damping;
    }
    desired_nullspace_.writeFromNonRT(std::move(nullspace));
    return;
  }
  if (message->mode == mios_msgs::msg::MiosActuatorCommand::MODE_CARTESIAN_FORCE) {
    if (!allow_runtime_cartesian_force_commands_) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                           "Ignoring MIOS Cartesian-force command: force commands are disabled.");
      return;
    }
    auto force = std::make_shared<MiosCartesianForceRequest>();
    force->received_nanoseconds = get_node()->now().nanoseconds();
    force->user_stopped = message->user_stopped;
    for (std::size_t index = 0; index < force->target_external_wrench.size(); ++index) {
      if (!std::isfinite(message->target_external_wrench[index])) {
        RCLCPP_WARN(get_node()->get_logger(),
                    "Ignoring non-finite MIOS Cartesian-force target; clearing the active force request.");
        force->user_stopped = true;
        desired_cartesian_force_.writeFromNonRT(std::move(force));
        return;
      }
      force->target_external_wrench[index] = message->target_external_wrench[index];
    }
    desired_cartesian_force_.writeFromNonRT(std::move(force));
    return;
  }
  if (!allow_runtime_actuator_commands_) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                         "Ignoring MIOS actuator command: runtime actuator commands are disabled.");
    return;
  }

  auto desired = std::make_shared<MiosJointImpedanceRequest>();
  desired->received_nanoseconds = get_node()->now().nanoseconds();
  desired->user_stopped = message->user_stopped ||
                          message->mode == mios_msgs::msg::MiosActuatorCommand::MODE_STOP;
  if (desired->user_stopped) {
    desired_actuator_.writeFromNonRT(std::move(desired));
    return;
  }
  if (message->mode == mios_msgs::msg::MiosActuatorCommand::MODE_CARTESIAN_IMPEDANCE) {
    if (!allow_runtime_cartesian_actuator_commands_) {
      RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 5000,
          "Ignoring MIOS Cartesian actuator command: Cartesian actuator commands are disabled.");
      return;
    }
    auto cartesian = std::make_shared<MiosCartesianImpedanceRequest>();
    cartesian->received_nanoseconds = get_node()->now().nanoseconds();
    cartesian->use_coriolis_compensation = message->use_coriolis_compensation;
    for (std::size_t index = 0; index < cartesian->target_base_to_end_effector.size(); ++index) {
      const double value = message->target_base_to_end_effector[index];
      if (!std::isfinite(value)) {
        RCLCPP_WARN(get_node()->get_logger(),
                    "Ignoring non-finite MIOS Cartesian target; clearing the active Cartesian request.");
        cartesian->user_stopped = true;
        desired_cartesian_actuator_.writeFromNonRT(std::move(cartesian));
        return;
      }
      cartesian->target_base_to_end_effector[index] = value;
    }
    for (std::size_t index = 0; index < cartesian->target_velocity.size(); ++index) {
      const double velocity = message->target_cartesian_velocity[index];
      const double wrench = message->cartesian_wrench_feedforward[index];
      const double stiffness = message->cartesian_stiffness[index];
      const double damping = message->cartesian_damping[index];
      if (!std::isfinite(velocity) || !std::isfinite(wrench) || !std::isfinite(stiffness) ||
          !std::isfinite(damping) || stiffness < 0.0 || damping < 0.0) {
        RCLCPP_WARN(get_node()->get_logger(),
                    "Ignoring invalid MIOS Cartesian impedance command; clearing the active Cartesian request.");
        cartesian->user_stopped = true;
        desired_cartesian_actuator_.writeFromNonRT(std::move(cartesian));
        return;
      }
      cartesian->target_velocity[index] = velocity;
      cartesian->feedforward_wrench[index] = wrench;
      cartesian->stiffness[index] = stiffness;
      cartesian->damping[index] = damping;
    }
    desired_cartesian_actuator_.writeFromNonRT(std::move(cartesian));
    return;
  }
  if (message->mode != mios_msgs::msg::MiosActuatorCommand::MODE_JOINT_IMPEDANCE) {
    RCLCPP_WARN(get_node()->get_logger(), "Ignoring unsupported MIOS actuator mode %u.",
                static_cast<unsigned>(message->mode));
    desired->user_stopped = true;
    desired_actuator_.writeFromNonRT(std::move(desired));
    return;
  }

  for (std::size_t index = 0; index < desired->position.size(); ++index) {
    const double position = message->joint_position[index];
    const double velocity = message->joint_velocity[index];
    const double feedforward = message->joint_torque_feedforward[index];
    const double stiffness = message->joint_stiffness[index];
    const double damping = message->joint_damping[index];
    if (!std::isfinite(position) || !std::isfinite(velocity) || !std::isfinite(feedforward) ||
        !std::isfinite(stiffness) || !std::isfinite(damping) || stiffness < 0.0 || damping < 0.0) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Ignoring invalid MIOS joint-impedance command; clearing the active actuator request.");
      desired->user_stopped = true;
      desired_actuator_.writeFromNonRT(std::move(desired));
      return;
    }
    desired->position[index] = position;
    desired->velocity[index] = velocity;
    desired->feedforward_effort[index] = feedforward;
    desired->stiffness[index] = stiffness;
    desired->damping[index] = damping;
  }
  desired_actuator_.writeFromNonRT(std::move(desired));
}

void MiosEffortController::receive_robot_state(
    const franka_msgs::msg::FrankaRobotState::SharedPtr message) {
  auto safety = std::make_shared<RobotSafetyState>();
  safety->received_nanoseconds = get_node()->now().nanoseconds();
  // A stationary IDLE robot is valid immediately before controller_manager
  // takes the effort interfaces, and switches to MOVE as the Franka control
  // loop starts.  All other modes are fail-safe, including guiding, reflex,
  // user stop, and automatic recovery.
  safety->ready_for_control =
      message->robot_mode == franka_msgs::msg::FrankaRobotState::ROBOT_MODE_IDLE ||
      message->robot_mode == franka_msgs::msg::FrankaRobotState::ROBOT_MODE_MOVE;
  if (message->measured_joint_state.position.size() != safety->position.size() ||
      message->measured_joint_state.velocity.size() != safety->velocity.size()) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                         "Ignoring incomplete Franka robot-state message for effort activation safety.");
    return;
  }
  const auto& external_wrench = message->o_f_ext_hat_k.wrench;
  safety->external_wrench_base = {external_wrench.force.x, external_wrench.force.y,
                                  external_wrench.force.z, external_wrench.torque.x,
                                  external_wrench.torque.y, external_wrench.torque.z};
  for (const double value : safety->external_wrench_base) {
    if (!std::isfinite(value)) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                           "Ignoring non-finite external wrench for Cartesian-force safety.");
      return;
    }
  }
  for (std::size_t index = 0; index < safety->position.size(); ++index) {
    if (!std::isfinite(message->measured_joint_state.position[index]) ||
        !std::isfinite(message->measured_joint_state.velocity[index])) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                           "Ignoring non-finite Franka robot-state message for effort activation safety.");
      return;
    }
    safety->position[index] = message->measured_joint_state.position[index];
    safety->velocity[index] = message->measured_joint_state.velocity[index];
  }
  robot_safety_state_.writeFromNonRT(std::move(safety));
}

controller_interface::return_type MiosEffortController::update(
    const rclcpp::Time&, const rclcpp::Duration& period) {
  if (zero_effort_baseline_) {
    for (std::size_t index = 0; index < joints_.size(); ++index) {
      if (!command_interfaces_[index].set_value(0.0)) {
        return controller_interface::return_type::ERROR;
      }
    }
    return controller_interface::return_type::OK;
  }
  MiosRobotState state;
  for (size_t index = 0; index < joints_.size(); ++index) {
    state.position[index] = state_interfaces_[index * 3].get_value();
    state.velocity[index] = state_interfaces_[index * 3 + 1].get_value();
    state.effort[index] = state_interfaces_[index * 3 + 2].get_value();
    if (!std::isfinite(state.position[index]) || !std::isfinite(state.velocity[index]) ||
        !std::isfinite(state.effort[index])) {
      RCLCPP_ERROR(get_node()->get_logger(), "Non-finite robot state for joint %zu.", index);
      return controller_interface::return_type::ERROR;
    }
  }
  if (franka_model_) {
    try {
      if (cartesian_velocity_damping_enabled_) {
        model_state_.body_jacobian = franka_model_->getBodyJacobian(franka::Frame::kEndEffector);
      }
      if (cartesian_workspace_enabled_ || allow_runtime_cartesian_actuator_commands_ ||
          allow_runtime_cartesian_force_commands_) {
        model_state_.zero_jacobian = franka_model_->getZeroJacobian(franka::Frame::kEndEffector);
        state.base_to_end_effector = franka_model_->getPoseMatrix(franka::Frame::kEndEffector);
      }
      if (allow_runtime_cartesian_actuator_commands_) {
        model_state_.coriolis = franka_model_->getCoriolisForceVector();
      }
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_node()->get_logger(), "Unable to read Franka model state: %s", error.what());
      return controller_interface::return_type::ERROR;
    }
  }
  const auto request = *desired_effort_.readFromRT();
  const auto actuator_request = *desired_actuator_.readFromRT();
  const auto cartesian_actuator_request = *desired_cartesian_actuator_.readFromRT();
  // controller_manager may use a steady clock for `time`, whereas incoming
  // ROS messages are timestamped with this node's ROS clock.  Compare all
  // subscription freshness timestamps against the same clock to avoid a
  // clock-domain mismatch that would incorrectly reject every command/state.
  const auto now_nanoseconds = get_node()->now().nanoseconds();
  if (robot_state_safety_enabled_ && robot_state_source_ == "hardware") {
    const auto robot_state = robot_state_box_ == nullptr ? std::optional<franka::RobotState>{}
                                                          : robot_state_box_->try_get();
    state.user_stopped = !robot_state.has_value() ||
                         !ready_for_effort_control(robot_state->robot_mode);
    if (robot_state.has_value()) {
      state.external_wrench_base = {robot_state->O_F_ext_hat_K[0], robot_state->O_F_ext_hat_K[1],
                                    robot_state->O_F_ext_hat_K[2], robot_state->O_F_ext_hat_K[3],
                                    robot_state->O_F_ext_hat_K[4], robot_state->O_F_ext_hat_K[5]};
      for (const double value : state.external_wrench_base) {
        if (!std::isfinite(value)) {
          state.user_stopped = true;
          break;
        }
      }
    }
  } else {
    const auto robot_safety = *robot_safety_state_.readFromRT();
    state.user_stopped = robot_safety == nullptr || !robot_safety->ready_for_control ||
                         robot_safety->received_nanoseconds <= 0 ||
                         now_nanoseconds < robot_safety->received_nanoseconds ||
                             now_nanoseconds - robot_safety->received_nanoseconds >
                                 static_cast<std::int64_t>(robot_state_timeout_ * 1.0e9);
    if (robot_safety != nullptr) {
      state.external_wrench_base = robot_safety->external_wrench_base;
    }
  }
  MiosTorqueRequest actuator_effort;
  const MiosTorqueRequest* selected_request = request.get();
  const bool joint_actuator_request_fresh =
      actuator_request != nullptr && actuator_request->received_nanoseconds > 0 &&
      now_nanoseconds >= actuator_request->received_nanoseconds &&
      now_nanoseconds - actuator_request->received_nanoseconds <=
          static_cast<std::int64_t>(command_timeout_ * 1.0e9);
  if (joint_actuator_request_fresh) {
    actuator_effort.received_nanoseconds = actuator_request->received_nanoseconds;
    actuator_effort.user_stopped = actuator_request->user_stopped;
    if (!actuator_effort.user_stopped &&
        !control_core_.evaluate_joint_impedance(state, *actuator_request, actuator_effort.effort)) {
      // The subscription validates the message, but retain this real-time
      // guard so invalid data cannot fall back to an older raw torque command.
      actuator_effort.user_stopped = true;
    }
    // A fresh high-level MIOS snapshot takes precedence over raw torque.
    // Independent command sources are never summed at the hardware boundary.
    selected_request = &actuator_effort;
  }
  const bool cartesian_actuator_request_fresh =
      cartesian_actuator_request != nullptr &&
      cartesian_actuator_request->received_nanoseconds > 0 &&
      now_nanoseconds >= cartesian_actuator_request->received_nanoseconds &&
      now_nanoseconds - cartesian_actuator_request->received_nanoseconds <=
          static_cast<std::int64_t>(command_timeout_ * 1.0e9);
  if (cartesian_actuator_request_fresh &&
      (!joint_actuator_request_fresh ||
       cartesian_actuator_request->received_nanoseconds > actuator_request->received_nanoseconds)) {
    actuator_effort = {};
    actuator_effort.received_nanoseconds = cartesian_actuator_request->received_nanoseconds;
    actuator_effort.user_stopped = cartesian_actuator_request->user_stopped;
    if (!actuator_effort.user_stopped &&
        !control_core_.evaluate_cartesian_impedance(state, model_state_,
                                                    *cartesian_actuator_request, period.seconds(),
                                                    actuator_effort.effort)) {
      // A malformed pose/model must not fall back to a stale joint or raw
      // command; it becomes an immediate controller fail-safe stop.
      actuator_effort.user_stopped = true;
    }
    selected_request = &actuator_effort;
  }
  const auto cartesian_force_request = *desired_cartesian_force_.readFromRT();
  const bool cartesian_force_request_fresh =
      cartesian_force_request != nullptr && cartesian_force_request->received_nanoseconds > 0 &&
      now_nanoseconds >= cartesian_force_request->received_nanoseconds &&
      now_nanoseconds - cartesian_force_request->received_nanoseconds <=
          static_cast<std::int64_t>(command_timeout_ * 1.0e9);
  if (cartesian_force_request_fresh &&
      (!joint_actuator_request_fresh ||
       cartesian_force_request->received_nanoseconds > actuator_request->received_nanoseconds) &&
      (!cartesian_actuator_request_fresh ||
       cartesian_force_request->received_nanoseconds >
           cartesian_actuator_request->received_nanoseconds)) {
    actuator_effort = {};
    actuator_effort.received_nanoseconds = cartesian_force_request->received_nanoseconds;
    actuator_effort.user_stopped = cartesian_force_request->user_stopped;
    if (!actuator_effort.user_stopped &&
        !control_core_.evaluate_cartesian_force(state, model_state_, *cartesian_force_request,
                                                period.seconds(), actuator_effort.effort)) {
      actuator_effort.user_stopped = true;
    }
    selected_request = &actuator_effort;
  } else if (!cartesian_force_request_fresh) {
    control_core_.reset_cartesian_force();
  }
  const auto nullspace_request = *desired_nullspace_.readFromRT();
  const bool nullspace_request_fresh =
      nullspace_request != nullptr && nullspace_request->received_nanoseconds > 0 &&
      now_nanoseconds >= nullspace_request->received_nanoseconds &&
      now_nanoseconds - nullspace_request->received_nanoseconds <=
          static_cast<std::int64_t>(command_timeout_ * 1.0e9);
  if (nullspace_request_fresh &&
      (!joint_actuator_request_fresh ||
       nullspace_request->received_nanoseconds > actuator_request->received_nanoseconds) &&
      (!cartesian_actuator_request_fresh ||
       nullspace_request->received_nanoseconds >
           cartesian_actuator_request->received_nanoseconds) &&
      (!cartesian_force_request_fresh ||
       nullspace_request->received_nanoseconds >
           cartesian_force_request->received_nanoseconds)) {
    actuator_effort = {};
    actuator_effort.received_nanoseconds = nullspace_request->received_nanoseconds;
    actuator_effort.user_stopped = nullspace_request->user_stopped;
    if (!actuator_effort.user_stopped &&
        !control_core_.evaluate_nullspace(state, model_state_, *nullspace_request,
                                          actuator_effort.effort)) {
      actuator_effort.user_stopped = true;
    }
    selected_request = &actuator_effort;
  }
  const Effort effort = control_core_.step(state, model_state_, selected_request,
                                            now_nanoseconds, period.seconds());
  for (size_t index = 0; index < effort.size(); ++index) {
    if (!command_interfaces_[index].set_value(effort[index])) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to write desired effort for joint %zu.", index);
      return controller_interface::return_type::ERROR;
    }
  }
  // This is a low-rate, non-blocking diagnostic only. A busy publisher drops
  // its frame rather than delaying the 1 kHz control update.
  if (effort_diagnostic_publisher_ != nullptr) {
    effort_diagnostic_elapsed_seconds_ += period.seconds();
    if (effort_diagnostic_elapsed_seconds_ >= 1.0 / effort_diagnostic_publish_rate_) {
      effort_diagnostic_elapsed_seconds_ = 0.0;
      if (effort_diagnostic_publisher_->trylock()) {
        auto& data = effort_diagnostic_publisher_->msg_.data;
        const auto& hold_reference = control_core_.position_hold_reference();
        for (std::size_t index = 0; index < effort.size(); ++index) {
          data[index] = effort[index];
          data[7 + index] = state.effort[index];
          data[14 + index] = state.position[index];
          data[21 + index] = state.velocity[index];
          data[28 + index] = hold_reference[index];
        }
        effort_diagnostic_publisher_->unlockAndPublish();
      }
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace mios_ros2_control

PLUGINLIB_EXPORT_CLASS(mios_ros2_control::MiosEffortController,
                       controller_interface::ControllerInterface)
