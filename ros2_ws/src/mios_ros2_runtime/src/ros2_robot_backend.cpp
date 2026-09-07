#include "mios_ros2_runtime/ros2_robot_backend.hpp"

#include <cmath>
#include <utility>

namespace mios_ros2_runtime {
namespace {

bool is_finite(double value) { return std::isfinite(value); }

bool has_size_and_finite_values(const std::vector<double>& values) {
  if (values.size() != kJointCount) {
    return false;
  }
  for (const double value : values) {
    if (!is_finite(value)) {
      return false;
    }
  }
  return true;
}

bool is_finite_wrench(const geometry_msgs::msg::Wrench& wrench) {
  return is_finite(wrench.force.x) && is_finite(wrench.force.y) && is_finite(wrench.force.z) &&
         is_finite(wrench.torque.x) && is_finite(wrench.torque.y) && is_finite(wrench.torque.z);
}

}  // namespace

Ros2RobotBackend::Ros2RobotBackend(rclcpp::Node& node, std::string robot_state_topic,
                                   std::string robot_model_topic,
                                   std::string effort_command_topic,
                                   std::string actuator_command_topic,
                                   std::string joint_velocity_command_topic,
                                   std::string cartesian_velocity_command_topic,
                                   std::string joint_position_command_topic,
                                   std::string cartesian_pose_command_topic)
    : node_(node) {
  state_subscription_ = node_.create_subscription<franka_msgs::msg::FrankaRobotState>(
      std::move(robot_state_topic), rclcpp::QoS(1).best_effort(),
      std::bind(&Ros2RobotBackend::receive_state, this, std::placeholders::_1));
  model_subscription_ = node_.create_subscription<mios_msgs::msg::MiosRobotModel>(
      std::move(robot_model_topic), rclcpp::QoS(1).best_effort(),
      std::bind(&Ros2RobotBackend::receive_model, this, std::placeholders::_1));
  effort_publisher_ = node_.create_publisher<mios_msgs::msg::MiosEffortCommand>(
      std::move(effort_command_topic), rclcpp::QoS(1).best_effort());
  actuator_publisher_ = node_.create_publisher<mios_msgs::msg::MiosActuatorCommand>(
      std::move(actuator_command_topic), rclcpp::QoS(1).best_effort());
  joint_velocity_publisher_ = node_.create_publisher<mios_msgs::msg::MiosActuatorCommand>(
      std::move(joint_velocity_command_topic), rclcpp::QoS(1).best_effort());
  cartesian_velocity_publisher_ = node_.create_publisher<mios_msgs::msg::MiosActuatorCommand>(
      std::move(cartesian_velocity_command_topic), rclcpp::QoS(1).best_effort());
  joint_position_publisher_ = node_.create_publisher<mios_msgs::msg::MiosActuatorCommand>(
      std::move(joint_position_command_topic), rclcpp::QoS(1).best_effort());
  cartesian_pose_publisher_ = node_.create_publisher<mios_msgs::msg::MiosActuatorCommand>(
      std::move(cartesian_pose_command_topic), rclcpp::QoS(1).best_effort());
}

std::optional<RobotSnapshot> Ros2RobotBackend::latest_snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_snapshot_;
}

std::optional<mios::control::RobotState> Ros2RobotBackend::latest_mios_robot_state() const {
  const auto snapshot = latest_snapshot();
  if (!snapshot.has_value()) {
    return std::nullopt;
  }
  return to_mios_robot_state(*snapshot);
}

std::optional<RobotModelSnapshot> Ros2RobotBackend::latest_robot_model() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_model_;
}

bool Ros2RobotBackend::has_fresh_robot_state(const std::chrono::nanoseconds maximum_age) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_snapshot_.has_value() && latest_state_received_ != std::chrono::steady_clock::time_point{} &&
         std::chrono::steady_clock::now() - latest_state_received_ <= maximum_age;
}

bool Ros2RobotBackend::has_fresh_robot_model(const std::chrono::nanoseconds maximum_age) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_model_.has_value() && latest_model_received_ != std::chrono::steady_clock::time_point{} &&
         std::chrono::steady_clock::now() - latest_model_received_ <= maximum_age;
}

void Ros2RobotBackend::set_state_observer(StateObserver observer) {
  std::lock_guard<std::mutex> lock(mutex_);
  observer_ = std::move(observer);
}

std::size_t Ros2RobotBackend::add_state_observer(StateObserver observer) {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::size_t observer_id = next_observer_id_++;
  additional_observers_.emplace_back(observer_id, std::move(observer));
  return observer_id;
}

void Ros2RobotBackend::remove_state_observer(const std::size_t observer_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  additional_observers_.erase(
      std::remove_if(additional_observers_.begin(), additional_observers_.end(),
                     [observer_id](const auto& entry) { return entry.first == observer_id; }),
      additional_observers_.end());
}

bool Ros2RobotBackend::publish_effort(const std::array<double, kJointCount>& effort,
                                      bool user_stopped) {
  for (const double value : effort) {
    if (!is_finite(value)) {
      RCLCPP_ERROR(node_.get_logger(), "Refusing a MIOS effort command with a non-finite value.");
      return false;
    }
  }
  mios_msgs::msg::MiosEffortCommand message;
  message.stamp = node_.now();
  message.effort = effort;
  message.user_stopped = user_stopped;
  effort_publisher_->publish(message);
  return true;
}

bool Ros2RobotBackend::publish_zero_effort(bool user_stopped) {
  return publish_effort({}, user_stopped);
}

bool Ros2RobotBackend::publish_joint_impedance(
    const std::array<double, kJointCount>& position,
    const std::array<double, kJointCount>& velocity,
    const std::array<double, kJointCount>& feedforward_effort,
    const std::array<double, kJointCount>& stiffness,
    const std::array<double, kJointCount>& damping, bool user_stopped) {
  for (std::size_t index = 0; index < kJointCount; ++index) {
    if (!is_finite(position[index]) || !is_finite(velocity[index]) ||
        !is_finite(feedforward_effort[index]) || !is_finite(stiffness[index]) ||
        !is_finite(damping[index]) || stiffness[index] < 0.0 || damping[index] < 0.0) {
      RCLCPP_ERROR(node_.get_logger(),
                   "Refusing a MIOS joint-impedance command with invalid values.");
      return false;
    }
  }
  mios_msgs::msg::MiosActuatorCommand message;
  message.stamp = node_.now();
  message.mode = mios_msgs::msg::MiosActuatorCommand::MODE_JOINT_IMPEDANCE;
  message.joint_position = position;
  message.joint_velocity = velocity;
  message.joint_torque_feedforward = feedforward_effort;
  message.joint_stiffness = stiffness;
  message.joint_damping = damping;
  message.user_stopped = user_stopped;
  actuator_publisher_->publish(message);
  return true;
}

bool Ros2RobotBackend::publish_cartesian_impedance(
    const std::array<double, 16>& target_base_to_end_effector,
    const std::array<double, 6>& target_velocity,
    const std::array<double, 6>& feedforward_wrench,
    const std::array<double, 6>& stiffness,
    const std::array<double, 6>& damping, bool use_coriolis_compensation,
    bool user_stopped) {
  for (const double value : target_base_to_end_effector) {
    if (!is_finite(value)) {
      RCLCPP_ERROR(node_.get_logger(),
                   "Refusing a MIOS Cartesian impedance command with a non-finite pose.");
      return false;
    }
  }
  for (std::size_t index = 0; index < target_velocity.size(); ++index) {
    if (!is_finite(target_velocity[index]) || !is_finite(feedforward_wrench[index]) ||
        !is_finite(stiffness[index]) || !is_finite(damping[index]) || stiffness[index] < 0.0 ||
        damping[index] < 0.0) {
      RCLCPP_ERROR(node_.get_logger(),
                   "Refusing a MIOS Cartesian impedance command with invalid values.");
      return false;
    }
  }
  mios_msgs::msg::MiosActuatorCommand message;
  message.stamp = node_.now();
  message.mode = mios_msgs::msg::MiosActuatorCommand::MODE_CARTESIAN_IMPEDANCE;
  message.target_base_to_end_effector = target_base_to_end_effector;
  message.target_cartesian_velocity = target_velocity;
  message.cartesian_wrench_feedforward = feedforward_wrench;
  message.cartesian_stiffness = stiffness;
  message.cartesian_damping = damping;
  message.use_coriolis_compensation = use_coriolis_compensation;
  message.user_stopped = user_stopped;
  actuator_publisher_->publish(message);
  return true;
}

bool Ros2RobotBackend::publish_joint_velocity(
    const std::array<double, kJointCount>& velocity, const bool user_stopped) {
  for (const double value : velocity) {
    if (!is_finite(value)) {
      RCLCPP_ERROR(node_.get_logger(),
                   "Refusing a MIOS joint-velocity command with a non-finite value.");
      return false;
    }
  }
  mios_msgs::msg::MiosActuatorCommand message;
  message.stamp = node_.now();
  message.mode = mios_msgs::msg::MiosActuatorCommand::MODE_JOINT_VELOCITY;
  message.joint_velocity = velocity;
  message.user_stopped = user_stopped;
  joint_velocity_publisher_->publish(message);
  return true;
}

bool Ros2RobotBackend::publish_cartesian_velocity(
    const std::array<double, 6>& velocity, const bool user_stopped) {
  for (const double value : velocity) {
    if (!is_finite(value)) {
      RCLCPP_ERROR(node_.get_logger(),
                   "Refusing a MIOS Cartesian velocity command with a non-finite value.");
      return false;
    }
  }
  mios_msgs::msg::MiosActuatorCommand message;
  message.stamp = node_.now();
  message.mode = mios_msgs::msg::MiosActuatorCommand::MODE_CARTESIAN_VELOCITY;
  message.target_cartesian_velocity = velocity;
  message.user_stopped = user_stopped;
  cartesian_velocity_publisher_->publish(message);
  return true;
}

bool Ros2RobotBackend::publish_joint_position(
    const std::array<double, kJointCount>& position, const bool user_stopped) {
  for (const double value : position) {
    if (!is_finite(value)) {
      RCLCPP_ERROR(node_.get_logger(),
                   "Refusing a MIOS joint-position command with a non-finite value.");
      return false;
    }
  }
  mios_msgs::msg::MiosActuatorCommand message;
  message.stamp = node_.now();
  message.mode = mios_msgs::msg::MiosActuatorCommand::MODE_JOINT_POSITION;
  message.joint_position = position;
  message.user_stopped = user_stopped;
  joint_position_publisher_->publish(message);
  return true;
}

bool Ros2RobotBackend::publish_cartesian_pose(
    const std::array<double, 16>& base_to_end_effector, const bool user_stopped) {
  for (const double value : base_to_end_effector) {
    if (!is_finite(value)) {
      RCLCPP_ERROR(node_.get_logger(),
                   "Refusing a MIOS Cartesian-pose command with a non-finite value.");
      return false;
    }
  }
  mios_msgs::msg::MiosActuatorCommand message;
  message.stamp = node_.now();
  message.mode = mios_msgs::msg::MiosActuatorCommand::MODE_CARTESIAN_POSE;
  message.target_base_to_end_effector = base_to_end_effector;
  message.user_stopped = user_stopped;
  cartesian_pose_publisher_->publish(message);
  return true;
}

bool Ros2RobotBackend::publish_cartesian_force(
    const std::array<double, 6>& target_external_wrench, const bool user_stopped) {
  for (const double value : target_external_wrench) {
    if (!is_finite(value)) {
      RCLCPP_ERROR(node_.get_logger(),
                   "Refusing a MIOS Cartesian-force command with a non-finite value.");
      return false;
    }
  }
  mios_msgs::msg::MiosActuatorCommand message;
  message.stamp = node_.now();
  message.mode = mios_msgs::msg::MiosActuatorCommand::MODE_CARTESIAN_FORCE;
  message.target_external_wrench = target_external_wrench;
  message.user_stopped = user_stopped;
  actuator_publisher_->publish(message);
  return true;
}

bool Ros2RobotBackend::publish_nullspace(
    const std::array<double, kJointCount>& position,
    const std::array<double, kJointCount>& stiffness,
    const std::array<double, kJointCount>& damping, const bool user_stopped) {
  for (std::size_t index = 0; index < position.size(); ++index) {
    if (!is_finite(position[index]) || !is_finite(stiffness[index]) ||
        !is_finite(damping[index]) || stiffness[index] < 0.0 || damping[index] < 0.0) {
      RCLCPP_ERROR(node_.get_logger(), "Refusing a MIOS null-space command with invalid values.");
      return false;
    }
  }
  mios_msgs::msg::MiosActuatorCommand message;
  message.stamp = node_.now();
  message.mode = mios_msgs::msg::MiosActuatorCommand::MODE_NULLSPACE;
  message.joint_position = position;
  message.joint_stiffness = stiffness;
  message.joint_damping = damping;
  message.user_stopped = user_stopped;
  actuator_publisher_->publish(message);
  return true;
}

void Ros2RobotBackend::receive_state(
    const franka_msgs::msg::FrankaRobotState::SharedPtr message) {
  const auto& measured = message->measured_joint_state;
  const auto& motors = message->measured_joint_motor_state;
  const auto& external = message->tau_ext_hat_filtered;
  const auto& pose = message->o_t_ee.pose;
  if (!has_size_and_finite_values(measured.position) ||
      !has_size_and_finite_values(measured.velocity) ||
      !has_size_and_finite_values(measured.effort) ||
      !has_size_and_finite_values(motors.position) ||
      !has_size_and_finite_values(motors.velocity) ||
      !has_size_and_finite_values(external.effort) || !is_finite_wrench(message->o_f_ext_hat_k.wrench) ||
      !is_finite_wrench(message->k_f_ext_hat_k.wrench) || !is_finite(pose.position.x) ||
      !is_finite(pose.position.y) || !is_finite(pose.position.z) || !is_finite(pose.orientation.x) ||
      !is_finite(pose.orientation.y) || !is_finite(pose.orientation.z) ||
      !is_finite(pose.orientation.w)) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "Ignoring incomplete or non-finite Franka robot-state message.");
    return;
  }

  const double qx = pose.orientation.x;
  const double qy = pose.orientation.y;
  const double qz = pose.orientation.z;
  const double qw = pose.orientation.w;
  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (norm <= 1.0e-12) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "Ignoring Franka robot-state message with a zero-norm orientation.");
    return;
  }
  const double x = qx / norm;
  const double y = qy / norm;
  const double z = qz / norm;
  const double w = qw / norm;

  RobotSnapshot snapshot;
  for (size_t index = 0; index < kJointCount; ++index) {
    snapshot.position[index] = measured.position[index];
    snapshot.velocity[index] = measured.velocity[index];
    snapshot.effort[index] = measured.effort[index];
    snapshot.motor_position[index] = motors.position[index];
    snapshot.motor_velocity[index] = motors.velocity[index];
    snapshot.external_effort[index] = external.effort[index];
  }
  // Eigen and the legacy MIOS percept interpret this 4x4 representation in
  // column-major order, matching libfranka's O_T_EE convention.
  snapshot.base_to_end_effector = {
      1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + z * w),
      2.0 * (x * z - y * w),       0.0,
      2.0 * (x * y - z * w),       1.0 - 2.0 * (x * x + z * z),
      2.0 * (y * z + x * w),       0.0,
      2.0 * (x * z + y * w),       2.0 * (y * z - x * w),
      1.0 - 2.0 * (x * x + y * y), 0.0,
      pose.position.x,              pose.position.y,
      pose.position.z,              1.0};
  snapshot.external_wrench_origin = {
      message->o_f_ext_hat_k.wrench.force.x, message->o_f_ext_hat_k.wrench.force.y,
      message->o_f_ext_hat_k.wrench.force.z, message->o_f_ext_hat_k.wrench.torque.x,
      message->o_f_ext_hat_k.wrench.torque.y, message->o_f_ext_hat_k.wrench.torque.z};
  snapshot.external_wrench_stiffness = {
      message->k_f_ext_hat_k.wrench.force.x, message->k_f_ext_hat_k.wrench.force.y,
      message->k_f_ext_hat_k.wrench.force.z, message->k_f_ext_hat_k.wrench.torque.x,
      message->k_f_ext_hat_k.wrench.torque.y, message->k_f_ext_hat_k.wrench.torque.z};
  snapshot.stamp_nanoseconds = rclcpp::Time(message->header.stamp).nanoseconds();
  switch (message->robot_mode) {
    case franka_msgs::msg::FrankaRobotState::ROBOT_MODE_IDLE:
      snapshot.robot_mode = RobotRuntimeMode::kIdle;
      break;
    case franka_msgs::msg::FrankaRobotState::ROBOT_MODE_MOVE:
      snapshot.robot_mode = RobotRuntimeMode::kMove;
      break;
    case franka_msgs::msg::FrankaRobotState::ROBOT_MODE_GUIDING:
      snapshot.robot_mode = RobotRuntimeMode::kGuiding;
      break;
    case franka_msgs::msg::FrankaRobotState::ROBOT_MODE_REFLEX:
      snapshot.robot_mode = RobotRuntimeMode::kReflex;
      break;
    case franka_msgs::msg::FrankaRobotState::ROBOT_MODE_USER_STOPPED:
      snapshot.robot_mode = RobotRuntimeMode::kUserStopped;
      break;
    case franka_msgs::msg::FrankaRobotState::ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY:
      snapshot.robot_mode = RobotRuntimeMode::kAutomaticErrorRecovery;
      break;
    case franka_msgs::msg::FrankaRobotState::ROBOT_MODE_OTHER:
    default:
      snapshot.robot_mode = RobotRuntimeMode::kOther;
      break;
  }
  snapshot.user_stopped = snapshot.robot_mode == RobotRuntimeMode::kUserStopped;

  std::vector<StateObserver> observers;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_snapshot_ = snapshot;
    latest_state_received_ = std::chrono::steady_clock::now();
    if (observer_) {
      observers.push_back(observer_);
    }
    for (const auto& entry : additional_observers_) {
      observers.push_back(entry.second);
    }
  }
  for (const auto& observer : observers) {
    observer(snapshot);
  }
}

void Ros2RobotBackend::receive_model(const mios_msgs::msg::MiosRobotModel::SharedPtr message) {
  const auto is_finite_array = [](const auto& values) {
    for (const double value : values) {
      if (!is_finite(value)) {
        return false;
      }
    }
    return true;
  };
  if (!is_finite_array(message->mass) || !is_finite_array(message->coriolis) ||
      !is_finite_array(message->gravity) || !is_finite_array(message->body_jacobian) ||
      !is_finite_array(message->zero_jacobian)) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "Ignoring non-finite MIOS robot-model message.");
    return;
  }
  RobotModelSnapshot snapshot;
  snapshot.model.mass = message->mass;
  snapshot.model.coriolis = message->coriolis;
  snapshot.model.gravity = message->gravity;
  snapshot.model.body_jacobian = message->body_jacobian;
  snapshot.model.zero_jacobian = message->zero_jacobian;
  snapshot.stamp_nanoseconds = rclcpp::Time(message->stamp).nanoseconds();
  std::lock_guard<std::mutex> lock(mutex_);
  latest_model_ = snapshot;
  latest_model_received_ = std::chrono::steady_clock::now();
}

}  // namespace mios_ros2_runtime
