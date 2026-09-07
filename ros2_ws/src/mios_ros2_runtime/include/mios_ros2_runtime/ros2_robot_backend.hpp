#pragma once

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "franka_msgs/msg/franka_robot_state.hpp"
#include "mios_msgs/msg/mios_actuator_command.hpp"
#include "mios_msgs/msg/mios_effort_command.hpp"
#include "mios_msgs/msg/mios_robot_model.hpp"
#include "mios_ros2_runtime/robot_model_snapshot.hpp"
#include "mios_ros2_runtime/robot_snapshot.hpp"
#include "mios_ros2_runtime/robot_state_translation.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mios_ros2_runtime {

// ROS-only backend for the non-real-time MIOS runtime. It communicates through
// ROS topics and never owns libfranka, an FCI connection, or hardware handles.
class Ros2RobotBackend {
 public:
  using StateObserver = std::function<void(const RobotSnapshot&)>;

  Ros2RobotBackend(rclcpp::Node& node, std::string robot_state_topic,
                   std::string robot_model_topic,
                   std::string effort_command_topic, std::string actuator_command_topic,
                   std::string joint_velocity_command_topic,
                   std::string cartesian_velocity_command_topic,
                   std::string joint_position_command_topic,
                   std::string cartesian_pose_command_topic);

  std::optional<RobotSnapshot> latest_snapshot() const;
  // State-only portion needed by the future legacy-Core adapter. Robot model
  // and gripper data intentionally remain separate ROS adapters.
  std::optional<mios::control::RobotState> latest_mios_robot_state() const;
  std::optional<RobotModelSnapshot> latest_robot_model() const;
  bool has_fresh_robot_state(std::chrono::nanoseconds maximum_age) const;
  bool has_fresh_robot_model(std::chrono::nanoseconds maximum_age) const;
  void set_state_observer(StateObserver observer);
  // Additional observers are used by non-real-time adapters such as the
  // legacy Core bridge. They never run in the ros2_control update thread.
  std::size_t add_state_observer(StateObserver observer);
  void remove_state_observer(std::size_t observer_id);
  bool publish_effort(const std::array<double, kJointCount>& effort, bool user_stopped = false);
  bool publish_zero_effort(bool user_stopped = false);
  bool publish_joint_impedance(const std::array<double, kJointCount>& position,
                               const std::array<double, kJointCount>& velocity,
                               const std::array<double, kJointCount>& feedforward_effort,
                               const std::array<double, kJointCount>& stiffness,
                               const std::array<double, kJointCount>& damping,
                               bool user_stopped = false);
  bool publish_cartesian_impedance(const std::array<double, 16>& target_base_to_end_effector,
                                   const std::array<double, 6>& target_velocity,
                                   const std::array<double, 6>& feedforward_wrench,
                                   const std::array<double, 6>& stiffness,
                                   const std::array<double, 6>& damping,
                                   bool use_coriolis_compensation = false,
                                   bool user_stopped = false);
  bool publish_joint_velocity(const std::array<double, kJointCount>& velocity,
                              bool user_stopped = false);
  bool publish_cartesian_velocity(const std::array<double, 6>& velocity,
                                  bool user_stopped = false);
  bool publish_joint_position(const std::array<double, kJointCount>& position,
                              bool user_stopped = false);
  bool publish_cartesian_pose(const std::array<double, 16>& base_to_end_effector,
                              bool user_stopped = false);
  bool publish_cartesian_force(const std::array<double, 6>& target_external_wrench,
                               bool user_stopped = false);
  bool publish_nullspace(const std::array<double, kJointCount>& position,
                         const std::array<double, kJointCount>& stiffness,
                         const std::array<double, kJointCount>& damping,
                         bool user_stopped = false);

 private:
  void receive_state(const franka_msgs::msg::FrankaRobotState::SharedPtr message);
  void receive_model(const mios_msgs::msg::MiosRobotModel::SharedPtr message);

  rclcpp::Node& node_;
  rclcpp::Subscription<franka_msgs::msg::FrankaRobotState>::SharedPtr state_subscription_;
  rclcpp::Subscription<mios_msgs::msg::MiosRobotModel>::SharedPtr model_subscription_;
  rclcpp::Publisher<mios_msgs::msg::MiosEffortCommand>::SharedPtr effort_publisher_;
  rclcpp::Publisher<mios_msgs::msg::MiosActuatorCommand>::SharedPtr actuator_publisher_;
  rclcpp::Publisher<mios_msgs::msg::MiosActuatorCommand>::SharedPtr joint_velocity_publisher_;
  rclcpp::Publisher<mios_msgs::msg::MiosActuatorCommand>::SharedPtr cartesian_velocity_publisher_;
  rclcpp::Publisher<mios_msgs::msg::MiosActuatorCommand>::SharedPtr joint_position_publisher_;
  rclcpp::Publisher<mios_msgs::msg::MiosActuatorCommand>::SharedPtr cartesian_pose_publisher_;
  mutable std::mutex mutex_;
  std::optional<RobotSnapshot> latest_snapshot_;
  std::optional<RobotModelSnapshot> latest_model_;
  std::chrono::steady_clock::time_point latest_state_received_{};
  std::chrono::steady_clock::time_point latest_model_received_{};
  StateObserver observer_;
  std::size_t next_observer_id_{1};
  std::vector<std::pair<std::size_t, StateObserver>> additional_observers_;
};

}  // namespace mios_ros2_runtime
