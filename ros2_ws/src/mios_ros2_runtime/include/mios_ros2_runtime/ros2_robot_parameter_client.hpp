#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "franka_msgs/srv/set_cartesian_stiffness.hpp"
#include "franka_msgs/srv/set_force_torque_collision_behavior.hpp"
#include "franka_msgs/srv/set_joint_stiffness.hpp"
#include "franka_msgs/srv/set_load.hpp"
#include "franka_msgs/srv/set_stiffness_frame.hpp"
#include "franka_msgs/srv/set_tcp_frame.hpp"
#include "mios_ros2_runtime/robot_parameter_snapshot.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mios_ros2_runtime {

struct RobotParameterOperationResult {
  bool accepted{false};
  bool success{false};
  std::string message;
};

// Explicit non-real-time client for the ROS-owned Franka parameter services.
// It has no libfranka object and sends no request until apply() is called.
class Ros2RobotParameterClient {
 public:
  using CompletionCallback = std::function<void(const RobotParameterOperationResult&)>;

  Ros2RobotParameterClient(rclcpp::Node& node, std::string service_namespace,
                           bool allow_parameter_application);

  // Parameter services call into libfranka outside controller_manager's
  // real-time loop.  They must stay disabled while an arm command controller
  // owns FCI motion.  The Core backend treats this disabled state as an
  // intentional no-op, rather than turning every skill setup into a failure.
  [[nodiscard]] bool parameter_application_enabled() const {
    return allow_parameter_application_;
  }

  // Sends load, TCP frame, collision behavior, stiffness frame, Cartesian
  // stiffness, and joint stiffness in the original MIOS order. All inputs are
  // validated and all service servers must be ready before the first request.
  bool apply(const RobotParameterSnapshot& snapshot, CompletionCallback completion) const;

 private:
  struct ApplyOperation {
    RobotParameterSnapshot snapshot;
    CompletionCallback completion;
  };

  static std::string service_name(const std::string& service_namespace,
                                  const std::string& leaf);
  bool all_services_ready() const;
  void complete(const std::shared_ptr<ApplyOperation>& operation,
                RobotParameterOperationResult result) const;
  void apply_load(const std::shared_ptr<ApplyOperation>& operation) const;
  void apply_tcp_frame(const std::shared_ptr<ApplyOperation>& operation) const;
  void apply_stiffness_frame(const std::shared_ptr<ApplyOperation>& operation) const;
  void apply_collision_behavior(const std::shared_ptr<ApplyOperation>& operation) const;
  void apply_cartesian_stiffness(const std::shared_ptr<ApplyOperation>& operation) const;
  void apply_joint_stiffness(const std::shared_ptr<ApplyOperation>& operation) const;

  rclcpp::Client<franka_msgs::srv::SetLoad>::SharedPtr set_load_client_;
  rclcpp::Client<franka_msgs::srv::SetTCPFrame>::SharedPtr set_tcp_frame_client_;
  rclcpp::Client<franka_msgs::srv::SetStiffnessFrame>::SharedPtr set_stiffness_frame_client_;
  rclcpp::Client<franka_msgs::srv::SetForceTorqueCollisionBehavior>::SharedPtr
      set_collision_behavior_client_;
  rclcpp::Client<franka_msgs::srv::SetCartesianStiffness>::SharedPtr
      set_cartesian_stiffness_client_;
  rclcpp::Client<franka_msgs::srv::SetJointStiffness>::SharedPtr set_joint_stiffness_client_;
  const bool allow_parameter_application_;
  mutable std::mutex operation_mutex_;
  mutable bool operation_active_{false};
};

}  // namespace mios_ros2_runtime
