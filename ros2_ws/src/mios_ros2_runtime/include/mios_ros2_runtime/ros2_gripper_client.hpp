#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "franka_msgs/action/grasp.hpp"
#include "franka_msgs/action/homing.hpp"
#include "franka_msgs/action/move.hpp"
#include "mios_ros2_runtime/gripper_state_snapshot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace mios_ros2_runtime {

struct GripperOperationResult {
  bool accepted{false};
  bool success{false};
  std::string message;
};

// Non-real-time client for the ROS-owned Franka gripper node. This class owns
// no libfranka object and sends no request until its explicit methods are
// called by a future Core/task adapter.
class Ros2GripperClient {
 public:
  using CompletionCallback = std::function<void(const GripperOperationResult&)>;

  Ros2GripperClient(rclcpp::Node& node, std::string grasp_action, std::string move_action,
                    std::string homing_action, std::string stop_service,
                    std::string state_topic, double configured_max_width);

  bool grasp(double width, double speed, double force, double epsilon_inner,
             double epsilon_outer, CompletionCallback completion) const;
  bool move(double width, double speed, CompletionCallback completion) const;
  bool home(CompletionCallback completion) const;
  bool stop(CompletionCallback completion) const;
  std::optional<GripperStateSnapshot> latest_state() const;
  bool has_fresh_state(std::chrono::nanoseconds maximum_age) const;

 private:
  using Grasp = franka_msgs::action::Grasp;
  using Move = franka_msgs::action::Move;
  using Homing = franka_msgs::action::Homing;

  struct SharedState {
    mutable std::mutex mutex;
    std::optional<GripperStateSnapshot> latest;
    std::chrono::steady_clock::time_point received{};
  };

  void receive_joint_state(const sensor_msgs::msg::JointState::SharedPtr message);
  void set_grasped_from_action(bool is_grasped) const;

  rclcpp_action::Client<Grasp>::SharedPtr grasp_client_;
  rclcpp_action::Client<Move>::SharedPtr move_client_;
  rclcpp_action::Client<Homing>::SharedPtr homing_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscription_;
  const double configured_max_width_;
  std::shared_ptr<SharedState> state_;
};

}  // namespace mios_ros2_runtime
