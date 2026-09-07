#include "mios_ros2_runtime/ros2_gripper_client.hpp"

#include <cmath>
#include <memory>
#include <utility>

#include "mios_ros2_runtime/gripper_state_translation.hpp"

namespace mios_ros2_runtime {
namespace {

void complete(const std::shared_ptr<Ros2GripperClient::CompletionCallback>& callback,
              const GripperOperationResult& result) {
  if (*callback) {
    (*callback)(result);
  }
}

bool valid_nonnegative(const double value) { return std::isfinite(value) && value >= 0.0; }
bool valid_positive(const double value) { return std::isfinite(value) && value > 0.0; }

template <typename Action>
bool send_action(const typename rclcpp_action::Client<Action>::SharedPtr& client,
                 const typename Action::Goal& goal,
                 Ros2GripperClient::CompletionCallback completion_callback) {
  const auto callback =
      std::make_shared<Ros2GripperClient::CompletionCallback>(std::move(completion_callback));
  if (!client || !client->action_server_is_ready()) {
    complete(callback, {false, false, "Gripper action server is not ready."});
    return false;
  }

  typename rclcpp_action::Client<Action>::SendGoalOptions options;
  options.goal_response_callback = [callback](std::shared_ptr<rclcpp_action::ClientGoalHandle<Action>> goal_handle) {
    if (!goal_handle) {
      complete(callback, {false, false, "Gripper action goal was rejected."});
    }
  };
  options.result_callback = [callback](
                                const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult& result) {
    const bool success = result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
                         result.result->success;
    std::string message = success ? "Gripper action completed."
                                  : "Gripper action did not complete successfully.";
    if (result.result && !result.result->error.empty()) {
      message = result.result->error;
    }
    complete(callback, {true, success, std::move(message)});
  };
  client->async_send_goal(goal, options);
  return true;
}

}  // namespace

Ros2GripperClient::Ros2GripperClient(rclcpp::Node& node, std::string grasp_action,
                                     std::string move_action, std::string homing_action,
                                     std::string stop_service, std::string state_topic,
                                     const double configured_max_width)
    : configured_max_width_(std::isfinite(configured_max_width) && configured_max_width > 0.0
                                ? configured_max_width
                                : 0.08),
      state_(std::make_shared<SharedState>()) {
  grasp_client_ = rclcpp_action::create_client<Grasp>(&node, std::move(grasp_action));
  move_client_ = rclcpp_action::create_client<Move>(&node, std::move(move_action));
  homing_client_ = rclcpp_action::create_client<Homing>(&node, std::move(homing_action));
  stop_client_ = node.create_client<std_srvs::srv::Trigger>(std::move(stop_service));
  state_subscription_ = node.create_subscription<sensor_msgs::msg::JointState>(
      std::move(state_topic), rclcpp::QoS(1),
      std::bind(&Ros2GripperClient::receive_joint_state, this, std::placeholders::_1));
}

bool Ros2GripperClient::grasp(const double width, const double speed, const double force,
                              const double epsilon_inner, const double epsilon_outer,
                              CompletionCallback completion) const {
  if (!valid_nonnegative(width) || !valid_positive(speed) || !valid_nonnegative(force) ||
      !valid_nonnegative(epsilon_inner) || !valid_nonnegative(epsilon_outer)) {
    if (completion) {
      completion({false, false, "Invalid gripper grasp values."});
    }
    return false;
  }
  Grasp::Goal goal;
  goal.width = width;
  goal.speed = speed;
  goal.force = force;
  goal.epsilon.inner = epsilon_inner;
  goal.epsilon.outer = epsilon_outer;
  return send_action<Grasp>(
      grasp_client_, goal,
      [this, completion = std::move(completion)](const GripperOperationResult& result) {
        if (result.success) {
          set_grasped_from_action(true);
        }
        if (completion) {
          completion(result);
        }
      });
}

bool Ros2GripperClient::move(const double width, const double speed,
                             CompletionCallback completion) const {
  if (!valid_nonnegative(width) || !valid_positive(speed)) {
    if (completion) {
      completion({false, false, "Invalid gripper move values."});
    }
    return false;
  }
  Move::Goal goal;
  goal.width = width;
  goal.speed = speed;
  return send_action<Move>(
      move_client_, goal,
      [this, completion = std::move(completion)](const GripperOperationResult& result) {
        if (result.success) {
          set_grasped_from_action(false);
        }
        if (completion) {
          completion(result);
        }
      });
}

bool Ros2GripperClient::home(CompletionCallback completion) const {
  return send_action<Homing>(
      homing_client_, Homing::Goal{},
      [this, completion = std::move(completion)](const GripperOperationResult& result) {
        if (result.success) {
          set_grasped_from_action(false);
        }
        if (completion) {
          completion(result);
        }
      });
}

bool Ros2GripperClient::stop(CompletionCallback completion) const {
  const auto callback = std::make_shared<CompletionCallback>(std::move(completion));
  if (!stop_client_ || !stop_client_->service_is_ready()) {
    complete(callback, {false, false, "Gripper stop service is not ready."});
    return false;
  }
  stop_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [callback](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const auto response = future.get();
        complete(callback, {true, response->success, response->message});
      });
  return true;
}

std::optional<GripperStateSnapshot> Ros2GripperClient::latest_state() const {
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->latest;
}

bool Ros2GripperClient::has_fresh_state(const std::chrono::nanoseconds maximum_age) const {
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->latest.has_value() && state_->received != std::chrono::steady_clock::time_point{} &&
         std::chrono::steady_clock::now() - state_->received <= maximum_age;
}

void Ros2GripperClient::receive_joint_state(
    const sensor_msgs::msg::JointState::SharedPtr message) {
  bool is_grasped = false;
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->latest) {
      is_grasped = state_->latest->state.is_grasped;
    }
  }
  const auto snapshot = to_mios_gripper_state(*message, configured_max_width_, is_grasped);
  if (!snapshot) {
    return;
  }
  std::lock_guard<std::mutex> lock(state_->mutex);
  state_->latest = *snapshot;
  state_->received = std::chrono::steady_clock::now();
}

void Ros2GripperClient::set_grasped_from_action(const bool is_grasped) const {
  std::lock_guard<std::mutex> lock(state_->mutex);
  if (!state_->latest) {
    return;
  }
  state_->latest->state.is_grasped = is_grasped;
  state_->received = std::chrono::steady_clock::now();
}

}  // namespace mios_ros2_runtime
