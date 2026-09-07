#include "mios_ros2_runtime/ros2_robot_parameter_client.hpp"

#include <exception>
#include <utility>

namespace mios_ros2_runtime {
namespace {

template <typename Service>
bool response_succeeded(const typename Service::Response::SharedPtr& response) {
  return response && response->success;
}

template <typename Service>
std::string response_error(const typename Service::Response::SharedPtr& response) {
  if (!response || response->error.empty()) {
    return "Franka parameter service returned an unsuccessful response.";
  }
  return response->error;
}

}  // namespace

std::string Ros2RobotParameterClient::service_name(const std::string& service_namespace,
                                                    const std::string& leaf) {
  if (service_namespace.empty() || service_namespace == "/") {
    return "/" + leaf;
  }
  return service_namespace.back() == '/' ? service_namespace + leaf
                                         : service_namespace + "/" + leaf;
}

Ros2RobotParameterClient::Ros2RobotParameterClient(
    rclcpp::Node& node, std::string service_namespace, const bool allow_parameter_application)
    : allow_parameter_application_(allow_parameter_application) {
  set_load_client_ = node.create_client<franka_msgs::srv::SetLoad>(
      service_name(service_namespace, "set_load"));
  set_tcp_frame_client_ = node.create_client<franka_msgs::srv::SetTCPFrame>(
      service_name(service_namespace, "set_tcp_frame"));
  set_stiffness_frame_client_ = node.create_client<franka_msgs::srv::SetStiffnessFrame>(
      service_name(service_namespace, "set_stiffness_frame"));
  set_collision_behavior_client_ =
      node.create_client<franka_msgs::srv::SetForceTorqueCollisionBehavior>(
          service_name(service_namespace, "set_force_torque_collision_behavior"));
  set_cartesian_stiffness_client_ = node.create_client<franka_msgs::srv::SetCartesianStiffness>(
      service_name(service_namespace, "set_cartesian_stiffness"));
  set_joint_stiffness_client_ = node.create_client<franka_msgs::srv::SetJointStiffness>(
      service_name(service_namespace, "set_joint_stiffness"));
}

bool Ros2RobotParameterClient::all_services_ready() const {
  return set_load_client_->service_is_ready() && set_tcp_frame_client_->service_is_ready() &&
         set_stiffness_frame_client_->service_is_ready() &&
         set_collision_behavior_client_->service_is_ready() &&
         set_cartesian_stiffness_client_->service_is_ready() &&
         set_joint_stiffness_client_->service_is_ready();
}

bool Ros2RobotParameterClient::apply(const RobotParameterSnapshot& snapshot,
                                     CompletionCallback completion_callback) const {
  if (!allow_parameter_application_) {
    if (completion_callback) {
      completion_callback({false, false, "Robot parameter application is disabled."});
    }
    return false;
  }
  if (!valid_robot_parameter_snapshot(snapshot)) {
    if (completion_callback) {
      completion_callback({false, false, "Invalid MIOS robot parameter snapshot."});
    }
    return false;
  }
  if (!all_services_ready()) {
    if (completion_callback) {
      completion_callback({false, false, "One or more Franka parameter services are not ready."});
    }
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    if (operation_active_) {
      if (completion_callback) {
        completion_callback({false, false, "A robot parameter operation is already active."});
      }
      return false;
    }
    operation_active_ = true;
  }
  const auto operation = std::make_shared<ApplyOperation>(
      ApplyOperation{snapshot, std::move(completion_callback)});
  try {
    apply_load(operation);
  } catch (const std::exception& error) {
    complete(operation, {true, false, error.what()});
    return false;
  }
  return true;
}

void Ros2RobotParameterClient::complete(const std::shared_ptr<ApplyOperation>& operation,
                                        RobotParameterOperationResult result) const {
  {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    operation_active_ = false;
  }
  if (operation->completion) {
    operation->completion(result);
  }
}

void Ros2RobotParameterClient::apply_load(const std::shared_ptr<ApplyOperation>& operation) const {
  auto request = std::make_shared<franka_msgs::srv::SetLoad::Request>();
  request->mass = operation->snapshot.load_mass;
  request->center_of_mass = operation->snapshot.load_center_of_mass;
  request->load_inertia = operation->snapshot.load_inertia;
  set_load_client_->async_send_request(
      request, [this, operation](rclcpp::Client<franka_msgs::srv::SetLoad>::SharedFuture future) {
        const auto response = future.get();
        if (!response_succeeded<franka_msgs::srv::SetLoad>(response)) {
          complete(operation, {true, false, response_error<franka_msgs::srv::SetLoad>(response)});
          return;
        }
        try {
          apply_tcp_frame(operation);
        } catch (const std::exception& error) {
          complete(operation, {true, false, error.what()});
        }
      });
}

void Ros2RobotParameterClient::apply_tcp_frame(
    const std::shared_ptr<ApplyOperation>& operation) const {
  auto request = std::make_shared<franka_msgs::srv::SetTCPFrame::Request>();
  request->transformation = operation->snapshot.tcp_frame;
  set_tcp_frame_client_->async_send_request(
      request,
      [this, operation](rclcpp::Client<franka_msgs::srv::SetTCPFrame>::SharedFuture future) {
        const auto response = future.get();
        if (!response_succeeded<franka_msgs::srv::SetTCPFrame>(response)) {
          complete(operation,
                   {true, false, response_error<franka_msgs::srv::SetTCPFrame>(response)});
          return;
        }
        try {
          apply_collision_behavior(operation);
        } catch (const std::exception& error) {
          complete(operation, {true, false, error.what()});
        }
      });
}

void Ros2RobotParameterClient::apply_stiffness_frame(
    const std::shared_ptr<ApplyOperation>& operation) const {
  auto request = std::make_shared<franka_msgs::srv::SetStiffnessFrame::Request>();
  request->transformation = operation->snapshot.stiffness_frame;
  set_stiffness_frame_client_->async_send_request(
      request,
      [this, operation](rclcpp::Client<franka_msgs::srv::SetStiffnessFrame>::SharedFuture future) {
        const auto response = future.get();
        if (!response_succeeded<franka_msgs::srv::SetStiffnessFrame>(response)) {
          complete(operation,
                   {true, false, response_error<franka_msgs::srv::SetStiffnessFrame>(response)});
          return;
        }
        try {
          apply_cartesian_stiffness(operation);
        } catch (const std::exception& error) {
          complete(operation, {true, false, error.what()});
        }
      });
}

void Ros2RobotParameterClient::apply_collision_behavior(
    const std::shared_ptr<ApplyOperation>& operation) const {
  auto request = std::make_shared<franka_msgs::srv::SetForceTorqueCollisionBehavior::Request>();
  request->lower_torque_thresholds_nominal = operation->snapshot.lower_torque_thresholds;
  request->upper_torque_thresholds_nominal = operation->snapshot.upper_torque_thresholds;
  request->lower_force_thresholds_nominal = operation->snapshot.lower_force_thresholds;
  request->upper_force_thresholds_nominal = operation->snapshot.upper_force_thresholds;
  set_collision_behavior_client_->async_send_request(
      request,
      [this, operation](
          rclcpp::Client<franka_msgs::srv::SetForceTorqueCollisionBehavior>::SharedFuture future) {
        const auto response = future.get();
        if (!response_succeeded<franka_msgs::srv::SetForceTorqueCollisionBehavior>(response)) {
          complete(operation, {true, false,
                               response_error<franka_msgs::srv::SetForceTorqueCollisionBehavior>(
                                   response)});
          return;
        }
        try {
          apply_stiffness_frame(operation);
        } catch (const std::exception& error) {
          complete(operation, {true, false, error.what()});
        }
      });
}

void Ros2RobotParameterClient::apply_cartesian_stiffness(
    const std::shared_ptr<ApplyOperation>& operation) const {
  auto request = std::make_shared<franka_msgs::srv::SetCartesianStiffness::Request>();
  request->cartesian_stiffness = operation->snapshot.cartesian_stiffness;
  set_cartesian_stiffness_client_->async_send_request(
      request,
      [this, operation](
          rclcpp::Client<franka_msgs::srv::SetCartesianStiffness>::SharedFuture future) {
        const auto response = future.get();
        if (!response_succeeded<franka_msgs::srv::SetCartesianStiffness>(response)) {
          complete(operation,
                   {true, false, response_error<franka_msgs::srv::SetCartesianStiffness>(
                                     response)});
          return;
        }
        try {
          apply_joint_stiffness(operation);
        } catch (const std::exception& error) {
          complete(operation, {true, false, error.what()});
        }
      });
}

void Ros2RobotParameterClient::apply_joint_stiffness(
    const std::shared_ptr<ApplyOperation>& operation) const {
  auto request = std::make_shared<franka_msgs::srv::SetJointStiffness::Request>();
  request->joint_stiffness = operation->snapshot.joint_stiffness;
  set_joint_stiffness_client_->async_send_request(
      request,
      [this, operation](rclcpp::Client<franka_msgs::srv::SetJointStiffness>::SharedFuture future) {
        const auto response = future.get();
        if (!response_succeeded<franka_msgs::srv::SetJointStiffness>(response)) {
          complete(operation,
                   {true, false, response_error<franka_msgs::srv::SetJointStiffness>(response)});
          return;
        }
        complete(operation, {true, true, "Robot parameters applied."});
      });
}

}  // namespace mios_ros2_runtime
