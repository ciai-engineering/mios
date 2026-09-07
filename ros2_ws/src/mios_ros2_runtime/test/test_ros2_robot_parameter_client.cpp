#include <cassert>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "mios_ros2_runtime/ros2_robot_parameter_client.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

namespace {

mios_ros2_runtime::RobotParameterSnapshot valid_snapshot() {
  mios_ros2_runtime::RobotParameterSnapshot snapshot;
  snapshot.load_inertia = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
  snapshot.tcp_frame = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  snapshot.stiffness_frame = snapshot.tcp_frame;
  snapshot.lower_torque_thresholds.fill(1.0);
  snapshot.upper_torque_thresholds.fill(2.0);
  snapshot.lower_force_thresholds.fill(1.0);
  snapshot.upper_force_thresholds.fill(2.0);
  return snapshot;
}

struct Completion {
  std::mutex mutex;
  std::condition_variable condition;
  bool complete{false};
  mios_ros2_runtime::RobotParameterOperationResult result;
};

}  // namespace

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  {
    const auto snapshot = valid_snapshot();
    auto client_node = std::make_shared<rclcpp::Node>("mios_parameter_client_test");
    bool callback_called = false;
    mios_ros2_runtime::RobotParameterOperationResult result;
    const auto capture = [&callback_called, &result](
                             const mios_ros2_runtime::RobotParameterOperationResult& callback_result) {
      callback_called = true;
      result = callback_result;
    };

    mios_ros2_runtime::Ros2RobotParameterClient disabled_client(
        *client_node, "/test/service_server", false);
    assert(!disabled_client.apply(snapshot, capture));
    assert(callback_called && !result.accepted && !result.success);

    auto server_node = std::make_shared<rclcpp::Node>("mios_parameter_service_test");
    std::mutex order_mutex;
    std::vector<std::string> call_order;
    const auto record_call = [&order_mutex, &call_order](const std::string& name) {
      std::lock_guard<std::mutex> lock(order_mutex);
      call_order.push_back(name);
    };
    const auto set_load = server_node->create_service<franka_msgs::srv::SetLoad>(
        "/test/service_server/set_load",
        [&snapshot, &record_call](const std::shared_ptr<franka_msgs::srv::SetLoad::Request> request,
                                  std::shared_ptr<franka_msgs::srv::SetLoad::Response> response) {
          assert(request->mass == snapshot.load_mass);
          assert(request->center_of_mass == snapshot.load_center_of_mass);
          assert(request->load_inertia == snapshot.load_inertia);
          record_call("load");
          response->success = true;
        });
    const auto set_tcp = server_node->create_service<franka_msgs::srv::SetTCPFrame>(
        "/test/service_server/set_tcp_frame",
        [&snapshot, &record_call](const std::shared_ptr<franka_msgs::srv::SetTCPFrame::Request> request,
                                  std::shared_ptr<franka_msgs::srv::SetTCPFrame::Response> response) {
          assert(request->transformation == snapshot.tcp_frame);
          record_call("tcp");
          response->success = true;
        });
    const auto set_collision =
        server_node->create_service<franka_msgs::srv::SetForceTorqueCollisionBehavior>(
            "/test/service_server/set_force_torque_collision_behavior",
            [&snapshot, &record_call](
                const std::shared_ptr<
                    franka_msgs::srv::SetForceTorqueCollisionBehavior::Request> request,
                std::shared_ptr<franka_msgs::srv::SetForceTorqueCollisionBehavior::Response>
                    response) {
              assert(request->lower_torque_thresholds_nominal == snapshot.lower_torque_thresholds);
              assert(request->upper_torque_thresholds_nominal == snapshot.upper_torque_thresholds);
              assert(request->lower_force_thresholds_nominal == snapshot.lower_force_thresholds);
              assert(request->upper_force_thresholds_nominal == snapshot.upper_force_thresholds);
              record_call("collision");
              response->success = true;
            });
    const auto set_stiffness_frame = server_node->create_service<franka_msgs::srv::SetStiffnessFrame>(
        "/test/service_server/set_stiffness_frame",
        [&snapshot, &record_call](
            const std::shared_ptr<franka_msgs::srv::SetStiffnessFrame::Request> request,
            std::shared_ptr<franka_msgs::srv::SetStiffnessFrame::Response> response) {
          assert(request->transformation == snapshot.stiffness_frame);
          record_call("stiffness_frame");
          response->success = true;
        });
    const auto set_cartesian = server_node->create_service<franka_msgs::srv::SetCartesianStiffness>(
        "/test/service_server/set_cartesian_stiffness",
        [&snapshot, &record_call](
            const std::shared_ptr<franka_msgs::srv::SetCartesianStiffness::Request> request,
            std::shared_ptr<franka_msgs::srv::SetCartesianStiffness::Response> response) {
          assert(request->cartesian_stiffness == snapshot.cartesian_stiffness);
          record_call("cartesian_stiffness");
          response->success = true;
        });
    const auto set_joint = server_node->create_service<franka_msgs::srv::SetJointStiffness>(
        "/test/service_server/set_joint_stiffness",
        [&snapshot, &record_call](
            const std::shared_ptr<franka_msgs::srv::SetJointStiffness::Request> request,
            std::shared_ptr<franka_msgs::srv::SetJointStiffness::Response> response) {
          assert(request->joint_stiffness == snapshot.joint_stiffness);
          record_call("joint_stiffness");
          response->success = true;
        });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(client_node);
    executor.add_node(server_node);
    std::thread spin_thread([&executor] { executor.spin(); });

    mios_ros2_runtime::Ros2RobotParameterClient enabled_client(
        *client_node, "/test/service_server", true);
    Completion completion;
    const auto wait_for_completion = [&completion](
                                         const mios_ros2_runtime::RobotParameterOperationResult& callback_result) {
      std::lock_guard<std::mutex> lock(completion.mutex);
      completion.result = callback_result;
      completion.complete = true;
      completion.condition.notify_all();
    };
    bool accepted = false;
    for (int attempt = 0; attempt < 100 && !accepted; ++attempt) {
      {
        std::lock_guard<std::mutex> lock(completion.mutex);
        completion.complete = false;
      }
      accepted = enabled_client.apply(snapshot, wait_for_completion);
      if (!accepted) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    assert(accepted);
    {
      std::unique_lock<std::mutex> lock(completion.mutex);
      assert(completion.condition.wait_for(lock, std::chrono::seconds(2),
                                           [&completion] { return completion.complete; }));
      assert(completion.result.accepted && completion.result.success);
    }
    {
      std::lock_guard<std::mutex> lock(order_mutex);
      assert((call_order == std::vector<std::string>{"load", "tcp", "collision",
                                                      "stiffness_frame", "cartesian_stiffness",
                                                      "joint_stiffness"}));
    }
    executor.cancel();
    spin_thread.join();
    executor.remove_node(server_node);
    executor.remove_node(client_node);
  }
  rclcpp::shutdown();
  return 0;
}
