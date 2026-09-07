#include <cassert>
#include <memory>

#include "mios_ros2_runtime/ros2_gripper_client.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_ros2_gripper_client");
  mios_ros2_runtime::Ros2GripperClient client(
      *node, "/test/grasp", "/test/move", "/test/homing", "/test/stop", "/test/joint_states",
      0.08);

  bool callback_called = false;
  mios_ros2_runtime::GripperOperationResult result;
  const auto capture = [&callback_called, &result](
                           const mios_ros2_runtime::GripperOperationResult& callback_result) {
    callback_called = true;
    result = callback_result;
  };

  // Reject malformed requests before discovery or any ROS action request.
  assert(!client.grasp(-0.01, 0.1, 1.0, 0.005, 0.005, capture));
  assert(callback_called);
  assert(!result.accepted);
  assert(!result.success);

  callback_called = false;
  assert(!client.move(0.01, 0.0, capture));
  assert(callback_called);
  assert(!result.accepted);
  assert(!result.success);

  // With no server in this isolated test graph, explicit operations fail
  // locally instead of blocking or opening a direct gripper connection.
  callback_called = false;
  assert(!client.home(capture));
  assert(callback_called);
  assert(!result.accepted);

  callback_called = false;
  assert(!client.stop(capture));
  assert(callback_called);
  assert(!result.accepted);

  rclcpp::shutdown();
}
