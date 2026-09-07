#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "mios_ros2_runtime/ros2_arm_command_dispatcher.hpp"
#include "mios_ros2_runtime/ros2_robot_backend.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

using namespace std::chrono_literals;
using mios::control::ArmCommand;
using mios::control::CommandMode;
using mios_msgs::msg::MiosActuatorCommand;
using mios_msgs::msg::MiosEffortCommand;

bool spin_until(rclcpp::executors::SingleThreadedExecutor& executor,
                const std::function<bool()>& predicate) {
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(2ms);
  }
  executor.spin_some();
  return predicate();
}

void spin_for(rclcpp::executors::SingleThreadedExecutor& executor,
              const std::chrono::milliseconds duration) {
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(2ms);
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  {
    auto runtime_node = std::make_shared<rclcpp::Node>("mios_arm_dispatch_runtime_test");
    auto monitor_node = std::make_shared<rclcpp::Node>("mios_arm_dispatch_monitor_test");
    const auto qos = rclcpp::QoS(10).best_effort();

    std::vector<MiosEffortCommand> effort_messages;
    std::vector<MiosActuatorCommand> joint_velocity_messages;
    std::vector<MiosActuatorCommand> cartesian_velocity_messages;
    std::vector<MiosActuatorCommand> joint_position_messages;
    std::vector<MiosActuatorCommand> cartesian_pose_messages;

    const auto effort_subscription = monitor_node->create_subscription<MiosEffortCommand>(
        "/test/effort", qos, [&effort_messages](const MiosEffortCommand::SharedPtr message) {
          effort_messages.push_back(*message);
        });
    const auto joint_velocity_subscription =
        monitor_node->create_subscription<MiosActuatorCommand>(
            "/test/joint_velocity", qos,
            [&joint_velocity_messages](const MiosActuatorCommand::SharedPtr message) {
              joint_velocity_messages.push_back(*message);
            });
    const auto cartesian_velocity_subscription =
        monitor_node->create_subscription<MiosActuatorCommand>(
            "/test/cartesian_velocity", qos,
            [&cartesian_velocity_messages](const MiosActuatorCommand::SharedPtr message) {
              cartesian_velocity_messages.push_back(*message);
            });
    const auto joint_position_subscription = monitor_node->create_subscription<MiosActuatorCommand>(
        "/test/joint_position", qos,
        [&joint_position_messages](const MiosActuatorCommand::SharedPtr message) {
          joint_position_messages.push_back(*message);
        });
    const auto cartesian_pose_subscription = monitor_node->create_subscription<MiosActuatorCommand>(
        "/test/cartesian_pose", qos,
        [&cartesian_pose_messages](const MiosActuatorCommand::SharedPtr message) {
          cartesian_pose_messages.push_back(*message);
        });

    mios_ros2_runtime::Ros2RobotBackend backend(
        *runtime_node, "/test/robot_state", "/test/robot_model", "/test/effort", "/test/actuator",
        "/test/joint_velocity", "/test/cartesian_velocity", "/test/joint_position",
        "/test/cartesian_pose");
    mios_ros2_runtime::Ros2ArmCommandDispatcher dispatcher(backend);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(runtime_node);
    executor.add_node(monitor_node);
    // Wait for local DDS endpoint discovery before publishing the first command.
    spin_for(executor, 100ms);

    ArmCommand torque;
    torque.mode = CommandMode::kTorque;
    torque.joints[0] = 1.25;
    assert(dispatcher.dispatch(torque, false));
    assert(spin_until(executor, [&effort_messages] { return effort_messages.size() == 1; }));
    assert(effort_messages.back().effort[0] == 1.25);
    assert(!effort_messages.back().user_stopped);

    ArmCommand joint_velocity;
    joint_velocity.mode = CommandMode::kJointVelocity;
    joint_velocity.joints[1] = -0.4;
    assert(dispatcher.dispatch(joint_velocity, false));
    assert(spin_until(executor,
                      [&joint_velocity_messages] { return joint_velocity_messages.size() == 1; }));
    assert(joint_velocity_messages.back().mode == MiosActuatorCommand::MODE_JOINT_VELOCITY);
    assert(joint_velocity_messages.back().joint_velocity[1] == -0.4);
    assert(!joint_velocity_messages.back().user_stopped);

    ArmCommand cartesian_velocity;
    cartesian_velocity.mode = CommandMode::kCartesianVelocity;
    cartesian_velocity.cartesian[2] = 0.03;
    assert(dispatcher.dispatch(cartesian_velocity, false));
    assert(spin_until(executor, [&cartesian_velocity_messages] {
      return cartesian_velocity_messages.size() == 1;
    }));
    assert(cartesian_velocity_messages.back().mode ==
           MiosActuatorCommand::MODE_CARTESIAN_VELOCITY);
    assert(cartesian_velocity_messages.back().target_cartesian_velocity[2] == 0.03);
    assert(!cartesian_velocity_messages.back().user_stopped);

    ArmCommand joint_position;
    joint_position.mode = CommandMode::kJointPosition;
    joint_position.joints[4] = 0.6;
    assert(dispatcher.dispatch(joint_position, false));
    assert(spin_until(executor,
                      [&joint_position_messages] { return joint_position_messages.size() == 1; }));
    assert(joint_position_messages.back().mode == MiosActuatorCommand::MODE_JOINT_POSITION);
    assert(joint_position_messages.back().joint_position[4] == 0.6);
    assert(!joint_position_messages.back().user_stopped);

    ArmCommand cartesian_pose;
    cartesian_pose.mode = CommandMode::kCartesianPose;
    cartesian_pose.pose = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                           0.0, 0.0, 1.0, 0.0, 0.1, 0.2, 0.3, 1.0};
    assert(dispatcher.dispatch(cartesian_pose, false));
    assert(spin_until(executor,
                      [&cartesian_pose_messages] { return cartesian_pose_messages.size() == 1; }));
    assert(cartesian_pose_messages.back().mode == MiosActuatorCommand::MODE_CARTESIAN_POSE);
    assert(cartesian_pose_messages.back().target_base_to_end_effector[14] == 0.3);
    assert(!cartesian_pose_messages.back().user_stopped);

    // A completed torque command must publish an explicit stopped zero-effort message.
    ArmCommand completed_torque;
    completed_torque.mode = CommandMode::kTorque;
    completed_torque.joints.fill(9.0);
    completed_torque.motion_finished = true;
    assert(dispatcher.dispatch(completed_torque, false));
    assert(spin_until(executor, [&effort_messages] { return effort_messages.size() == 2; }));
    for (const double effort : effort_messages.back().effort) {
      assert(effort == 0.0);
    }
    assert(effort_messages.back().user_stopped);

    // An unknown command mode is rejected before it can reach any ROS command topic.
    ArmCommand unsupported;
    unsupported.mode = static_cast<CommandMode>(std::uint8_t{255});
    assert(!dispatcher.dispatch(unsupported, false));
    executor.spin_some();
    assert(effort_messages.size() == 2);
    assert(joint_velocity_messages.size() == 1);
    assert(cartesian_velocity_messages.size() == 1);
    assert(joint_position_messages.size() == 1);
    assert(cartesian_pose_messages.size() == 1);

    executor.remove_node(monitor_node);
    executor.remove_node(runtime_node);
    (void)effort_subscription;
    (void)joint_velocity_subscription;
    (void)cartesian_velocity_subscription;
    (void)joint_position_subscription;
    (void)cartesian_pose_subscription;
  }
  rclcpp::shutdown();
  return 0;
}
