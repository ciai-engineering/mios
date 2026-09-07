#include <chrono>
#include <memory>
#include <string>

#include "mios_ros2_runtime/ros2_arm_command_dispatcher.hpp"
#include "mios_ros2_runtime/ros2_core_robot_backend.hpp"
#include "mios_ros2_runtime/ros2_gripper_client.hpp"
#include "mios_ros2_runtime/ros2_robot_parameter_client.hpp"
#include "mios_ros2_runtime/ros2_robot_backend.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mios_ros2_runtime {

class MiosRuntimeNode : public rclcpp::Node {
 public:
  MiosRuntimeNode() : Node("mios_runtime") {
    const auto robot_state_topic = declare_parameter<std::string>(
        "robot_state_topic", "/franka_robot_state_broadcaster/robot_state");
    const auto robot_model_topic = declare_parameter<std::string>(
        "robot_model_topic", "/mios_robot_model_broadcaster/robot_model");
    const auto effort_command_topic = declare_parameter<std::string>(
        "effort_command_topic", "/mios_effort_controller/mios_effort_command");
    const auto actuator_command_topic = declare_parameter<std::string>(
        "actuator_command_topic", "/mios_effort_controller/mios_actuator_command");
    const auto joint_velocity_command_topic = declare_parameter<std::string>(
        "joint_velocity_command_topic",
        "/mios_joint_velocity_controller/mios_actuator_command");
    const auto cartesian_velocity_command_topic = declare_parameter<std::string>(
        "cartesian_velocity_command_topic",
        "/mios_cartesian_velocity_controller/mios_actuator_command");
    const auto joint_position_command_topic = declare_parameter<std::string>(
        "joint_position_command_topic",
        "/mios_joint_position_controller/mios_actuator_command");
    const auto cartesian_pose_command_topic = declare_parameter<std::string>(
        "cartesian_pose_command_topic",
        "/mios_cartesian_pose_controller/mios_actuator_command");
    const auto gripper_grasp_action =
        declare_parameter<std::string>("gripper_grasp_action", "/franka_gripper/grasp");
    const auto gripper_move_action =
        declare_parameter<std::string>("gripper_move_action", "/franka_gripper/move");
    const auto gripper_homing_action =
        declare_parameter<std::string>("gripper_homing_action", "/franka_gripper/homing");
    const auto gripper_stop_service =
        declare_parameter<std::string>("gripper_stop_service", "/franka_gripper/stop");
    const auto gripper_state_topic =
        declare_parameter<std::string>("gripper_state_topic", "/franka_gripper/joint_states");
    const double gripper_max_width = declare_parameter<double>("gripper_max_width", 0.08);
    const auto parameter_service_namespace =
        declare_parameter<std::string>("parameter_service_namespace", "/service_server");
    const bool allow_robot_parameter_application =
        declare_parameter<bool>("allow_robot_parameter_application", false);
    const bool publish_zero_commands = declare_parameter<bool>("publish_zero_commands", false);
    const bool enable_core_task_execution =
        declare_parameter<bool>("enable_core_task_execution", false);
    const auto core_state_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(declare_parameter<double>("core_state_timeout_seconds", 0.1)));
    const auto core_gripper_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(declare_parameter<double>("core_gripper_timeout_seconds", 20.0)));
    const auto core_parameter_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(declare_parameter<double>("core_parameter_timeout_seconds", 5.0)));
    backend_ = std::make_unique<Ros2RobotBackend>(*this, robot_state_topic, robot_model_topic,
                                                   effort_command_topic, actuator_command_topic, joint_velocity_command_topic,
                                                   cartesian_velocity_command_topic,
                                                   joint_position_command_topic,
                                                   cartesian_pose_command_topic);
    command_dispatcher_ = std::make_unique<Ros2ArmCommandDispatcher>(*backend_);
    gripper_client_ = std::make_unique<Ros2GripperClient>(
        *this, gripper_grasp_action, gripper_move_action, gripper_homing_action,
        gripper_stop_service, gripper_state_topic, gripper_max_width);
    parameter_client_ = std::make_unique<Ros2RobotParameterClient>(
        *this, parameter_service_namespace, allow_robot_parameter_application);
    core_backend_ = std::make_unique<Ros2CoreRobotBackend>(
        *backend_, *command_dispatcher_, *gripper_client_, parameter_client_.get(),
        Ros2CoreRobotBackend::ParameterSnapshotProvider{}, core_state_timeout,
        core_gripper_timeout, core_parameter_timeout, enable_core_task_execution);
    if (publish_zero_commands) {
      backend_->set_state_observer(
          [this](const RobotSnapshot& snapshot) { backend_->publish_zero_effort(snapshot.user_stopped); });
    }
    RCLCPP_INFO(
        get_logger(),
        "MIOS ROS runtime started without libfranka or FCI ownership. "
        "ArmCommand dispatcher, model subscription, gripper client, parameter client, and Core backend: ready; "
        "Core task execution: %s; parameter application: %s; zero-command publishing: %s",
        enable_core_task_execution ? "enabled" : "disabled",
        allow_robot_parameter_application ? "enabled" : "disabled",
        publish_zero_commands ? "enabled" : "disabled");
  }

 private:
  std::unique_ptr<Ros2RobotBackend> backend_;
  std::unique_ptr<Ros2ArmCommandDispatcher> command_dispatcher_;
  std::unique_ptr<Ros2GripperClient> gripper_client_;
  std::unique_ptr<Ros2RobotParameterClient> parameter_client_;
  std::unique_ptr<Ros2CoreRobotBackend> core_backend_;
};

}  // namespace mios_ros2_runtime

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mios_ros2_runtime::MiosRuntimeNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
