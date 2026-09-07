#include <cassert>
#include <chrono>
#include <memory>
#include <optional>

#include "mios_ros2_runtime/ros2_arm_command_dispatcher.hpp"
#include "mios_ros2_runtime/ros2_core_robot_backend.hpp"
#include "mios_ros2_runtime/ros2_gripper_client.hpp"
#include "mios_ros2_runtime/ros2_robot_parameter_client.hpp"
#include "mios_ros2_runtime/ros2_robot_backend.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

mios::control::RobotParameters valid_robot_parameters() {
  mios::control::RobotParameters parameters;
  parameters.load_inertia = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
  parameters.tcp_frame = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                          0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  parameters.stiffness_frame = parameters.tcp_frame;
  parameters.lower_torque_thresholds.fill(1.0);
  parameters.upper_torque_thresholds.fill(2.0);
  parameters.lower_force_thresholds.fill(1.0);
  parameters.upper_force_thresholds.fill(2.0);
  return parameters;
}

}  // namespace

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  {
    using mios::control::RobotMode;
    using mios_ros2_runtime::Ros2CoreRobotBackend;
    assert(Ros2CoreRobotBackend::is_pre_run_robot_mode_permitted(RobotMode::kIdle, false));
    assert(!Ros2CoreRobotBackend::is_pre_run_robot_mode_permitted(RobotMode::kMove, false));
    assert(Ros2CoreRobotBackend::is_pre_run_robot_mode_permitted(RobotMode::kMove, true));
    assert(!Ros2CoreRobotBackend::is_pre_run_robot_mode_permitted(RobotMode::kGuiding, true));

    auto node = std::make_shared<rclcpp::Node>("mios_core_backend_disabled_test");
    mios_ros2_runtime::Ros2RobotBackend robot_backend(
        *node, "/test/robot_state", "/test/robot_model", "/test/effort", "/test/actuator",
        "/test/joint_velocity", "/test/cartesian_velocity", "/test/joint_position",
        "/test/cartesian_pose");
    mios_ros2_runtime::Ros2ArmCommandDispatcher command_dispatcher(robot_backend);
    mios_ros2_runtime::Ros2GripperClient gripper_client(
        *node, "/test/grasp", "/test/move", "/test/home", "/test/stop", "/test/joint_states",
        0.08);
    mios_ros2_runtime::Ros2CoreRobotBackend core_backend(
        robot_backend, command_dispatcher, gripper_client, std::chrono::milliseconds(100),
        std::chrono::milliseconds(100), false);

    bool callback_was_called = false;
    const auto result = core_backend.control(
        mios::control::CommandMode::kTorque,
        [&callback_was_called](const mios::control::RobotState&, const mios::control::RobotModel&,
                               const mios::control::GripperState&, double) {
          callback_was_called = true;
          return mios::control::ArmCommand{};
        });
    assert(result.exception);
    assert(result.error == "TaskExecutionDisabled");
    assert(!callback_was_called);
    assert(!core_backend.grasp(0.02, 0.1, 5.0, 0.001, 0.001));
    assert(!core_backend.set_robot_parameters());

    mios_ros2_runtime::Ros2RobotParameterClient parameter_client(
        *node, "/test/parameter_service", true);
    int disabled_provider_calls = 0;
    mios_ros2_runtime::Ros2CoreRobotBackend disabled_parameter_backend(
        robot_backend, command_dispatcher, gripper_client, &parameter_client,
        [&disabled_provider_calls]() -> std::optional<mios::control::RobotParameters> {
          ++disabled_provider_calls;
          return valid_robot_parameters();
        },
        std::chrono::milliseconds(100), std::chrono::milliseconds(100),
        std::chrono::milliseconds(100), false);
    assert(!disabled_parameter_backend.set_robot_parameters());
    assert(disabled_provider_calls == 0);

    mios_ros2_runtime::Ros2RobotParameterClient disabled_parameter_client(
        *node, "/test/parameter_service", false);
    int no_op_provider_calls = 0;
    mios_ros2_runtime::Ros2CoreRobotBackend no_op_parameter_backend(
        robot_backend, command_dispatcher, gripper_client, &disabled_parameter_client,
        [&no_op_provider_calls]() -> std::optional<mios::control::RobotParameters> {
          ++no_op_provider_calls;
          return valid_robot_parameters();
        },
        std::chrono::milliseconds(100), std::chrono::milliseconds(100),
        std::chrono::milliseconds(100), true);
    // Disabled ROS parameter application is intentional during an FCI-owned
    // task; no service request or parameter snapshot must be produced.
    assert(no_op_parameter_backend.set_robot_parameters());
    assert(no_op_provider_calls == 0);

    mios_ros2_runtime::Ros2CoreRobotBackend missing_parameter_backend(
        robot_backend, command_dispatcher, gripper_client, &parameter_client,
        []() -> std::optional<mios::control::RobotParameters> { return std::nullopt; },
        std::chrono::milliseconds(100), std::chrono::milliseconds(100),
        std::chrono::milliseconds(100), true);
    assert(!missing_parameter_backend.set_robot_parameters());

    int provider_calls = 0;
    mios_ros2_runtime::Ros2CoreRobotBackend unavailable_service_backend(
        robot_backend, command_dispatcher, gripper_client, &parameter_client,
        {}, std::chrono::milliseconds(100), std::chrono::milliseconds(100),
        std::chrono::milliseconds(100), true);
    unavailable_service_backend.set_robot_parameter_provider(
        [&provider_calls]() -> std::optional<mios::control::RobotParameters> {
          ++provider_calls;
          return valid_robot_parameters();
        });
    assert(!unavailable_service_backend.set_robot_parameters());
    assert(provider_calls == 1);
  }
  rclcpp::shutdown();
  return 0;
}
