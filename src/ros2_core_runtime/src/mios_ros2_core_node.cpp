#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "mios/core/core.hpp"
#include "mios/utils/configuration.hpp"
#include "mios/utils/context.hpp"
#include "mios_ros2_runtime/ros2_arm_command_dispatcher.hpp"
#include "mios_ros2_runtime/ros2_core_robot_backend.hpp"
#include "mios_ros2_runtime/ros2_gripper_client.hpp"
#include "mios_ros2_runtime/ros2_robot_parameter_client.hpp"
#include "mios_ros2_runtime/ros2_robot_backend.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

std::chrono::milliseconds seconds_parameter(rclcpp::Node& node, const std::string& name,
                                            const double default_value) {
  const auto seconds = node.declare_parameter<double>(name, default_value);
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(seconds));
}

unsigned unsigned_parameter(rclcpp::Node& node, const std::string& name,
                            const std::int64_t default_value) {
  const auto value = node.declare_parameter<std::int64_t>(name, default_value);
  if (value < 0 || static_cast<std::uint64_t>(value) > std::numeric_limits<unsigned>::max()) {
    throw std::invalid_argument(name + " must be a non-negative unsigned integer");
  }
  return static_cast<unsigned>(value);
}

}  // namespace

namespace mios_ros2_runtime {

// This executable owns the MIOS Core and gives it exactly one backend: the
// ROS-only adapter. It never creates PandaBody or an FCI/libfranka client.
class MiosRos2CoreNode final : public rclcpp::Node {
 public:
  MiosRos2CoreNode() : Node("mios_ros2_core") {
    configuration_.verbosity = declare_parameter<std::string>("mios_verbosity", "info");
    configuration_.robot_ip = declare_parameter<std::string>("robot_ip", "");
    configuration_.database_name = declare_parameter<std::string>("database_name", "mios");
    configuration_.robot_configuration = unsigned_parameter(*this, "robot_configuration", 1);
    configuration_.database_port = unsigned_parameter(*this, "database_port", 27017);
    configuration_.websocket_port = unsigned_parameter(*this, "websocket_port", 12000);
    configuration_.udp_port = unsigned_parameter(*this, "udp_port", 12002);
    configuration_.rpc_port = unsigned_parameter(*this, "rpc_port", 12001);
    configuration_.use_desk = declare_parameter<bool>("use_desk", false);

    const auto robot_state_topic = declare_parameter<std::string>(
        "robot_state_topic", "/franka_robot_state_broadcaster/robot_state");
    const auto robot_model_topic = declare_parameter<std::string>(
        "robot_model_topic", "/mios_robot_model_broadcaster/robot_model");
    const auto effort_command_topic = declare_parameter<std::string>(
        "effort_command_topic", "/mios_effort_controller/mios_effort_command");
    const auto actuator_command_topic = declare_parameter<std::string>(
        "actuator_command_topic", "/mios_effort_controller/mios_actuator_command");
    const auto joint_velocity_command_topic = declare_parameter<std::string>(
        "joint_velocity_command_topic", "/mios_joint_velocity_controller/mios_actuator_command");
    const auto cartesian_velocity_command_topic = declare_parameter<std::string>(
        "cartesian_velocity_command_topic", "/mios_cartesian_velocity_controller/mios_actuator_command");
    const auto joint_position_command_topic = declare_parameter<std::string>(
        "joint_position_command_topic", "/mios_joint_position_controller/mios_actuator_command");
    const auto cartesian_pose_command_topic = declare_parameter<std::string>(
        "cartesian_pose_command_topic", "/mios_cartesian_pose_controller/mios_actuator_command");
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
    const auto parameter_service_namespace =
        declare_parameter<std::string>("parameter_service_namespace", "/service_server");
    const auto gripper_max_width = declare_parameter<double>("gripper_max_width", 0.08);
    const auto allow_robot_parameter_application =
        declare_parameter<bool>("allow_robot_parameter_application", false);
    const auto enable_core_task_execution =
        declare_parameter<bool>("enable_core_task_execution", false);
    const auto enable_core_scheduler =
        declare_parameter<bool>("enable_core_scheduler", false);
    const auto allow_controller_owned_move_mode =
        declare_parameter<bool>("allow_controller_owned_move_mode", false);
    // A scheduler-only diagnostic can run while ros2_control owns a command
    // interface, but it must be requested separately from physical Core task
    // execution. The ROS backend still rejects every control and gripper
    // request while enable_core_task_execution is false.
    const auto allow_controller_owned_move_mode_without_task_execution =
        declare_parameter<bool>("allow_controller_owned_move_mode_without_task_execution", false);
    configuration_.defer_database_connection = !enable_core_scheduler;

    core_state_timeout_ = seconds_parameter(*this, "core_state_timeout_seconds", 0.1);
    // A full 80 mm gripper stroke at the teaching default of 5 mm/s takes
    // 16 seconds before contact/force evaluation.  Keep the Core wait longer
    // than that so it never starts a second gripper action while the first is
    // still active.
    const auto core_gripper_timeout = seconds_parameter(*this, "core_gripper_timeout_seconds", 20.0);
    const auto core_parameter_timeout =
        seconds_parameter(*this, "core_parameter_timeout_seconds", 5.0);

    backend_ = std::make_unique<Ros2RobotBackend>(
        *this, robot_state_topic, robot_model_topic, effort_command_topic, actuator_command_topic,
        joint_velocity_command_topic, cartesian_velocity_command_topic, joint_position_command_topic,
        cartesian_pose_command_topic);
    command_dispatcher_ = std::make_unique<Ros2ArmCommandDispatcher>(*backend_);
    gripper_client_ = std::make_unique<Ros2GripperClient>(
        *this, gripper_grasp_action, gripper_move_action, gripper_homing_action,
        gripper_stop_service, gripper_state_topic, gripper_max_width);
    parameter_client_ = std::make_unique<Ros2RobotParameterClient>(
        *this, parameter_service_namespace, allow_robot_parameter_application);

    // Core task scheduling is an explicit second gate. A future operator must
    // enable both switches before any Core task can dispatch robot commands.
    const bool allow_task_execution = enable_core_scheduler && enable_core_task_execution;
    // ros2_control puts Franka in MOVE while a controller owns an interface,
    // including its zero-command hold. This third gate retains the legacy
    // IDLE-only behavior until that ownership model is commissioned. A
    // separate fourth gate permits scheduler-only diagnostics in that state;
    // it does not enable command dispatch.
    const bool scheduler_only_move_diagnostic_enabled =
        enable_core_scheduler && !enable_core_task_execution &&
        allow_controller_owned_move_mode_without_task_execution;
    const bool controller_owned_move_mode_enabled =
        allow_controller_owned_move_mode &&
        (allow_task_execution || scheduler_only_move_diagnostic_enabled);
    configuration_.allow_controller_owned_move_mode = controller_owned_move_mode_enabled;
    auto core_backend = std::make_unique<Ros2CoreRobotBackend>(
        *backend_, *command_dispatcher_, *gripper_client_, parameter_client_.get(),
        Ros2CoreRobotBackend::ParameterSnapshotProvider{}, core_state_timeout_,
        core_gripper_timeout, core_parameter_timeout, allow_task_execution,
        controller_owned_move_mode_enabled);

    context_ = std::make_unique<MiosContext>(MiosContext{configuration_, shutdown_signal_});
    core_ = std::make_unique<mios::Core>(*context_, std::move(core_backend));

    RCLCPP_INFO(
        get_logger(),
        "MIOS Core is owned by the ROS 2 runtime with no PandaBody, libfranka, or FCI ownership. "
        "Core scheduler: %s; Core task commands: %s; controller-owned MOVE mode: %s; "
        "no-task MOVE diagnostic: %s; parameter application: %s",
        enable_core_scheduler ? "enabled" : "disabled",
        allow_task_execution ? "enabled" : "disabled",
        controller_owned_move_mode_enabled ? "enabled" : "disabled",
        scheduler_only_move_diagnostic_enabled ? "enabled" : "disabled",
        allow_robot_parameter_application ? "enabled" : "disabled");

    if (!enable_core_scheduler) {
      RCLCPP_INFO(get_logger(),
                  "Core construction test complete. The scheduler is disabled, so no database, "
                  "portal, gripper, or arm command is started.");
      return;
    }

    // Core needs fresh ROS state/model snapshots. Run initialization only
    // after this node has entered its executor and subscriptions can receive
    // their first messages.
    core_start_timer_ = create_wall_timer(
        std::chrono::milliseconds(100), [this] { start_core_scheduler(); });
  }

  ~MiosRos2CoreNode() override {
    shutdown_signal_.store(true);
    if (core_) {
      core_->terminate();
    }
    if (core_thread_.joinable()) {
      core_thread_.join();
    }
  }

 private:
  void start_core_scheduler() {
    // Do not call Core::initialize() until both subscriptions have received
    // current data. DDS discovery can take longer than the initial timer
    // period, particularly when this node starts after the broadcasters.
    // Retrying here is read-only and keeps all command gates unchanged.
    if (!backend_->has_fresh_robot_state(core_state_timeout_) ||
        !backend_->has_fresh_robot_model(core_state_timeout_)) {
      if (!waiting_for_initial_snapshots_logged_) {
        RCLCPP_WARN(get_logger(),
                    "Waiting for fresh robot-state and robot-model messages before starting "
                    "the MIOS Core scheduler.");
        waiting_for_initial_snapshots_logged_ = true;
      }
      return;
    }

    core_start_timer_->cancel();
    // Initializing Core opens its existing database/portal services. This is
    // deliberately opt-in and still cannot command hardware unless the second
    // task-execution gate above is also enabled.
    if (!core_->initialize()) {
      RCLCPP_ERROR(get_logger(), "MIOS Core initialization failed; task scheduler was not started.");
      return;
    }
    core_thread_ = std::thread([this] { core_->start(); });
  }

  std::atomic<bool> shutdown_signal_{false};
  mios::MiosConfiguration configuration_{};
  std::unique_ptr<MiosContext> context_;
  std::unique_ptr<Ros2RobotBackend> backend_;
  std::unique_ptr<Ros2ArmCommandDispatcher> command_dispatcher_;
  std::unique_ptr<Ros2GripperClient> gripper_client_;
  std::unique_ptr<Ros2RobotParameterClient> parameter_client_;
  std::unique_ptr<mios::Core> core_;
  std::thread core_thread_;
  rclcpp::TimerBase::SharedPtr core_start_timer_;
  std::chrono::milliseconds core_state_timeout_{100};
  bool waiting_for_initial_snapshots_logged_{false};
};

}  // namespace mios_ros2_runtime

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mios_ros2_runtime::MiosRos2CoreNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
