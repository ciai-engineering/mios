#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>

#include "mios/panda/robot_backend.hpp"
#include "mios_ros2_runtime/ros2_arm_command_dispatcher.hpp"
#include "mios_ros2_runtime/ros2_gripper_client.hpp"
#include "mios_ros2_runtime/ros2_robot_parameter_client.hpp"
#include "mios_ros2_runtime/ros2_robot_backend.hpp"

namespace mios_ros2_runtime {

// Adapter from the original MIOS Core contract to the ROS-only transport.
// It owns neither libfranka nor an FCI connection. Its callback runs from a
// non-real-time robot-state subscription, never from ros2_control::update().
class Ros2CoreRobotBackend final : public mios::RobotBackend {
 public:
  using ParameterSnapshotProvider = mios::RobotBackend::RobotParameterProvider;

  // Compatibility constructor. It intentionally has no parameter provider,
  // therefore set_robot_parameters() remains fail-closed.
  Ros2CoreRobotBackend(Ros2RobotBackend& robot_backend,
                       Ros2ArmCommandDispatcher& command_dispatcher,
                       Ros2GripperClient& gripper_client,
                       std::chrono::milliseconds state_timeout,
                       std::chrono::milliseconds gripper_timeout,
                       bool allow_task_execution);
  // Full ROS-only Core bridge. The provider is called outside the controller
  // update loop and supplies an immutable, already-owned parameter snapshot.
  Ros2CoreRobotBackend(Ros2RobotBackend& robot_backend,
                       Ros2ArmCommandDispatcher& command_dispatcher,
                       Ros2GripperClient& gripper_client,
                       Ros2RobotParameterClient* parameter_client,
                       ParameterSnapshotProvider parameter_snapshot_provider,
                       std::chrono::milliseconds state_timeout,
                       std::chrono::milliseconds gripper_timeout,
                       std::chrono::milliseconds parameter_timeout,
                       bool allow_task_execution,
                       bool allow_controller_owned_move_mode = false);
  ~Ros2CoreRobotBackend() override;

  bool initialize() override;
  bool connect_to_robot(const std::optional<std::string>& ip) override;
  bool connect_to_gripper(const std::optional<std::string>& ip) override;
  void disconnect_from_robot() override;
  void disconnect_from_gripper() override;
  mios::ControlReturnType recover() override;
  bool pre_run_checks() const override;
  mios::ControlReturnType control(mios::control::CommandMode mode,
                                  ControlCallback callback) override;
  void set_robot_parameter_provider(ParameterSnapshotProvider provider) override;
  bool set_robot_parameters() override;
  bool get_robot_snapshot(mios::control::RobotState& robot_state,
                          mios::control::RobotModel& robot_model,
                          mios::control::GripperState& gripper_state) const override;
  bool grasp(double width, double speed, double force, double epsilon_inner,
             double epsilon_outer) const override;
  bool move_to_finger_position(double width, double speed) const override;
  bool home_gripper() const override;
  bool unlock_brakes() override;
  bool lock_brakes() override;
  bool shutdown_robot() override;
  bool reboot_robot() override;

  // The controller manager enters Franka MOVE mode when it owns a command
  // interface. This is only acceptable after explicit commissioning.
  static bool is_pre_run_robot_mode_permitted(mios::control::RobotMode mode,
                                               bool allow_controller_owned_move_mode);

  // A caller may end a currently running Core control callback. It emits a
  // completed command for the active mode before returning from control().
  void stop_active_control();

 private:
  struct ActiveControl {
    std::uint64_t generation{0};
    mios::control::CommandMode mode{mios::control::CommandMode::kTorque};
    ControlCallback callback;
    std::int64_t previous_stamp_nanoseconds{0};
    bool complete{false};
    mios::ControlReturnType result{false, "None", ""};
  };

  void receive_state(const RobotSnapshot& snapshot);
  bool wait_for_gripper_operation(
      const std::function<bool(Ros2GripperClient::CompletionCallback)>& start_operation) const;
  bool wait_for_parameter_operation(const RobotParameterSnapshot& snapshot) const;
  bool state_and_model_are_fresh() const;
  void complete_active_control(std::uint64_t generation, mios::ControlReturnType result);

  Ros2RobotBackend& robot_backend_;
  Ros2ArmCommandDispatcher& command_dispatcher_;
  Ros2GripperClient& gripper_client_;
  Ros2RobotParameterClient* parameter_client_;
  mutable std::mutex parameter_provider_mutex_;
  ParameterSnapshotProvider parameter_snapshot_provider_;
  const std::chrono::milliseconds state_timeout_;
  const std::chrono::milliseconds gripper_timeout_;
  const std::chrono::milliseconds parameter_timeout_;
  const bool allow_task_execution_;
  const bool allow_controller_owned_move_mode_;
  const std::size_t state_observer_id_;

  mutable std::mutex control_mutex_;
  mutable std::mutex dispatch_mutex_;
  std::condition_variable control_complete_;
  std::optional<ActiveControl> active_control_;
  std::uint64_t next_control_generation_{1};
};

}  // namespace mios_ros2_runtime
