#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "franka/robot_state.h"
#include "franka_msgs/msg/franka_robot_state.hpp"
#include "franka_semantic_components/franka_robot_model.hpp"
#include "mios_msgs/msg/mios_actuator_command.hpp"
#include "mios_msgs/msg/mios_effort_command.hpp"
#include "mios_ros2_control/mios_control_core.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace mios_ros2_control {

// franka_hardware remains the only owner of libfranka and the FCI connection.
// The controller accepts either a commissioning-only raw torque request or a
// typed MIOS actuator snapshot. The latter is evaluated against live state in
// update(), then passes through the same safety/limit path.
class MiosEffortController : public controller_interface::ControllerInterface {
 public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                            const rclcpp::Duration& period) override;

 private:
  using Effort = MiosControlCore::Effort;

  // Written by a normal ROS subscription callback and read through a
  // RealtimeBuffer in update(). The update loop must not construct a complete
  // ROS message just to inspect robot mode.
  struct RobotSafetyState {
    std::int64_t received_nanoseconds{0};
    bool ready_for_control{false};
    Effort position{};
    Effort velocity{};
    MiosControlCore::Cartesian external_wrench_base{};
  };

  void receive_effort(const std_msgs::msg::Float64MultiArray::SharedPtr message);
  void receive_runtime_effort(const mios_msgs::msg::MiosEffortCommand::SharedPtr message);
  void receive_runtime_actuator(const mios_msgs::msg::MiosActuatorCommand::SharedPtr message);
  void receive_robot_state(const franka_msgs::msg::FrankaRobotState::SharedPtr message);
  bool configure_parameters();

  std::vector<std::string> joints_;
  std::string arm_id_{"fr3"};
  Effort effort_limits_{};
  double effort_rate_limit_{20.0};  // Nm/s, intentionally conservative by default.
  double command_timeout_{0.1};
  // Ordinary operation reads the ROS-control hardware's in-process Franka
  // state box.  The topic route exists only for GenericSystem lifecycle tests,
  // which cannot export a real Franka state box.
  std::string robot_state_source_{"hardware"};
  std::string robot_state_topic_{"/franka_robot_state_broadcaster/robot_state"};
  double robot_state_timeout_{0.1};
  bool robot_state_safety_enabled_{true};
  // A transport-free, exact-zero mode used only while commissioning the FCI
  // path. Its configuration rejects every non-zero command/hold option.
  bool zero_effort_baseline_{false};
  double activation_velocity_threshold_{0.01};
  bool allow_effort_activation_{false};
  bool allow_external_effort_commands_{false};
  bool allow_runtime_effort_commands_{false};
  bool allow_runtime_actuator_commands_{false};
  bool allow_runtime_cartesian_actuator_commands_{false};
  bool allow_runtime_cartesian_force_commands_{false};
  Effort joint_lower_limits_{};
  Effort joint_upper_limits_{};
  Effort joint_wall_stiffness_{};
  Effort joint_wall_damping_{};
  Effort joint_wall_max_torque_{};
  bool joint_safety_enabled_{true};
  Effort hold_stiffness_{};
  Effort hold_damping_{};
  Effort hold_max_torque_{};
  // A spring/damper hold is an active torque controller. It must not become
  // the implicit hardware commissioning baseline; the shipped configuration
  // uses the separately gated, exact-zero external-torque mode instead.
  bool position_hold_enabled_{false};
  // Disabled by default. When enabled for a supervised hold test, this is a
  // bounded-rate diagnostic publisher; it is never part of the control path.
  double effort_diagnostic_publish_rate_{0.0};
  double effort_diagnostic_elapsed_seconds_{0.0};
  MiosControlCore::Cartesian cartesian_velocity_threshold_{};
  MiosControlCore::Cartesian cartesian_velocity_damping_{};
  Effort cartesian_damping_max_torque_{};
  bool cartesian_velocity_damping_enabled_{false};
  MiosControlCore::Position workspace_lower_limits_{};
  MiosControlCore::Position workspace_upper_limits_{};
  MiosControlCore::Position workspace_stiffness_{};
  MiosControlCore::Position workspace_damping_{};
  MiosControlCore::Position workspace_max_force_{};
  bool cartesian_workspace_enabled_{false};
  MiosControlCore::Cartesian force_proportional_gain_{};
  MiosControlCore::Cartesian force_integral_gain_{};
  MiosControlCore::Cartesian force_integral_wrench_limit_{};
  MiosControlCore::Cartesian force_derivative_gain_{};
  MiosControlCore::Cartesian force_derivative_filter_time_constant_{};
  MiosControlCore::Cartesian force_derivative_wrench_limit_{};
  MiosControlCore::Cartesian force_output_wrench_limit_{};
  MiosControlCore::Cartesian force_output_wrench_rate_limit_{};
  bool cartesian_force_enabled_{false};
  MiosControlCore::Cartesian adaptation_contact_stiffness_gain_{};
  MiosControlCore::Cartesian adaptation_stiffness_lower_limit_{};
  MiosControlCore::Cartesian adaptation_stiffness_upper_limit_{};
  MiosControlCore::Cartesian adaptation_stiffness_rate_limit_{};
  bool cartesian_impedance_adaptation_enabled_{false};
  double nullspace_singularity_damping_{0.1};
  Effort nullspace_effort_limit_{};
  bool nullspace_enabled_{false};
  bool allow_runtime_nullspace_commands_{false};
  bool requires_franka_model_{false};
  MiosControlCore control_core_;
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_model_;
  realtime_tools::RealtimeThreadSafeBox<franka::RobotState>* robot_state_box_{nullptr};
  MiosRobotModel model_state_{};
  realtime_tools::RealtimeBuffer<std::shared_ptr<MiosTorqueRequest>> desired_effort_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<MiosJointImpedanceRequest>> desired_actuator_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<MiosCartesianImpedanceRequest>>
      desired_cartesian_actuator_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<MiosCartesianForceRequest>>
      desired_cartesian_force_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<MiosNullspaceRequest>> desired_nullspace_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<RobotSafetyState>> robot_safety_state_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr effort_subscription_;
  rclcpp::Subscription<mios_msgs::msg::MiosEffortCommand>::SharedPtr runtime_effort_subscription_;
  rclcpp::Subscription<mios_msgs::msg::MiosActuatorCommand>::SharedPtr runtime_actuator_subscription_;
  rclcpp::Subscription<franka_msgs::msg::FrankaRobotState>::SharedPtr robot_state_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      effort_diagnostic_lifecycle_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>
      effort_diagnostic_publisher_;
};

}  // namespace mios_ros2_control
