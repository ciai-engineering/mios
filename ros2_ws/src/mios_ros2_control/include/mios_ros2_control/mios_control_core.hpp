#pragma once

#include <array>
#include <cstdint>

namespace mios_ros2_control {

// The transport-neutral data consumed by the MIOS real-time control core.
// It deliberately contains no ROS handles, database objects, or libfranka
// objects, so `step()` is suitable for controller_manager's update thread.
struct MiosRobotState {
  std::array<double, 7> position{};
  std::array<double, 7> velocity{};
  std::array<double, 7> effort{};
  std::array<double, 6> external_wrench_base{};
  // Homogeneous base-to-end-effector pose in column-major order, matching
  // libfranka's O_T_EE and the zero Jacobian frame.
  std::array<double, 16> base_to_end_effector{};
  bool user_stopped{false};
};

struct MiosTorqueRequest {
  std::array<double, 7> effort{};
  std::int64_t received_nanoseconds{0};
  bool user_stopped{false};
};

// The transport-neutral subset of MIOS' Actuator used by the legacy
// joint-torque pipeline. It is evaluated against live state in the
// controller-manager update loop, not converted to torque in a ROS callback.
struct MiosJointImpedanceRequest {
  std::array<double, 7> position{};
  std::array<double, 7> velocity{};
  std::array<double, 7> feedforward_effort{};
  std::array<double, 7> stiffness{};
  std::array<double, 7> damping{};
  std::int64_t received_nanoseconds{0};
  bool user_stopped{false};
};

// Base-frame Cartesian actuator snapshot. The target pose and Jacobian share
// the column-major convention used by Franka's O_T_EE and zero Jacobian.
struct MiosCartesianImpedanceRequest {
  std::array<double, 16> target_base_to_end_effector{};
  std::array<double, 6> target_velocity{};
  std::array<double, 6> feedforward_wrench{};
  std::array<double, 6> stiffness{};
  std::array<double, 6> damping{};
  std::int64_t received_nanoseconds{0};
  bool use_coriolis_compensation{false};
  bool user_stopped{false};
};

struct MiosCartesianForceRequest {
  std::array<double, 6> target_external_wrench{};
  std::int64_t received_nanoseconds{0};
  bool user_stopped{false};
};

struct MiosNullspaceRequest {
  std::array<double, 7> position{};
  std::array<double, 7> stiffness{};
  std::array<double, 7> damping{};
  std::int64_t received_nanoseconds{0};
  bool user_stopped{false};
};

// Model values are produced by franka_hardware through Franka's semantic
// interfaces. They remain separate from transport data so the control core can
// reject incomplete/invalid dynamics before executing a MIOS algorithm.
struct MiosRobotModel {
  std::array<double, 49> mass{};
  std::array<double, 7> coriolis{};
  std::array<double, 7> gravity{};
  std::array<double, 42> body_jacobian{};
  std::array<double, 42> zero_jacobian{};
};

class MiosControlCore {
 public:
  using Effort = std::array<double, 7>;
  using Cartesian = std::array<double, 6>;
  using Position = std::array<double, 3>;

  void configure(const Effort& effort_limits, double effort_rate_limit,
                 std::int64_t command_timeout_nanoseconds);
  void configure_joint_safety(const Effort& lower_limits, const Effort& upper_limits,
                              const Effort& stiffness, const Effort& damping,
                              const Effort& max_torque, bool enabled);
  // The local hold is the mandatory no-motion torque-mode baseline. Its
  // reference is captured from the measured joints immediately before an
  // effort interface is activated; external requests are added on top only
  // after separate commissioning gates permit them.
  void configure_position_hold(const Effort& stiffness, const Effort& damping,
                               const Effort& max_torque, bool enabled);
  const Effort& position_hold_reference() const { return hold_reference_; }
  bool position_hold_reference_valid() const { return position_hold_reference_valid_; }
  void capture_position_hold_reference(const Effort& position);
  // Cartesian damping mirrors MIOS' legacy velocity-damping term. It uses
  // the end-effector/body Jacobian and is bounded per joint before it is
  // combined with the requested torque.
  void configure_cartesian_velocity_damping(const Cartesian& threshold,
                                            const Cartesian& damping,
                                            const Effort& max_torque, bool enabled);
  // The virtual workspace is armed only after the tool is initially observed
  // inside the configured box, matching the legacy virtual-cube activation
  // behavior. It operates in the base frame through the zero Jacobian.
  void configure_cartesian_workspace(const Position& lower_limits,
                                     const Position& upper_limits,
                                     const Position& stiffness,
                                     const Position& damping,
                                     const Position& max_force, bool enabled);
  void reset();

  // Computes tau = tau_ff + K(q_d - q) + D(dq_d - dq). Invalid high-level
  // input is rejected rather than partially evaluated.
  [[nodiscard]] bool evaluate_joint_impedance(
      const MiosRobotState& state, const MiosJointImpedanceRequest& request,
      Effort& effort) const;

  // Evaluates a base-frame Cartesian impedance wrench against live pose and
  // twist, maps it through the Franka zero Jacobian, and optionally adds the
  // model Coriolis vector used by the original Cartesian-joint pipeline.
  [[nodiscard]] bool evaluate_cartesian_impedance(
      const MiosRobotState& state, const MiosRobotModel& model,
      const MiosCartesianImpedanceRequest& request, double period_seconds,
      Effort& effort);
  void configure_cartesian_impedance_adaptation(
      const Cartesian& contact_stiffness_gain, const Cartesian& stiffness_lower_limit,
      const Cartesian& stiffness_upper_limit, const Cartesian& stiffness_rate_limit,
      bool enabled);
  void reset_cartesian_impedance_adaptation();

  void configure_cartesian_force(const Cartesian& proportional_gain,
                                 const Cartesian& integral_gain,
                                 const Cartesian& integral_wrench_limit,
                                 const Cartesian& derivative_gain,
                                 const Cartesian& derivative_filter_time_constant,
                                 const Cartesian& derivative_wrench_limit,
                                 const Cartesian& output_wrench_limit,
                                 const Cartesian& output_wrench_rate_limit,
                                 bool enabled);
  void reset_cartesian_force();
  [[nodiscard]] bool evaluate_cartesian_force(
      const MiosRobotState& state, const MiosRobotModel& model,
      const MiosCartesianForceRequest& request, double period_seconds,
      Effort& effort);
  void configure_nullspace(double singularity_damping, const Effort& effort_limit,
                           bool enabled);
  [[nodiscard]] bool evaluate_nullspace(const MiosRobotState& state,
                                        const MiosRobotModel& model,
                                        const MiosNullspaceRequest& request,
                                        Effort& effort) const;

  // Returns a finite, bounded and rate-limited torque command. Invalid input
  // or a user-stop state clears the applied command immediately. A stale
  // external request is ignored, while the local position hold remains active.
  [[nodiscard]] Effort step(const MiosRobotState& state, const MiosRobotModel& model,
                             const MiosTorqueRequest* request, std::int64_t now_nanoseconds,
                             double period_seconds);

 private:
  Effort effort_limits_{};
  Effort applied_effort_{};
  double effort_rate_limit_{0.0};
  std::int64_t command_timeout_nanoseconds_{0};
  Effort lower_joint_limits_{};
  Effort upper_joint_limits_{};
  Effort wall_stiffness_{};
  Effort wall_damping_{};
  Effort wall_max_torque_{};
  bool joint_safety_enabled_{false};
  Effort hold_stiffness_{};
  Effort hold_damping_{};
  Effort hold_max_torque_{};
  Effort hold_reference_{};
  bool position_hold_enabled_{false};
  bool position_hold_reference_valid_{false};
  Cartesian cartesian_velocity_threshold_{};
  Cartesian cartesian_velocity_damping_{};
  Effort cartesian_damping_max_torque_{};
  bool cartesian_velocity_damping_enabled_{false};
  Position workspace_lower_limits_{};
  Position workspace_upper_limits_{};
  Position workspace_stiffness_{};
  Position workspace_damping_{};
  Position workspace_max_force_{};
  bool cartesian_workspace_enabled_{false};
  bool workspace_safely_armed_{false};
  Cartesian force_proportional_gain_{};
  Cartesian force_integral_gain_{};
  Cartesian force_integral_wrench_limit_{};
  Cartesian force_derivative_gain_{};
  Cartesian force_derivative_filter_time_constant_{};
  Cartesian force_derivative_wrench_limit_{};
  Cartesian force_output_wrench_limit_{};
  Cartesian force_output_wrench_rate_limit_{};
  Cartesian force_integral_wrench_{};
  Cartesian force_previous_error_{};
  Cartesian force_filtered_error_rate_{};
  Cartesian force_applied_wrench_{};
  bool cartesian_force_enabled_{false};
  Cartesian adaptation_contact_stiffness_gain_{};
  Cartesian adaptation_stiffness_lower_limit_{};
  Cartesian adaptation_stiffness_upper_limit_{};
  Cartesian adaptation_stiffness_rate_limit_{};
  Cartesian adapted_stiffness_{};
  bool adapted_stiffness_valid_{false};
  bool cartesian_impedance_adaptation_enabled_{false};
  double nullspace_singularity_damping_{0.0};
  Effort nullspace_effort_limit_{};
  bool nullspace_enabled_{false};
};

}  // namespace mios_ros2_control
