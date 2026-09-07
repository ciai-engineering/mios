#include <cassert>
#include <cmath>
#include <limits>

#include "mios_ros2_control/mios_control_core.hpp"

namespace {

bool almost_equal(double left, double right) { return std::abs(left - right) < 1.0e-12; }

}  // namespace

int main() {
  using mios_ros2_control::MiosControlCore;
  using mios_ros2_control::MiosCartesianImpedanceRequest;
  using mios_ros2_control::MiosJointImpedanceRequest;
  using mios_ros2_control::MiosRobotModel;
  using mios_ros2_control::MiosRobotState;
  using mios_ros2_control::MiosTorqueRequest;

  MiosControlCore core;
  const MiosControlCore::Effort effort_limit = {20.0, 20.0, 20.0, 20.0,
                                                 20.0, 20.0, 20.0};
  core.configure(effort_limit, 1000.0, 100000000);

  MiosRobotState state;
  MiosRobotModel model;
  MiosTorqueRequest request;
  request.received_nanoseconds = 1000;
  model.body_jacobian[0] = 1.0;
  model.zero_jacobian[0] = 1.0;

  // The typed actuator path evaluates impedance from live state in the
  // control thread, before the same safety/rate-limit path as raw torque.
  MiosJointImpedanceRequest impedance;
  impedance.position[0] = 0.5;
  impedance.velocity[0] = 0.25;
  impedance.feedforward_effort[0] = 1.0;
  impedance.stiffness[0] = 4.0;
  impedance.damping[0] = 2.0;
  MiosControlCore::Effort impedance_effort{};
  assert(core.evaluate_joint_impedance(state, impedance, impedance_effort));
  assert(almost_equal(impedance_effort[0], 3.5));
  impedance.stiffness[0] = -1.0;
  assert(!core.evaluate_joint_impedance(state, impedance, impedance_effort));
  impedance.stiffness[0] = 4.0;

  // Cartesian impedance is evaluated in the base frame with the zero
  // Jacobian. It validates homogeneous transforms and can add the model
  // Coriolis term, matching the legacy Cartesian-joint torque composition.
  state.base_to_end_effector = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  model.zero_jacobian.fill(0.0);
  model.zero_jacobian[0] = 1.0;
  model.coriolis[0] = 0.25;
  MiosCartesianImpedanceRequest cartesian;
  cartesian.target_base_to_end_effector = state.base_to_end_effector;
  cartesian.target_base_to_end_effector[12] = 0.5;
  cartesian.feedforward_wrench[0] = 1.0;
  cartesian.stiffness[0] = 4.0;
  cartesian.use_coriolis_compensation = true;
  MiosControlCore::Effort cartesian_effort{};
  assert(core.evaluate_cartesian_impedance(state, model, cartesian, 0.01, cartesian_effort));
  assert(almost_equal(cartesian_effort[0], 3.25));
  cartesian.target_base_to_end_effector[0] = 1.1;
  assert(!core.evaluate_cartesian_impedance(state, model, cartesian, 0.01, cartesian_effort));

  // Contact-aware stiffness adaptation is bounded and slew limited before the
  // Cartesian impedance wrench is projected through the zero Jacobian.
  cartesian.target_base_to_end_effector[0] = 1.0;
  cartesian.feedforward_wrench[0] = 0.0;
  cartesian.stiffness[0] = 1.0;
  cartesian.use_coriolis_compensation = false;
  state.external_wrench_base.fill(0.0);
  state.external_wrench_base[0] = 1.0;
  core.configure_cartesian_impedance_adaptation(
      {2.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {10.0, 10.0, 10.0, 10.0, 10.0, 10.0}, {100.0, 1.0, 1.0, 1.0, 1.0, 1.0}, true);
  assert(core.evaluate_cartesian_impedance(state, model, cartesian, 0.01, cartesian_effort));
  assert(almost_equal(cartesian_effort[0], 1.0));

  // The desired-wrench path is a bounded, rate-limited PI stage driven by
  // the external wrench state and mapped by the zero Jacobian.
  core.configure_cartesian_force({1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                 {2.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                 {100.0, 1.0, 1.0, 1.0, 1.0, 1.0}, true);
  mios_ros2_control::MiosCartesianForceRequest force;
  force.target_external_wrench[0] = 3.0;
  state.external_wrench_base.fill(0.0);
  MiosControlCore::Effort force_effort{};
  assert(core.evaluate_cartesian_force(state, model, force, 0.01, force_effort));
  assert(almost_equal(force_effort[0], 1.0));
  assert(core.evaluate_cartesian_force(state, model, force, 0.01, force_effort));
  assert(almost_equal(force_effort[0], 2.0));
  core.reset_cartesian_force();
  force.target_external_wrench[0] = std::numeric_limits<double>::quiet_NaN();
  assert(!core.evaluate_cartesian_force(state, model, force, 0.01, force_effort));

  // A damped Jacobian projection removes the Cartesian task component from a
  // joint impedance torque while retaining a null-space component.
  core.configure_nullspace(0.01, {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0}, true);
  model.zero_jacobian.fill(0.0);
  model.zero_jacobian[0] = 1.0;
  state.position.fill(0.0);
  state.velocity.fill(0.0);
  mios_ros2_control::MiosNullspaceRequest nullspace;
  nullspace.position[0] = 1.0;
  nullspace.position[1] = 1.0;
  nullspace.stiffness[0] = 1.0;
  nullspace.stiffness[1] = 1.0;
  MiosControlCore::Effort nullspace_effort{};
  assert(core.evaluate_nullspace(state, model, nullspace, nullspace_effort));
  assert(std::abs(nullspace_effort[0]) < 1.0e-3);
  assert(almost_equal(nullspace_effort[1], 1.0));

  // The ported Cartesian damping is J^T F, with an explicit per-joint cap.
  state.velocity[0] = 2.0;
  core.configure_cartesian_velocity_damping({0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                             {1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                             {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0}, true);
  const auto damped = core.step(state, model, &request, 1000, 0.01);
  assert(almost_equal(damped[0], -2.0));

  // A user stop bypasses the rate limiter and clears torque in the same cycle.
  request.user_stopped = true;
  request.received_nanoseconds = 1010;
  const auto stopped = core.step(state, model, &request, 1010, 0.01);
  assert(almost_equal(stopped[0], 0.0));
  request.user_stopped = false;

  // The virtual workspace arms while inside, then pushes back through the
  // base/zero Jacobian once the tool crosses a wall.
  core.reset();
  state.velocity[0] = 0.0;
  core.configure_cartesian_velocity_damping({0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, false);
  core.configure_cartesian_workspace({-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0},
                                     {4.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
                                     {3.0, 3.0, 3.0}, true);
  state.base_to_end_effector[12] = 0.0;
  request.received_nanoseconds = 1020;
  assert(almost_equal(core.step(state, model, &request, 1020, 0.01)[0], 0.0));
  state.base_to_end_effector[12] = 1.5;
  request.received_nanoseconds = 1030;
  const auto wall_torque = core.step(state, model, &request, 1030, 0.01);
  assert(almost_equal(wall_torque[0], -2.0));

  // Invalid model data required by an enabled safety term is fail-safe rather
  // than rate-limited.
  model.zero_jacobian[0] = std::numeric_limits<double>::quiet_NaN();
  request.received_nanoseconds = 1040;
  assert(almost_equal(core.step(state, model, &request, 1040, 0.01)[0], 0.0));

  // Position hold remains active when no external command is fresh. It is
  // bounded before the global torque limit and uses damping at the captured
  // reference pose.
  core.reset();
  model.zero_jacobian[0] = 1.0;
  core.configure_cartesian_workspace({-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0},
                                     {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
                                     {0.0, 0.0, 0.0}, false);
  core.configure_position_hold({4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                               {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                               {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5}, true);
  state.position.fill(0.0);
  state.velocity.fill(0.0);
  state.position[0] = 0.25;
  state.velocity[0] = 0.1;
  MiosControlCore::Effort reference{};
  core.capture_position_hold_reference(reference);
  request.received_nanoseconds = 1;
  const auto held = core.step(state, model, &request, 1000000000, 0.01);
  assert(almost_equal(held[0], -0.5));

  // The production commissioning baseline keeps the local hold disabled.
  // With no fresh external request it must remain an exact zero external
  // torque command, matching Franka's gravity-compensation example rather
  // than silently applying a MIOS spring/damper torque.
  core.reset();
  core.configure_position_hold({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                               {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                               {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, false);
  state.user_stopped = false;
  state.position[0] = 0.25;
  state.velocity[0] = 0.1;
  const auto zero_effort = core.step(state, model, nullptr, 1000000000, 0.01);
  for (const double effort : zero_effort) {
    assert(almost_equal(effort, 0.0));
  }

  // A user stop overrides the local hold immediately.
  state.user_stopped = true;
  assert(almost_equal(core.step(state, model, nullptr, 1000000010, 0.01)[0], 0.0));
}
