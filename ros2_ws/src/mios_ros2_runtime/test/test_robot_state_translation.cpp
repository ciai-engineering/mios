#include <array>
#include <cassert>

#include "mios_ros2_runtime/robot_state_translation.hpp"

namespace {

using mios::control::RobotMode;
using mios_ros2_runtime::RobotRuntimeMode;
using mios_ros2_runtime::RobotSnapshot;
using mios_ros2_runtime::to_mios_robot_state;

void expect_mode(const RobotRuntimeMode runtime_mode, const RobotMode mios_mode) {
  RobotSnapshot snapshot;
  snapshot.robot_mode = runtime_mode;
  const auto state = to_mios_robot_state(snapshot);
  assert(state.robot_mode == mios_mode);
}

}  // namespace

int main() {
  RobotSnapshot snapshot;
  snapshot.position[0] = 0.1;
  snapshot.velocity[1] = 0.2;
  snapshot.motor_position[2] = 0.3;
  snapshot.motor_velocity[3] = 0.4;
  snapshot.effort[4] = 0.5;
  snapshot.external_effort[5] = 0.6;
  snapshot.base_to_end_effector[12] = 0.7;
  snapshot.external_wrench_origin[0] = 0.8;
  snapshot.external_wrench_stiffness[1] = 0.9;
  snapshot.robot_mode = RobotRuntimeMode::kMove;

  const auto state = to_mios_robot_state(snapshot);
  assert(state.position[0] == 0.1);
  assert(state.velocity[1] == 0.2);
  assert(state.motor_position[2] == 0.3);
  assert(state.motor_velocity[3] == 0.4);
  assert(state.effort[4] == 0.5);
  assert(state.external_effort[5] == 0.6);
  assert(state.base_to_end_effector[12] == 0.7);
  assert(state.external_wrench_origin[0] == 0.8);
  assert(state.external_wrench_stiffness[1] == 0.9);
  assert(state.robot_mode == RobotMode::kMove);
  assert(!state.user_stopped);

  expect_mode(RobotRuntimeMode::kOther, RobotMode::kOther);
  expect_mode(RobotRuntimeMode::kIdle, RobotMode::kIdle);
  expect_mode(RobotRuntimeMode::kMove, RobotMode::kMove);
  expect_mode(RobotRuntimeMode::kGuiding, RobotMode::kGuiding);
  expect_mode(RobotRuntimeMode::kReflex, RobotMode::kReflex);
  expect_mode(RobotRuntimeMode::kUserStopped, RobotMode::kUserStopped);
  expect_mode(RobotRuntimeMode::kAutomaticErrorRecovery, RobotMode::kAutomaticErrorRecovery);

  snapshot.robot_mode = RobotRuntimeMode::kMove;
  snapshot.user_stopped = true;
  const auto stopped_state = to_mios_robot_state(snapshot);
  assert(stopped_state.user_stopped);
  assert(stopped_state.robot_mode == RobotMode::kUserStopped);
}
