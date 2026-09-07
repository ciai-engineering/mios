#include <cassert>
#include <cstdint>

#include "mios_ros2_runtime/arm_command_translation.hpp"

namespace {

using mios::control::ArmCommand;
using mios::control::CommandMode;
using mios_ros2_runtime::ArmCommandDispatchMode;
using mios_ros2_runtime::translate_arm_command;

void expect_mode(const CommandMode command_mode, const ArmCommandDispatchMode dispatch_mode) {
  ArmCommand command;
  command.mode = command_mode;
  command.joints[0] = 1.25;
  command.cartesian[0] = -0.5;
  command.pose[12] = 0.3;

  const auto translated = translate_arm_command(command, false);
  assert(translated.has_value());
  assert(translated->mode == dispatch_mode);
  assert(translated->joints[0] == 1.25);
  assert(translated->cartesian[0] == -0.5);
  assert(translated->pose[12] == 0.3);
  assert(!translated->user_stopped);
}

}  // namespace

int main() {
  expect_mode(CommandMode::kTorque, ArmCommandDispatchMode::kEffort);
  expect_mode(CommandMode::kJointVelocity, ArmCommandDispatchMode::kJointVelocity);
  expect_mode(CommandMode::kCartesianVelocity, ArmCommandDispatchMode::kCartesianVelocity);
  expect_mode(CommandMode::kJointPosition, ArmCommandDispatchMode::kJointPosition);
  expect_mode(CommandMode::kCartesianPose, ArmCommandDispatchMode::kCartesianPose);

  ArmCommand finished;
  finished.motion_finished = true;
  const auto finished_translation = translate_arm_command(finished, false);
  assert(finished_translation.has_value());
  assert(finished_translation->user_stopped);

  ArmCommand running;
  const auto user_stop_translation = translate_arm_command(running, true);
  assert(user_stop_translation.has_value());
  assert(user_stop_translation->user_stopped);

  ArmCommand unknown;
  unknown.mode = static_cast<CommandMode>(std::uint8_t{255});
  assert(!translate_arm_command(unknown, false).has_value());
}
