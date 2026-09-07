#include "mios/tasks/ros2_no_motion_self_test.hpp"

#include "spdlog/spdlog.h"

namespace mios {

Ros2NoMotionSelfTest::Ros2NoMotionSelfTest(Core* core)
    : Task("Ros2NoMotionSelfTest", core) {}

void Ros2NoMotionSelfTest::initialize_context() {}

void Ros2NoMotionSelfTest::execute() {
  // Intentionally no robot-side operation. Returning completes the task
  // successfully through the normal MIOS task lifecycle.
  spdlog::info("ROS 2 no-motion self-test completed without a robot command.");
}

void Ros2NoMotionSelfTest::get_default_context(nlohmann::json& context) {
  context["parameters"] = nlohmann::json::object();
  context["skills"] = nlohmann::json::object();
}

bool Ros2NoMotionSelfTest::read_parameters(const nlohmann::json& /*parameters*/) {
  // This task deliberately has no configurable behavior.
  return true;
}

void Ros2NoMotionSelfTest::write_custom_results(nlohmann::json& custom_results) {
  custom_results["robot_commanded"] = false;
  custom_results["gripper_commanded"] = false;
  custom_results["robot_parameters_applied"] = false;
}

}  // namespace mios
