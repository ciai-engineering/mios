#pragma once

#include "mios/task/task.hpp"

namespace mios {

// A deliberately inert task used to validate the legacy MIOS scheduler on the
// ROS-only backend. It must remain free of skills, gripper operations, robot
// parameter application, and any Core control-cycle invocation.
class Ros2NoMotionSelfTest : public Task {
 public:
  explicit Ros2NoMotionSelfTest(Core* core);

  void initialize_context() override;
  void execute() override;
  void get_default_context(nlohmann::json& context) override;
  bool read_parameters(const nlohmann::json& parameters) override;
  void write_custom_results(nlohmann::json& custom_results) override;
};

}  // namespace mios
