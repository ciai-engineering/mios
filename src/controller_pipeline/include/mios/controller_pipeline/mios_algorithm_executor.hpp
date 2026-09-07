#pragma once

#include <memory>
#include <vector>

#include "mios/controller_pipeline/controller_pipeline.hpp"
#include "mios/safety_stage_1/safety_module_stage_1.hpp"
#include "mios/safety_stage_2/safety_module_stage_2.hpp"

namespace mios {

// Runs the original MIOS controller-pipeline and safety sequence without
// owning an FCI/libfranka connection. A robot backend supplies snapshots and
// is solely responsible for converting ArmCommand into its transport command.
class MiosAlgorithmExecutor {
 public:
  struct CycleResult {
    control::ArmCommand command{};
    bool valid{false};
  };

  MiosAlgorithmExecutor(std::unique_ptr<ControllerPipeline> pipeline,
                        control::CommandMode command_mode);
  ~MiosAlgorithmExecutor();

  MiosAlgorithmExecutor(const MiosAlgorithmExecutor&) = delete;
  MiosAlgorithmExecutor& operator=(const MiosAlgorithmExecutor&) = delete;

  void add_safety_stage_1(std::unique_ptr<SafetyModuleStage1> safety_module);
  void add_safety_stage_2(std::unique_ptr<SafetyModuleStage2> safety_module);

  bool initialize(Percept& initial_percept,
                  const control::ControlRuntimeConfig& config);
  CycleResult step(Percept& percept, Actuator& actuator);
  void update_percept(Percept::Controller& controller_percept);
  void terminate();

  [[nodiscard]] bool initialized() const { return initialized_; }
  [[nodiscard]] control::CommandMode command_mode() const { return command_mode_; }

 private:
  [[nodiscard]] control::ArmCommand safe_command() const;
  [[nodiscard]] bool command_is_finite(const control::ArmCommand& command) const;

  std::unique_ptr<ControllerPipeline> pipeline_;
  std::vector<std::unique_ptr<SafetyModuleStage1>> safety_stage_1_;
  std::vector<std::unique_ptr<SafetyModuleStage2>> safety_stage_2_;
  control::CommandMode command_mode_;
  control::ControlRuntimeConfig config_{};
  bool initialized_{false};
};

}  // namespace mios
