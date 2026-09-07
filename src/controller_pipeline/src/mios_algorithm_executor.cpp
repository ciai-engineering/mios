#include "mios/controller_pipeline/mios_algorithm_executor.hpp"

#include <cmath>

namespace mios {

MiosAlgorithmExecutor::MiosAlgorithmExecutor(
    std::unique_ptr<ControllerPipeline> pipeline,
    const control::CommandMode command_mode)
    : pipeline_(std::move(pipeline)), command_mode_(command_mode) {}

MiosAlgorithmExecutor::~MiosAlgorithmExecutor() {
  terminate();
}

void MiosAlgorithmExecutor::add_safety_stage_1(
    std::unique_ptr<SafetyModuleStage1> safety_module) {
  if (safety_module) {
    safety_stage_1_.push_back(std::move(safety_module));
  }
}

void MiosAlgorithmExecutor::add_safety_stage_2(
    std::unique_ptr<SafetyModuleStage2> safety_module) {
  if (safety_module) {
    safety_stage_2_.push_back(std::move(safety_module));
  }
}

bool MiosAlgorithmExecutor::initialize(
    Percept& initial_percept, const control::ControlRuntimeConfig& config) {
  if (!pipeline_) {
    return false;
  }

  config_ = config;
  pipeline_->initialize(initial_percept, config_);
  pipeline_->update_percept(initial_percept.controller);
  for (const auto& safety_module : safety_stage_1_) {
    safety_module->initialize(initial_percept, config_);
  }
  for (const auto& safety_module : safety_stage_2_) {
    safety_module->initialize(initial_percept, config_);
  }
  initialized_ = true;
  return true;
}

MiosAlgorithmExecutor::CycleResult MiosAlgorithmExecutor::step(Percept& percept,
                                                                 Actuator& actuator) {
  if (!initialized_ || !pipeline_) {
    return {safe_command(), false};
  }

  for (const auto& safety_module : safety_stage_1_) {
    safety_module->step(percept, actuator);
  }
  actuator.limit_output_rate(config_.limits);
  actuator.limit_output(config_.limits);
  if (actuator.is_new()) {
    pipeline_->context_switch(percept);
  }

  control::ArmCommand command = pipeline_->step(percept, actuator);
  if (command.mode != command_mode_ || !pipeline_->is_valid_command(command) ||
      !command_is_finite(command)) {
    return {safe_command(), false};
  }

  percept.update_controller();
  pipeline_->update_percept(percept.controller);
  for (const auto& safety_module : safety_stage_2_) {
    safety_module->step(percept, command);
  }

  if (command.mode != command_mode_ || !command_is_finite(command)) {
    return {safe_command(), false};
  }
  return {command, true};
}

void MiosAlgorithmExecutor::update_percept(Percept::Controller& controller_percept) {
  if (pipeline_) {
    pipeline_->update_percept(controller_percept);
  }
}

void MiosAlgorithmExecutor::terminate() {
  if (!initialized_) {
    return;
  }
  for (const auto& safety_module : safety_stage_1_) {
    safety_module->terminate();
  }
  for (const auto& safety_module : safety_stage_2_) {
    safety_module->terminate();
  }
  pipeline_->terminate();
  initialized_ = false;
}

control::ArmCommand MiosAlgorithmExecutor::safe_command() const {
  control::ArmCommand command;
  command.mode = command_mode_;
  command.motion_finished = true;
  return command;
}

bool MiosAlgorithmExecutor::command_is_finite(
    const control::ArmCommand& command) const {
  for (const double value : command.joints) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  for (const double value : command.cartesian) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  for (const double value : command.pose) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

}  // namespace mios
