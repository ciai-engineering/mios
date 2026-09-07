#include "mios/controller_pipeline/joint_position_pipeline.hpp"

#include "spdlog/spdlog.h"

namespace mios {

JointPositionControllerPipeline::JointPositionControllerPipeline() {
  m_command.mode = control::CommandMode::kJointPosition;
  spdlog::trace("JointPositionControllerPipeline::JointPositionControllerPipeline");
}

void JointPositionControllerPipeline::initialize(
    const Percept& p_0, const control::ControlRuntimeConfig& config) {
  m_q_d = p_0.proprioception.q;
  m_q_0 = m_q_d;
  m_q_lower = config.limits.joint_space.q_lower;
  m_q_upper = config.limits.joint_space.q_upper;
  spdlog::trace("JointPositionControllerPipeline::initialize");
}

control::ArmCommand JointPositionControllerPipeline::step(const Percept&,
                                                           const Actuator& cmd) {
  m_q_d = detail::next_joint_position_target(*cmd.get_command_pattern(), m_q_d, m_q_0,
                                              cmd.q_d, cmd.dq_d, m_q_lower, m_q_upper);

  m_command.joints = {m_q_d(0), m_q_d(1), m_q_d(2), m_q_d(3),
                      m_q_d(4), m_q_d(5), m_q_d(6)};
  return m_command;
}

bool JointPositionControllerPipeline::is_valid_command(
    const control::ArmCommand& cmd) const {
  return detail::is_valid_joint_position_command(cmd);
}

void JointPositionControllerPipeline::update_percept(Percept::Controller& p) {
  p.q_d = m_q_d;
  p.dq_d.setZero();
}

void JointPositionControllerPipeline::terminate() {
  spdlog::trace("JointPositionControllerPipeline::terminate");
}

void JointPositionControllerPipeline::context_switch(const Percept& p) {
  // Start a replacement primitive at the actual pose to avoid carrying a
  // stale integrated target across a task/primitive context transition.
  m_q_d = p.proprioception.q;
  m_q_0 = m_q_d;
  spdlog::trace("JointPositionControllerPipeline::context_switch");
}

}  // namespace mios
