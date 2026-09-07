#include "mios/controller_pipeline/joint_velocity_pipeline.hpp"
#include "spdlog/spdlog.h"

namespace mios {

JointVelocityControllerPipeline::JointVelocityControllerPipeline(){
    m_command.mode = control::CommandMode::kJointVelocity;
    spdlog::trace("JointVelocityControllerPipeline::JointVelocityControllerPipeline");
}


void JointVelocityControllerPipeline::initialize([[maybe_unused]] const Percept &p_0, [[maybe_unused]] const control::ControlRuntimeConfig& config){
    spdlog::trace("JointVelocityControllerPipeline::initialize");
}

control::ArmCommand JointVelocityControllerPipeline::step([[maybe_unused]] const Percept &p,
                                                           const Actuator &cmd){
    if(cmd.get_command_pattern()->find(CommandPatternJointPose)!=cmd.get_command_pattern()->end()){

    }
    m_command.joints = {cmd.dq_d(0), cmd.dq_d(1), cmd.dq_d(2), cmd.dq_d(3),
                        cmd.dq_d(4), cmd.dq_d(5), cmd.dq_d(6)};
    return m_command;
}

bool JointVelocityControllerPipeline::is_valid_command(const control::ArmCommand& cmd) const{
    if (cmd.mode != control::CommandMode::kJointVelocity) {
        return false;
    }
    for(unsigned i=0;i<7;i++){
        if(cmd.joints[i] != cmd.joints[i]){
            return false;
        }
    }
    return true;
}

void JointVelocityControllerPipeline::update_percept([[maybe_unused]] Percept::Controller &p){
}

void JointVelocityControllerPipeline::terminate(){
    spdlog::trace("JointVelocityControllerPipeline::terminate");
}

void JointVelocityControllerPipeline::context_switch([[maybe_unused]] const Percept &p){
    spdlog::trace("JointVelocityControllerPipeline::context_switch");
}

}
