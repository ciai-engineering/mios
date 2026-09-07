#include "mios/controller_pipeline/null_pipeline.hpp"
#include "spdlog/spdlog.h"

namespace mios {

NullControllerPipeline::NullControllerPipeline(){
    m_command.mode = control::CommandMode::kTorque;
    spdlog::trace("NullControllerPipeline::NullControllerPipeline");
}

void NullControllerPipeline::initialize([[maybe_unused]] const Percept &p_0, [[maybe_unused]] const control::ControlRuntimeConfig& config){
    spdlog::trace("NullControllerPipeline::initialize");
}

control::ArmCommand NullControllerPipeline::step([[maybe_unused]] const Percept &p,
                                                  [[maybe_unused]] const Actuator &cmd){
    return m_command;
}

bool NullControllerPipeline::is_valid_command([[maybe_unused]] const control::ArmCommand& cmd) const{
    return false;
}

void NullControllerPipeline::update_percept([[maybe_unused]] Percept::Controller &p){

}

void NullControllerPipeline::terminate(){
    spdlog::trace("NullControllerPipeline::terminate");
}

void NullControllerPipeline::context_switch([[maybe_unused]] const Percept &p){
    spdlog::trace("NullControllerPipeline::context_switch");
}

}
