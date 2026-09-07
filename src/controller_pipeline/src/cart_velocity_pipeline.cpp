#include "mios/controller_pipeline/cart_velocity_pipeline.hpp"
#include "spdlog/spdlog.h"

namespace mios {

CartVelocityControllerPipeline::CartVelocityControllerPipeline(){
    m_command.mode = control::CommandMode::kCartesianVelocity;
    spdlog::trace("CartVelocityControllerPipeline::CartVelocityControllerPipeline");
}


void CartVelocityControllerPipeline::initialize([[maybe_unused]] const Percept &p_0, [[maybe_unused]] const control::ControlRuntimeConfig& config){
    spdlog::trace("CartVelocityControllerPipeline::initialize");
}

control::ArmCommand CartVelocityControllerPipeline::step([[maybe_unused]] const Percept &p,
                                                           const Actuator &cmd){

    Eigen::Matrix<double,6,1> O_dX_d;
    O_dX_d<<cmd.O_R_T*cmd.TF_dX_d.block<3,1>(0,0),cmd.O_R_T*cmd.TF_dX_d.block<3,1>(3,0);
    m_command.cartesian = {O_dX_d(0), O_dX_d(1), O_dX_d(2), O_dX_d(3), O_dX_d(4), O_dX_d(5)};
    return m_command;
}

bool CartVelocityControllerPipeline::is_valid_command(const control::ArmCommand& cmd) const{
    if (cmd.mode != control::CommandMode::kCartesianVelocity) {
        return false;
    }
    for(unsigned i=0;i<6;i++){
        if(cmd.cartesian[i] != cmd.cartesian[i]){
            return false;
        }
    }
    return true;
}

void CartVelocityControllerPipeline::update_percept([[maybe_unused]] Percept::Controller &p){

}

void CartVelocityControllerPipeline::terminate(){
    spdlog::trace("CartVelocityControllerPipeline::terminate");
}

void CartVelocityControllerPipeline::context_switch([[maybe_unused]] const Percept &p){
    spdlog::trace("CartVelocityControllerPipeline::context_switch");
}

}
