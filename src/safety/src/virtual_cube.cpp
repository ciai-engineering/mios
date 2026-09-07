#include "mios/safety_stage_2/virtual_cube.hpp"
#include "spdlog/spdlog.h"

namespace mios {

VirtualCubeSafetyModule::VirtualCubeSafetyModule():m_virtual_cube_on(false),m_safe_activation(false){
    spdlog::trace("VirtualCubeSafetyModule::VirtualCubeSafetyModule");
}

VirtualCubeSafetyModule::~VirtualCubeSafetyModule(){
    spdlog::trace("VirtualCubeSafetyModule::~VirtualCubeSafetyModule()");
}

void VirtualCubeSafetyModule::initialize(const Percept &p_0, const control::ControlRuntimeConfig& config){
    initialize_virt_cube(p_0,config);
}

void VirtualCubeSafetyModule::step(const Percept &p, control::ArmCommand& cmd){
    if(m_virtual_cube_on && cmd.mode == control::CommandMode::kTorque){
        input_virt_cube(p);
        m_cube.step();
        if(!m_safe_activation && is_cube_valid(p)){
            m_safe_activation=true;
        }
        if(m_safe_activation){
            for(unsigned i=0;i<7;i++){
                cmd.joints[i] += m_cube.y.tau_vwalls[i];
            }
        }
    }
}

void VirtualCubeSafetyModule::terminate(){
    m_cube.terminate();
}

void VirtualCubeSafetyModule::initialize_virt_cube(const Percept &p,const control::ControlRuntimeConfig& config){
    const SafetyParameters& p_safety=config.safety;
    m_cube.p.damping_distance<<p_safety.virtual_cube.damping_dist;
    m_cube.p.damping_factor<<p_safety.virtual_cube.damping;
    m_cube.p.eta<<p_safety.virtual_cube.eta;
    m_cube.p.rho_min<<p_safety.virtual_cube.rho_min;
    m_cube.p.cube_walls=p_safety.virtual_cube.walls;
    m_cube.p.f_max<<p_safety.virtual_cube.f_max;

    input_virt_cube(p);
    m_cube.initialize(true,5000);

    m_virtual_cube_on=config.safety.virtual_cube.active;
}

void VirtualCubeSafetyModule::input_virt_cube(const Percept &p){
    m_cube.u.dx_EE=p.proprioception.O_dX_EE;
    m_cube.u.p_EE=p.proprioception.O_T_EE.block<3,1>(0,3);
    m_cube.u.Jacobian_EE=p.internal_model.B_J_O;
}

bool VirtualCubeSafetyModule::is_cube_valid(const Percept& p){
    std::array<bool,3> in_cube={false,false,false};
    bool safe_activation=true;
    for(unsigned i=0;i<6;i++){
        if(fabs(m_cube.y.wall_flag(i))>0){
            safe_activation=false;
            break;
        }
    }
    for(unsigned i=0;i<3;i++){
        if(p.proprioception.O_T_EE(i,3)>m_cube.p.cube_walls(i*2) || p.proprioception.O_T_EE(i,3)<m_cube.p.cube_walls(i*2+1)){
            in_cube[i]=false;
        }else{
            in_cube[i]=true;
        }
    }
    if(in_cube[0] && in_cube[1] && in_cube[2] && safe_activation){
        return true;
    }
    else{
        return false;
    }
}

}
