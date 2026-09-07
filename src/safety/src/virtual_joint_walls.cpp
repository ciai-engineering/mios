#include "mios/safety_stage_2/virtual_joint_walls.hpp"

namespace mios {

VirtualJointWallsSafetyModule::VirtualJointWallsSafetyModule():m_virtual_walls_on(false){

}

void VirtualJointWallsSafetyModule::initialize(const Percept &p_0, const control::ControlRuntimeConfig& config){
    initialize_virt_walls(p_0,config);
}

void VirtualJointWallsSafetyModule::step(const Percept &p, control::ArmCommand& cmd){
    if(m_virtual_walls_on && cmd.mode == control::CommandMode::kTorque){
        input_virt_walls(p);
        m_walls.step();
        for(unsigned i=0;i<7;i++){
            cmd.joints[i] += m_walls.y.tau_vwalls[i];
        }
    }
}

void VirtualJointWallsSafetyModule::terminate(){
    m_walls.terminate();
}

void VirtualJointWallsSafetyModule::initialize_virt_walls(const Percept &p,const control::ControlRuntimeConfig& config){
    const SafetyParameters& p_cntr=config.safety;
    m_walls.p.damping_distance=p_cntr.virtual_joint_walls.damping_dist;
    m_walls.p.damping_factor=p_cntr.virtual_joint_walls.damping;
    m_walls.p.eta=p_cntr.virtual_joint_walls.eta;
    m_walls.p.rho_min=p_cntr.virtual_joint_walls.rho_min;
    m_walls.p.tau_max=p_cntr.virtual_joint_walls.tau_max;
    m_walls.p.walls=p_cntr.virtual_joint_walls.walls;

    input_virt_walls(p);

    m_walls.initialize();

    m_virtual_walls_on=config.safety.virtual_joint_walls.active;
}

void VirtualJointWallsSafetyModule::input_virt_walls(const Percept &p){
    m_walls.u.dq=p.proprioception.dq;
    m_walls.u.q=p.proprioception.q;
}


bool VirtualJointWallsSafetyModule::is_walls_valid(const Percept& p){
    std::array<bool,7> in_walls={false,false,false,false,false,false,false};

    bool safe_activation=true;
    for(unsigned i=0;i<7;i++){
        if(fabs(m_walls.y.wall_flag(i))>0){
            safe_activation=false;
            break;
        }
    }
    for(unsigned i=0;i<7;i++){
        if(p.proprioception.q(i)>m_walls.p.walls(i*2) || p.proprioception.q(i)<m_walls.p.walls(i*2+1)){
            in_walls[i]=false;
        }else{
            in_walls[i]=true;
        }
    }
    if(in_walls[0] && in_walls[1] && in_walls[2] && in_walls[3] && in_walls[4] && in_walls[5] && in_walls[6] && safe_activation){
        return true;
    }else{
        return false;
    }
}

}
