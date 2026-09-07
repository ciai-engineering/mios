#include "mios/safety_stage_1/velocity_walls.hpp"

namespace mios {

VelocityWallsSafetyModule::VelocityWallsSafetyModule(){
    m_walls.setZero();
    m_brake_distance=0;
}

void VelocityWallsSafetyModule::initialize([[maybe_unused]] const Percept &p_0,
                                           const control::ControlRuntimeConfig& config){
    m_walls=config.safety.velocity_walls.walls;
    m_brake_distance=config.safety.velocity_walls.brake_distance;
    m_active=config.safety.velocity_walls.active;
}

void VelocityWallsSafetyModule::step(const Percept &p, Actuator &cmd){
    if(m_active){
        double dx_scale=0;
        for(unsigned i=0;i<3;i++){
            double diff_lower = p.proprioception.T_T_EE(i,3)-m_walls(2*i);
            dx_scale=diff_lower/m_brake_distance;
            if(dx_scale>1)dx_scale=1;
            if(dx_scale<0)dx_scale=0;
            if(p.proprioception.T_T_EE(i,3)<=m_walls(2*i)+m_brake_distance && p.proprioception.TF_dX_EE(i)<0){
                cmd.TF_dX_d(i)*=dx_scale;
            }
            double diff_upper = m_walls(2*i+1)-p.proprioception.T_T_EE(i,3);
            dx_scale=diff_upper/m_brake_distance;
            if(dx_scale>1)dx_scale=1;
            if(dx_scale<0)dx_scale=0;
            if(p.proprioception.T_T_EE(i,3)>=m_walls(2*i+1)-m_brake_distance && p.proprioception.TF_dX_EE(i)>0){
                cmd.TF_dX_d(i)*=dx_scale;
            }
        }
    }
}

void VelocityWallsSafetyModule::terminate(){

}

}
