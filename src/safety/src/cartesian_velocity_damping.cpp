#include "mios/safety_stage_2/cartesian_velocity_damping.hpp"
#include "mirmi_cpp_utils/math/math.hpp"

namespace mios{

CartesianVelocityDampingSafetyModule::CartesianVelocityDampingSafetyModule():m_damping_on(false),m_D_x(Eigen::Matrix<double,6,1>::Zero(6,1)),m_dX_thr(Eigen::Matrix<double,6,1>::Zero(6,1)){

}

void CartesianVelocityDampingSafetyModule::initialize([[maybe_unused]] const Percept &p_0,
                                                       const control::ControlRuntimeConfig& config){
    m_damping_on = config.safety.cartesian_velocity_damping.active;
    m_dX_thr = config.safety.cartesian_velocity_damping.dX_thr;
    m_D_x = config.safety.cartesian_velocity_damping.D_x;
}

void CartesianVelocityDampingSafetyModule::step(const Percept &p, control::ArmCommand& cmd){
    if(m_damping_on && cmd.mode == control::CommandMode::kTorque){

        Eigen::Matrix<double,6,1> F_damp;
        F_damp.setZero();
        for(unsigned i=0;i<6;i++){
            if(fabs(p.proprioception.TF_dX_EE(i))>m_dX_thr(i)){
                F_damp(i)=-mirmi_utils::sgn(p.proprioception.TF_dX_EE(i))*(fabs(p.proprioception.TF_dX_EE(i))-m_dX_thr(i))*m_D_x(i);
            }
        }
        Eigen::Matrix<double,7,1> tau_damp = p.internal_model.B_J_EE.transpose()*F_damp;
        for(unsigned i=0;i<7;i++){
            cmd.joints[i] += tau_damp(i);
        }
    }
}

void CartesianVelocityDampingSafetyModule::terminate(){

}

}
