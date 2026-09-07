#include "mios/controller_pipeline/joint_torque_pipeline.hpp"
#include "spdlog/spdlog.h"

namespace mios {

JointTorqueControllerPipeline::JointTorqueControllerPipeline(){
    m_command.mode = control::CommandMode::kTorque;
    spdlog::trace("JointTorqueControllerPipeline::JointTorqueControllerPipeline()");
}


void JointTorqueControllerPipeline::initialize(const Percept &p_0, const control::ControlRuntimeConfig& config){
    spdlog::trace("JointTorqueControllerPipeline::initialize()");
    initialize_cntr_joint_imp(p_0,config);
    initialize_cntr_mux(p_0,config);

    m_q_0=p_0.proprioception.q;
}

control::ArmCommand JointTorqueControllerPipeline::step(const Percept &p, const Actuator &cmd){
    input_cntr_joint_imp(p);
    input_cntr_mux(p);

    if(cmd.get_command_pattern()->find(CommandPatternJointPose)!=cmd.get_command_pattern()->end()){
        m_cntr_joint_imp.u.theta_d=cmd.q_d;
    }else if(cmd.get_command_pattern()->find(CommandPatternJointVelocities)!=cmd.get_command_pattern()->end()){
        m_q_d+=cmd.dq_d*0.001;
        m_cntr_joint_imp.u.theta_d=m_q_d;
    }else{
        m_cntr_joint_imp.u.theta_d=m_q_0;
    }
    m_cntr_joint_imp.u.K_theta=cmd.K_theta;

    m_cntr_joint_imp.step();

    Eigen::Matrix<double,7,1> tau_J_d_total=m_cntr_joint_imp.y.tau_J_d+cmd.tau_ff;

    m_cntr_mux.u.tau_J_d=tau_J_d_total;
    m_cntr_mux.step();
    m_command.joints = {tau_J_d_total(0), tau_J_d_total(1), tau_J_d_total(2), tau_J_d_total(3),
                        tau_J_d_total(4), tau_J_d_total(5), tau_J_d_total(6)};

    return m_command;
}

bool JointTorqueControllerPipeline::is_valid_command(const control::ArmCommand& cmd) const{
    if (cmd.mode != control::CommandMode::kTorque) {
        return false;
    }
    for(unsigned i=0;i<7;i++){
        if(cmd.joints[i] != cmd.joints[i]){
            return false;
        }
    }
    return true;
}

void JointTorqueControllerPipeline::update_percept(Percept::Controller &p){
    p.q_d=m_cntr_joint_imp.u.theta_d;
    p.dq_d=m_cntr_joint_imp.u.dtheta_d;
}

void JointTorqueControllerPipeline::terminate(){
    spdlog::trace("JointTorqueControllerPipeline::terminate()");
    m_cntr_joint_imp.terminate();
    m_cntr_mux.terminate();
}

void JointTorqueControllerPipeline::context_switch(const Percept &p){
    spdlog::trace("JointTorqueControllerPipeline::context_switch()");
    m_q_d=p.proprioception.q;
    m_q_0=m_q_d;
}

void JointTorqueControllerPipeline::initialize_cntr_joint_imp(const Percept &p, const control::ControlRuntimeConfig& config){
    spdlog::trace("JointTorqueControllerPipeline::initialize_cntr_joint_imp()");
    m_q_d=p.proprioception.q;
    const ControlParameters& p_cntr=config.control;

    m_cntr_joint_imp.p.enable_ffwd_acc.setZero();
    m_cntr_joint_imp.p.enable_ffwd_vel.setZero();

    input_cntr_joint_imp(p);

    m_cntr_joint_imp.u.K_theta=p_cntr.joint_imp.K_theta;
    m_cntr_joint_imp.u.D_theta=p_cntr.joint_imp.xi_theta;

    m_cntr_joint_imp.initialize();
}

void JointTorqueControllerPipeline::input_cntr_joint_imp(const Percept &p){
    m_cntr_joint_imp.u.theta=p.proprioception.q;
    m_cntr_joint_imp.u.theta_d=p.proprioception.q;
    m_cntr_joint_imp.u.dtheta=p.proprioception.dq;
    m_cntr_joint_imp.u.dtheta_d<<0,0,0,0,0,0,0;
    m_cntr_joint_imp.u.ddtheta_d<<0,0,0,0,0,0,0;
    m_cntr_joint_imp.u.M=p.internal_model.M;
    m_cntr_joint_imp.u.tau_ff.setZero();
}

void JointTorqueControllerPipeline::initialize_cntr_mux(const Percept &p,const control::ControlRuntimeConfig& config){
    spdlog::trace("JointTorqueControllerPipeline::initialize_cntr_mux()");
    const LimitParameters& p_limits=config.limits;

    m_cntr_mux.p.dtau_max=p_limits.joint_space.dtau_J_max;
    m_cntr_mux.p.tau_max=p_limits.joint_space.tau_J_max;

    input_cntr_mux(p);

    m_cntr_mux.initialize();
}

void JointTorqueControllerPipeline::input_cntr_mux(const Percept &p){
    m_cntr_mux.u.B_J_EE=p.internal_model.B_J_EE;
    m_cntr_mux.u.tau_J_d<<0,0,0,0,0,0,0;
}

}
