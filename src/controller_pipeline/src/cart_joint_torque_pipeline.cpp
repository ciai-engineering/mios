#include "mios/controller_pipeline/cart_joint_torque_pipeline.hpp"
#include "spdlog/spdlog.h"
#include "mirmi_cpp_utils/math/math.hpp"

namespace mios {

CartJointTorqueControllerPipeline::CartJointTorqueControllerPipeline():m_nullspace_control_on(false){
    m_command.mode = control::CommandMode::kTorque;
    spdlog::trace("CartJointTorqueControllerPipeline::CartJointTorqueControllerPipeline()");
}

CartJointTorqueControllerPipeline::~CartJointTorqueControllerPipeline(){
    spdlog::trace("CartJointTorqueControllerPipeline::~CartJointTorqueControllerPipeline()");
}

void CartJointTorqueControllerPipeline::initialize(const Percept &p_0, const control::ControlRuntimeConfig& config){
    spdlog::trace("CartJointTorqueControllerPipeline::initialize()");
    initialize_cntr_aic(p_0,config);
    initialize_cntr_force(p_0,config);
    initialize_cntr_mux(p_0,config);
    initialize_cntr_nullsp(p_0,config);

    m_T_T_EE_0=p_0.proprioception.T_T_EE;
    m_O_T_EE_d=p_0.proprioception.O_T_EE;
}

control::ArmCommand CartJointTorqueControllerPipeline::step(const Percept &p, const Actuator &cmd){

    input_cntr_aic(p);
    input_cntr_force(p);
    input_cntr_mux(p);
    input_cntr_nullsp(p);

    if(cmd.get_command_pattern()->find(CommandPatternCartesianJointPose)!=cmd.get_command_pattern()->end()){
        m_cntr_aic.u.TF_T_EE_d=cmd.TF_T_EE_d;
        m_cntr_joint_imp.u.theta_d=cmd.q_d;

    }else if(cmd.get_command_pattern()->find(CommandPatternCartesianJointTwist)!=cmd.get_command_pattern()->end()){
        m_conv_vel2pose.u.TF_dX_d=cmd.TF_dX_d;
        m_conv_vel2pose.step();
        m_cntr_aic.u.TF_T_EE_d=m_conv_vel2pose.y.TF_T_EE_d;
        m_q_d+=cmd.dq_d*0.001;
        m_cntr_joint_imp.u.theta_d=m_q_d;
    }
    else{
        m_cntr_aic.u.TF_T_EE_d=m_T_T_EE_0;
        m_cntr_joint_imp.u.theta_d=m_q_0;
    }

    m_O_T_EE_d=mirmi_utils::rotate_matrix(m_cntr_aic.u.TF_T_EE_d,cmd.O_R_T);


    m_cntr_aic.u.O_R_T=cmd.O_R_T;

    m_cntr_aic.u.K_x=cmd.K_x;
    m_cntr_aic.u.xi_x=cmd.xi_x;
    m_cntr_aic.u.TF_F_ff=cmd.TF_F_ff;
    m_cntr_aic.step();

    m_cntr_force.u.TF_F_d_K=cmd.TF_F_d;
    m_cntr_force.u.O_R_T=cmd.O_R_T;
    m_cntr_force.step();

    m_cntr_nullsp_q.u.theta_d=cmd.q_d_nullspace;
    m_cntr_nullsp_q.step();
    m_cntr_nullsp_proj.u.tau_c=m_cntr_nullsp_q.y.tau_J_d;
    m_cntr_nullsp_proj.step();
    
    m_cntr_joint_imp.u.K_theta=cmd.K_theta;
    m_cntr_joint_imp.step();

    //Eigen::Matrix<double,7,1> tau_J_d_total=m_cntr_joint_imp.y.tau_J_d+cmd.tau_ff;

    Eigen::Matrix<double,7,1> tau_J_d_total=m_cntr_aic.y.tau_J_d+cmd.tau_ff+p.internal_model.C;
    if(m_nullspace_control_on){
        tau_J_d_total+=m_cntr_nullsp_proj.y.tau_n;
    }
    if(cmd.get_command_pattern()->find(CommandPatternDesiredWrench)!=cmd.get_command_pattern()->end()){
        tau_J_d_total+=m_cntr_force.y.tau_J_d;
    }

    m_cntr_mux.u.tau_J_d=tau_J_d_total;
    m_cntr_mux.step();
    m_command.joints = {m_cntr_mux.y.tau_J_d_checked(0), m_cntr_mux.y.tau_J_d_checked(1),
                        m_cntr_mux.y.tau_J_d_checked(2), m_cntr_mux.y.tau_J_d_checked(3),
                        m_cntr_mux.y.tau_J_d_checked(4), m_cntr_mux.y.tau_J_d_checked(5),
                        m_cntr_mux.y.tau_J_d_checked(6)};

    return m_command;
}

bool CartJointTorqueControllerPipeline::is_valid_command(const control::ArmCommand& cmd) const{
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

void CartJointTorqueControllerPipeline::update_percept(Percept::Controller &p){
    p.TF_T_EE_d=m_cntr_aic.u.TF_T_EE_d;
    p.TF_F_d=m_cntr_force.u.TF_F_d_K;
    p.TF_F_active=m_cntr_force.p.active.cast<bool>();
    p.K_x=m_cntr_aic.l.K_x;
    p.K_theta=m_cntr_nullsp_q.l.K_theta;
    p.q_d=m_cntr_joint_imp.u.theta_d;
    p.dq_d=m_cntr_joint_imp.u.dtheta_d;
}

void CartJointTorqueControllerPipeline::terminate(){
    spdlog::trace("CartJointTorqueControllerPipeline::terminate()");
    m_conv_vel2pose.terminate();
    m_cntr_aic.terminate();
    m_cntr_force.terminate();
    m_cntr_joint_imp.terminate();
    m_cntr_mux.terminate();
    m_cntr_nullsp_q.terminate();
    m_cntr_nullsp_proj.terminate();
}

void CartJointTorqueControllerPipeline::context_switch(const Percept &p){
    spdlog::trace("CartJointTorqueControllerPipeline::context_switch()");
    m_conv_vel2pose.u.TF_dX_d<<0,0,0,0,0,0;
    m_conv_vel2pose.u.TF_T_EE=mirmi_utils::rotate_matrix(m_O_T_EE_d,p.controller.O_R_T.transpose());
    m_conv_vel2pose.u.reset<<1;
    m_conv_vel2pose.step();
    m_conv_vel2pose.u.reset<<0;
    m_T_T_EE_0=p.proprioception.T_T_EE;
    m_q_d=p.proprioception.q;
    m_q_0=m_q_d;
}

void CartJointTorqueControllerPipeline::initialize_cntr_aic(const Percept &p,const control::ControlRuntimeConfig& config){
    spdlog::trace("CartJointTorqueControllerPipeline::initialize_cntr_aic()");
    const ControlParameters& p_cntr=config.control;
    const FramesParameters& p_frames=config.frames;
    const LimitParameters& p_limits=config.limits;
    m_cntr_aic.p.alpha=p_cntr.cart_imp_adaptation_stage.alpha;
    m_cntr_aic.p.beta=p_cntr.cart_imp_adaptation_stage.beta;
    m_cntr_aic.p.gamma_a=p_cntr.cart_imp_adaptation_stage.gamma_a;
    m_cntr_aic.p.gamma_b=p_cntr.cart_imp_adaptation_stage.gamma_b;
    m_cntr_aic.p.L=p_cntr.cart_imp_adaptation_stage.L;
    m_cntr_aic.p.F_ff_0=p_cntr.cart_imp_adaptation_stage.F_ff_0;
    m_cntr_aic.p.K_0=p_cntr.cart_imp.K_x;
    m_cntr_aic.p.xi=p_cntr.cart_imp.xi_x;
    m_cntr_aic.p.F_ff_max<<p_limits.cartesian_space.F_J_max(0),p_limits.cartesian_space.F_J_max(0),p_limits.cartesian_space.F_J_max(0),p_limits.cartesian_space.F_J_max(1),p_limits.cartesian_space.F_J_max(1),p_limits.cartesian_space.F_J_max(1);
    m_cntr_aic.p.dF_ff_max<<p_limits.cartesian_space.dF_J_max(0),p_limits.cartesian_space.dF_J_max(0),p_limits.cartesian_space.dF_J_max(0),p_limits.cartesian_space.dF_J_max(1),p_limits.cartesian_space.dF_J_max(1),p_limits.cartesian_space.dF_J_max(1);
    m_cntr_aic.p.K_max=p_limits.cartesian_space.K_x_max;
    m_cntr_aic.p.dK_max=p_limits.cartesian_space.dK_x_max;
    m_cntr_aic.p.EE_T_K=p_frames.EE_T_K;
    m_cntr_aic.p.dtau_max=p_limits.joint_space.dtau_J_max;
    m_cntr_aic.p.tau_max=p_limits.joint_space.tau_J_max;
    m_cntr_aic.p.kappa<<p_cntr.cart_imp_adaptation_stage.kappa;

    input_cntr_aic(p);

    m_cntr_aic.u.K_x=p_cntr.cart_imp.K_x;
    m_cntr_aic.u.xi_x=p_cntr.cart_imp.xi_x;

    m_cntr_aic.initialize();
    m_conv_vel2pose.u.reset<<1;
    m_conv_vel2pose.initialize(true,1000);
    m_conv_vel2pose.step();
    m_conv_vel2pose.u.reset<<0;

}

void CartJointTorqueControllerPipeline::input_cntr_aic(const Percept &p){
    m_cntr_aic.u.B_J_EE=p.internal_model.B_J_EE;
    m_cntr_aic.u.M=p.internal_model.M;
    m_cntr_aic.u.dtheta=p.proprioception.dtheta;
    m_cntr_aic.u.TF_F_ext=p.proprioception.TF_F_ext_K;
    m_cntr_aic.u.TF_F_ff<<0,0,0,0,0,0;
    m_cntr_aic.u.TF_T_EE=p.proprioception.T_T_EE;
    m_cntr_aic.u.TF_T_EE_d=p.proprioception.T_T_EE;
    m_cntr_aic.u.O_R_T=p.controller.O_R_T;

    m_conv_vel2pose.u.TF_dX_d<<0,0,0,0,0,0;
    m_conv_vel2pose.u.TF_T_EE=p.proprioception.T_T_EE;
    m_conv_vel2pose.u.reset<<0;
}

void CartJointTorqueControllerPipeline::initialize_cntr_joint_imp(const Percept &p, const control::ControlRuntimeConfig& config){
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

void CartJointTorqueControllerPipeline::input_cntr_joint_imp(const Percept &p){
    m_cntr_joint_imp.u.theta=p.proprioception.q;
    m_cntr_joint_imp.u.theta_d=p.proprioception.q;
    m_cntr_joint_imp.u.dtheta=p.proprioception.dq;
    m_cntr_joint_imp.u.dtheta_d<<0,0,0,0,0,0,0;
    m_cntr_joint_imp.u.ddtheta_d<<0,0,0,0,0,0,0;
    m_cntr_joint_imp.u.M=p.internal_model.M;
    m_cntr_joint_imp.u.tau_ff.setZero();
}

void CartJointTorqueControllerPipeline::initialize_cntr_force(const Percept &p, const control::ControlRuntimeConfig& config){
    spdlog::trace("CartJointTorqueControllerPipeline::initialize_cntr_force()");
    const ControlParameters& p_cntr=config.control;
    const LimitParameters& p_limits=config.limits;
    m_cntr_force.p.active=p_cntr.force_control.active;
    m_cntr_force.p.dF_d_max<<p_limits.cartesian_space.dF_J_max(0),p_limits.cartesian_space.dF_J_max(0),p_limits.cartesian_space.dF_J_max(0),p_limits.cartesian_space.dF_J_max(1),p_limits.cartesian_space.dF_J_max(1),p_limits.cartesian_space.dF_J_max(1);
    m_cntr_force.p.F_d_max<<p_limits.cartesian_space.F_J_max(0),p_limits.cartesian_space.F_J_max(0),p_limits.cartesian_space.F_J_max(0),p_limits.cartesian_space.F_J_max(1),p_limits.cartesian_space.F_J_max(1),p_limits.cartesian_space.F_J_max(1);
    m_cntr_force.p.dtau_max=p_limits.joint_space.dtau_J_max;
    m_cntr_force.p.tau_max=p_limits.joint_space.tau_J_max;
    m_cntr_force.p.k_p=p_cntr.force_control.k_p;
    m_cntr_force.p.k_i=p_cntr.force_control.k_i;
    m_cntr_force.p.k_d=p_cntr.force_control.k_d;
    m_cntr_force.p.k_d_N=p_cntr.force_control.k_d_N;
    m_cntr_force.p.d_max=p_cntr.force_control.d_max;
    m_cntr_force.p.phi_max<<p_cntr.force_control.phi_max;
    m_cntr_force.p.sf_on<<p_cntr.force_control.sf_on;

    input_cntr_force(p);

    m_cntr_force.initialize();
}

void CartJointTorqueControllerPipeline::input_cntr_force(const Percept &p){
    m_cntr_force.u.B_J_0=p.internal_model.B_J_O;
    m_cntr_force.u.DX<<0,0,0,0,0,0;
    m_cntr_force.u.TF_F_d_K<<0,0,0,0,0,0;
    m_cntr_force.u.TF_F_ext_K=p.proprioception.TF_F_ext_K;
    m_cntr_force.u.O_R_T=p.controller.O_R_T;
}

void CartJointTorqueControllerPipeline::initialize_cntr_mux(const Percept &p,const control::ControlRuntimeConfig& config){
    spdlog::trace("CartJointTorqueControllerPipeline::initialize_cntr_mux()");
    const LimitParameters& p_limits=config.limits;

    m_cntr_mux.p.dtau_max=p_limits.joint_space.dtau_J_max;
    m_cntr_mux.p.tau_max=p_limits.joint_space.tau_J_max;

    input_cntr_mux(p);

    m_cntr_mux.initialize();
}

void CartJointTorqueControllerPipeline::input_cntr_mux(const Percept &p){
    m_cntr_mux.u.B_J_EE=p.internal_model.B_J_EE;
    m_cntr_mux.u.tau_J_d<<0,0,0,0,0,0,0;
}

void CartJointTorqueControllerPipeline::initialize_cntr_nullsp(const Percept& p,const control::ControlRuntimeConfig& config){
    spdlog::trace("CartJointTorqueControllerPipeline::initialize_cntr_nullsp()");
    const ControlParameters& p_cntr=config.control;

    m_cntr_nullsp_q.u.theta_d=p.proprioception.q;

    m_cntr_nullsp_q.p.enable_ffwd_acc.setZero();
    m_cntr_nullsp_q.p.enable_ffwd_vel.setZero();

    m_cntr_nullsp_proj.p.singlr_comp_mode.setZero();
    m_cntr_nullsp_proj.p.singlr_threshold.setZero();

    input_cntr_nullsp(p);

    m_cntr_nullsp_q.u.K_theta=p_cntr.joint_imp.K_theta;
    m_cntr_nullsp_q.u.D_theta=p_cntr.joint_imp.xi_theta;

    m_cntr_nullsp_q.initialize();
    m_cntr_nullsp_proj.initialize();

    m_nullspace_control_on = config.control.nullspace_control.active;
}

void CartJointTorqueControllerPipeline::input_cntr_nullsp(const Percept &p){
    m_cntr_nullsp_q.u.theta=p.proprioception.q;
    m_cntr_nullsp_q.u.theta_d=p.proprioception.q;
    m_cntr_nullsp_q.u.dtheta=p.proprioception.dq;
    m_cntr_nullsp_q.u.dtheta_d<<0,0,0,0,0,0,0;
    m_cntr_nullsp_q.u.ddtheta_d<<0,0,0,0,0,0,0;
    m_cntr_nullsp_q.u.M=p.internal_model.M;
    m_cntr_nullsp_q.u.tau_ff.setZero();

    m_cntr_nullsp_proj.u.J=p.internal_model.B_J_EE;
    m_cntr_nullsp_proj.u.M=p.internal_model.M;
    m_cntr_nullsp_proj.u.tau_c.setZero();
}

}
