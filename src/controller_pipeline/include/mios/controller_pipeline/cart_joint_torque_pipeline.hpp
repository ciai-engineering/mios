#pragma once

#include "mios/controller_pipeline/controller_pipeline.hpp"


#include "cntr_aic/cntr_aic_wrapper.hpp"
#include "cntr_force/cntr_force_wrapper.hpp"
#include "cntr_joint_var_imp/cntr_joint_var_imp_wrapper.hpp"
#include "cntr_mux/cntr_mux_wrapper.hpp"
#include "conv_vel2pose/conv_vel2pose_wrapper.hpp"
#include "cntr_nullsp_proj/cntr_nullsp_proj_wrapper.hpp"

#include "Eigen/Core"

namespace mios {

class CartJointTorqueControllerPipeline : public ControllerPipeline{
public:
    CartJointTorqueControllerPipeline();
    ~CartJointTorqueControllerPipeline();
    void initialize(const Percept& p_0,const control::ControlRuntimeConfig& config) override;
    control::ArmCommand step(const Percept &p, const Actuator &cmd) override;
    bool is_valid_command(const control::ArmCommand& cmd) const override;
    void update_percept(Percept::Controller &p) override;
    void terminate() override;
    void context_switch(const Percept &p) override;

private:
    void initialize_cntr_aic(const Percept &p,const control::ControlRuntimeConfig& config);
    void initialize_cntr_joint_imp(const Percept &p,const control::ControlRuntimeConfig& config);
    void initialize_cntr_force(const Percept &p,const control::ControlRuntimeConfig& config);
    void initialize_cntr_mux(const Percept &p, const control::ControlRuntimeConfig& config);
    void initialize_cntr_nullsp(const Percept &p,const control::ControlRuntimeConfig& config);

    void input_cntr_aic(const Percept& p);
    void input_cntr_joint_imp(const Percept& p);
    void input_cntr_force(const Percept& p);
    void input_cntr_mux(const Percept& p);
    void input_cntr_nullsp(const Percept& p);

private:
    control::ArmCommand m_command{};
    bool m_nullspace_control_on;
    Eigen::Matrix<double,4,4> m_T_T_EE_0;
    Eigen::Matrix<double,4,4> m_O_T_EE_d;
    Eigen::Matrix<double,7,1> m_q_d;
    Eigen::Matrix<double,7,1> m_q_0;



private:
    cntr_aic::cntr_aic m_cntr_aic;
    cntr_force::cntr_force m_cntr_force;
    cntr_mux::cntr_mux m_cntr_mux;
    conv_vel2pose::conv_vel2pose m_conv_vel2pose;
    cntr_joint_var_imp::cntr_joint_var_imp m_cntr_joint_imp;
    cntr_joint_var_imp::cntr_joint_var_imp m_cntr_nullsp_q;
    cntr_nullsp_proj::cntr_nullsp_proj m_cntr_nullsp_proj;

};

}
