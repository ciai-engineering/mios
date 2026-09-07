#pragma once

#include "mios/controller_pipeline/controller_pipeline.hpp"

#include "cntr_joint_var_imp/cntr_joint_var_imp_wrapper.hpp"
#include "cntr_mux/cntr_mux_wrapper.hpp"

#include "Eigen/Core"

namespace mios {

class JointTorqueControllerPipeline : public ControllerPipeline{
public:
    JointTorqueControllerPipeline();
    void initialize(const Percept& p_0,const control::ControlRuntimeConfig& config) override;
    control::ArmCommand step(const Percept &p, const Actuator &cmd) override;
    bool is_valid_command(const control::ArmCommand& cmd) const override;
    void update_percept(Percept::Controller &p) override;
    void terminate() override;
    void context_switch(const Percept &p) override;

private:
    void initialize_cntr_joint_imp(const Percept &p,const control::ControlRuntimeConfig& config);
    void initialize_cntr_mux(const Percept &p, const control::ControlRuntimeConfig& config);

    void input_cntr_joint_imp(const Percept& p);
    void input_cntr_mux(const Percept& p);

private:
    control::ArmCommand m_command{};
    Eigen::Matrix<double,7,1> m_q_d;
    Eigen::Matrix<double,7,1> m_q_0;

private:
    cntr_mux::cntr_mux m_cntr_mux;
    cntr_joint_var_imp::cntr_joint_var_imp m_cntr_joint_imp;
};

}
