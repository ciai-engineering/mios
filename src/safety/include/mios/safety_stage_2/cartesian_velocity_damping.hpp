#pragma once

#include "mios/safety_stage_2/safety_module_stage_2.hpp"

namespace mios{

class CartesianVelocityDampingSafetyModule : public SafetyModuleStage2{
public:
    CartesianVelocityDampingSafetyModule();

    void initialize(const Percept &p_0, const control::ControlRuntimeConfig& config) override;
    void step(const Percept &p, control::ArmCommand& cmd) override;
    void terminate() override;

private:
    bool m_damping_on;
    Eigen::Matrix<double,6,1> m_D_x;
    Eigen::Matrix<double,6,1> m_dX_thr;
};
}
