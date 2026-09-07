#pragma once

#include "mios/safety_stage_1/safety_module_stage_1.hpp"

#include "Eigen/Core"

namespace mios{

class VelocityWallsSafetyModule : public SafetyModuleStage1{
public:
    VelocityWallsSafetyModule();

    void initialize(const Percept &p_0, const control::ControlRuntimeConfig& config) override;
    void step(const Percept &p, Actuator &cmd) override;
    void terminate() override;

private:
    Eigen::Matrix<double,6,1> m_walls;
    double m_brake_distance;
    bool m_active;

};
}
