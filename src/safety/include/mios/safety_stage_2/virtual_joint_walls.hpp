#pragma once

#include "mios/safety_stage_2/safety_module_stage_2.hpp"
#include "virtual_walls_joint/virtual_walls_joint_wrapper.hpp"

namespace mios{

class VirtualJointWallsSafetyModule : public SafetyModuleStage2{
public:
    VirtualJointWallsSafetyModule();

    void initialize(const Percept &p_0, const control::ControlRuntimeConfig& config) override;
    void step(const Percept &p, control::ArmCommand& cmd) override;
    void terminate() override;

private:
    void initialize_virt_walls(const Percept &p, const control::ControlRuntimeConfig& config);
    void input_virt_walls(const Percept& p);
    bool is_walls_valid(const Percept &p);

private:
    virtual_walls_joint::virtual_walls_joint m_walls;
    bool m_virtual_walls_on;

};
}
