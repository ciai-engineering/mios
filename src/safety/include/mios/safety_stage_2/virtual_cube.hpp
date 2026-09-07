#pragma once

#include "mios/safety_stage_2/safety_module_stage_2.hpp"
#include "virtual_cube/virtual_cube_wrapper.hpp"

namespace mios{

class VirtualCubeSafetyModule : public SafetyModuleStage2{
public:
    VirtualCubeSafetyModule();
    ~VirtualCubeSafetyModule();

    void initialize(const Percept &p_0, const control::ControlRuntimeConfig& config) override;
    void step(const Percept &p, control::ArmCommand& cmd) override;
    void terminate() override;

private:
    void initialize_virt_cube(const Percept &p, const control::ControlRuntimeConfig& config);
    void input_virt_cube(const Percept& p);
    bool is_cube_valid(const Percept &p);

private:
    virtual_cube::virtual_cube m_cube;
    bool m_virtual_cube_on;
    bool m_safe_activation;

};
}
