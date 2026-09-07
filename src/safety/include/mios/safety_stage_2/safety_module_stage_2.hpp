#pragma once

#include "mios/data_structures/actuator.hpp"
#include "mios/data_structures/percept.hpp"
#include "mios/control/control_types.hpp"
#include "mios/control/control_runtime_config.hpp"

namespace mios {

class SafetyModuleStage2{
public:
    virtual ~SafetyModuleStage2(){}
    virtual void initialize(const Percept& p_0,
                            const control::ControlRuntimeConfig& config) = 0;
    virtual void step(const Percept& p, control::ArmCommand& cmd) = 0;
    virtual void terminate() = 0;

};

}
