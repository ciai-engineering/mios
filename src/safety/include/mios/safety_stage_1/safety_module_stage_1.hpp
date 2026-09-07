#pragma once

#include "mios/data_structures/actuator.hpp"
#include "mios/data_structures/percept.hpp"
#include "mios/control/control_runtime_config.hpp"

namespace mios {

class SafetyModuleStage1{
public:
    virtual ~SafetyModuleStage1(){}
    virtual void initialize(const Percept& p_0,
                            const control::ControlRuntimeConfig& config) = 0;
    virtual void step(const Percept& p,Actuator& cmd) = 0;
    virtual void terminate() = 0;

};

}
