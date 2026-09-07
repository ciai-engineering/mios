#pragma once

#include "mios/data_structures/percept.hpp"
#include "mios/data_structures/actuator.hpp"
#include "mios/control/control_types.hpp"
#include "mios/control/control_runtime_config.hpp"

namespace mios {

class ControllerPipeline{
public:
    virtual ~ControllerPipeline(){}
    virtual void initialize(const Percept& p_0,
                            const control::ControlRuntimeConfig& config) = 0;
    // Pipelines implement the original MIOS control algorithms, but do not
    // construct a libfranka command. The backend owns that final conversion.
    virtual control::ArmCommand step(const Percept& p, const Actuator& cmd) = 0;
    virtual bool is_valid_command(const control::ArmCommand& cmd) const = 0;
    virtual void terminate() = 0;
    virtual void update_percept(Percept::Controller& p) = 0;
    virtual void context_switch(const Percept& p) = 0;
};

}
