#pragma once

#include "mios/controller_pipeline/controller_pipeline.hpp"

namespace mios {

class CartVelocityControllerPipeline : public ControllerPipeline{
public:
    CartVelocityControllerPipeline();
    void initialize(const Percept& p_0,const control::ControlRuntimeConfig& config) override;
    control::ArmCommand step(const Percept &p, const Actuator &cmd) override;
    bool is_valid_command(const control::ArmCommand& cmd) const override;
    void update_percept(Percept::Controller &p) override;
    void terminate() override;
    void context_switch(const Percept &p) override;
private:
    control::ArmCommand m_command{};
};

}
