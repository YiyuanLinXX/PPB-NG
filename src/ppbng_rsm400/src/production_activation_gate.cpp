#include "ppbng_rsm400/production_activation_gate.hpp"

namespace ppbng_rsm400
{

ProductionActivationGate::ProductionActivationGate(const bool hardware_enabled) noexcept
: hardware_enabled_(hardware_enabled) {}

GateResult ProductionActivationGate::arm(const std::string & com_port)
{
  if (!hardware_enabled_) {
    return {false, "hardware_enabled is false; RSM400 remains inert"};
  }
  if (com_port.empty() || com_port.find("REQUIRED") != std::string::npos) {
    return {false, "a non-placeholder RSM400 COM port is required"};
  }
  if (state_ == ProductionGateState::started) {
    return {false, "cannot arm while the RSM400 port is started"};
  }
  state_ = ProductionGateState::armed;
  return {true, "RSM400 configuration armed; port remains closed"};
}

GateResult ProductionActivationGate::start()
{
  if (state_ != ProductionGateState::armed) {
    return {false, "RSM400 start requires a successful arm"};
  }
  state_ = ProductionGateState::started;
  return {true, "RSM400 port open authorized"};
}

void ProductionActivationGate::start_failed() noexcept {state_ = ProductionGateState::fault;}
void ProductionActivationGate::stop() noexcept {state_ = ProductionGateState::inert;}
ProductionGateState ProductionActivationGate::state() const noexcept {return state_;}

}  // namespace ppbng_rsm400
