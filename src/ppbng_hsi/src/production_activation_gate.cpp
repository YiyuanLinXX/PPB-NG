#include "ppbng_hsi/production_activation_gate.hpp"

namespace ppbng_hsi
{

ProductionActivationGate::ProductionActivationGate(const bool hardware_enabled) noexcept
: hardware_enabled_(hardware_enabled) {}

ProductionGateResult ProductionActivationGate::arm(const bool configuration_valid)
{
  if (!hardware_enabled_) {
    return {false, "hardware_enabled is false"};
  }
  if (!configuration_valid) {
    return {false, "hardware configuration is incomplete or invalid"};
  }
  if (state_ != ProductionGateState::inert) {
    return {false, "arm is accepted only from inert state"};
  }
  state_ = ProductionGateState::armed;
  return {true, "armed without opening hardware"};
}

ProductionGateResult ProductionActivationGate::start()
{
  if (state_ != ProductionGateState::armed) {
    return {false, "start requires a prior successful arm"};
  }
  state_ = ProductionGateState::started;
  return {true, "start authorized"};
}

void ProductionActivationGate::start_failed() noexcept
{
  if (state_ == ProductionGateState::started) {
    state_ = ProductionGateState::fault;
  }
}

void ProductionActivationGate::stop() noexcept {state_ = ProductionGateState::inert;}
ProductionGateState ProductionActivationGate::state() const noexcept {return state_;}

}  // namespace ppbng_hsi
