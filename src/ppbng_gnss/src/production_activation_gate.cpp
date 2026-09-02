#include "ppbng_gnss/production_activation_gate.hpp"

#include <algorithm>
#include <string_view>

namespace ppbng_gnss
{

ProductionGateResult validate_receive_only_configuration(
  const SerialReceiveConfiguration & configuration)
{
  constexpr std::string_view prefix{"\\\\.\\"};
  std::string_view port = configuration.com_path;
  if (port.rfind(prefix, 0U) == 0U) {
    port.remove_prefix(prefix.size());
  }
  if (port.size() <= 3U || port.substr(0U, 3U) != "COM" ||
    !std::all_of(port.begin() + 3, port.end(), [](const char value) {
      return value >= '0' && value <= '9';
    }))
  {
    return {false, "com_path must be an explicit COMn or \\\\.\\COMn path"};
  }
  if (configuration.baud_rate == 0U || configuration.read_chunk_bytes == 0U ||
    configuration.max_line_bytes == 0U ||
    configuration.read_chunk_bytes > configuration.max_line_bytes)
  {
    return {false, "baud rate and bounded receive buffers must be explicitly configured"};
  }
  return {true, "receive-only configuration is structurally valid"};
}

ProductionActivationGate::ProductionActivationGate(const bool hardware_enabled) noexcept
: hardware_enabled_(hardware_enabled) {}

ProductionGateResult ProductionActivationGate::arm(
  const SerialReceiveConfiguration & configuration)
{
  if (!hardware_enabled_) {
    return {false, "hardware_enabled is false"};
  }
  const auto valid = validate_receive_only_configuration(configuration);
  if (!valid.accepted) {
    return valid;
  }
  if (state_ != ProductionGateState::inert) {
    return {false, "arm is accepted only from inert state"};
  }
  state_ = ProductionGateState::armed;
  return {true, "armed without opening the COM port"};
}

ProductionGateResult ProductionActivationGate::start()
{
  if (state_ != ProductionGateState::armed) {
    return {false, "start requires a prior successful arm"};
  }
  state_ = ProductionGateState::started;
  return {true, "receive-only COM open authorized"};
}

void ProductionActivationGate::start_failed() noexcept
{
  if (state_ == ProductionGateState::started) {state_ = ProductionGateState::fault;}
}

void ProductionActivationGate::stop() noexcept {state_ = ProductionGateState::inert;}
ProductionGateState ProductionActivationGate::state() const noexcept {return state_;}

}  // namespace ppbng_gnss
