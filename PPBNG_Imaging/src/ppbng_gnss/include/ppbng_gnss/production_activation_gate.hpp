#pragma once

#include "ppbng_gnss/um982_receiver.hpp"

#include <string>

namespace ppbng_gnss
{

enum class ProductionGateState {inert, armed, started, fault};

struct ProductionGateResult
{
  bool accepted{false};
  std::string detail;
};

[[nodiscard]] ProductionGateResult validate_receive_only_configuration(
  const SerialReceiveConfiguration & configuration);

class ProductionActivationGate
{
public:
  explicit ProductionActivationGate(bool hardware_enabled) noexcept;
  ProductionGateResult arm(const SerialReceiveConfiguration & configuration);
  ProductionGateResult start();
  void start_failed() noexcept;
  void stop() noexcept;
  [[nodiscard]] ProductionGateState state() const noexcept;

private:
  bool hardware_enabled_{false};
  ProductionGateState state_{ProductionGateState::inert};
};

}  // namespace ppbng_gnss
