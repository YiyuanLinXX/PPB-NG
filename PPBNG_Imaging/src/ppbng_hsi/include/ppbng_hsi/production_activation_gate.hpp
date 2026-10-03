#pragma once

#include <string>

namespace ppbng_hsi
{

enum class ProductionGateState {inert, armed, started, fault};

struct ProductionGateResult
{
  bool accepted{false};
  std::string detail;
};

class ProductionActivationGate
{
public:
  explicit ProductionActivationGate(bool hardware_enabled) noexcept;
  ProductionGateResult arm(bool configuration_valid);
  ProductionGateResult start();
  void start_failed() noexcept;
  void stop() noexcept;
  [[nodiscard]] ProductionGateState state() const noexcept;

private:
  bool hardware_enabled_{false};
  ProductionGateState state_{ProductionGateState::inert};
};

}  // namespace ppbng_hsi
