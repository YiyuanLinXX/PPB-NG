#pragma once

#include <string>

namespace ppbng_rsm400
{

enum class ProductionGateState {inert, armed, started, fault};

struct GateResult
{
  bool accepted{false};
  std::string detail;
};

class ProductionActivationGate
{
public:
  explicit ProductionActivationGate(bool hardware_enabled) noexcept;
  GateResult arm(const std::string & com_port);
  GateResult start();
  void start_failed() noexcept;
  void stop() noexcept;
  [[nodiscard]] ProductionGateState state() const noexcept;

private:
  bool hardware_enabled_{false};
  ProductionGateState state_{ProductionGateState::inert};
};

}  // namespace ppbng_rsm400
