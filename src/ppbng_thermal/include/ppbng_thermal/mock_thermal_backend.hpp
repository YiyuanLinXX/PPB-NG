#pragma once
#include "ppbng_thermal/thermal_adapter.hpp"

namespace ppbng_thermal
{
struct FakeThermalNodeMap
{
  bool discoverable{true};
  bool ready{true};
  bool fpa_cold{true};
  AccessMode access{AccessMode::control};
  bool force_readback_mismatch{false};
  std::uint64_t incomplete_every_n_frames{0};
  ThermalConfiguration values{};
};

class MockThermalBackend final : public IThermalBackend
{
public:
  explicit MockThermalBackend(FakeThermalNodeMap nodes = {});
  Status discover(std::chrono::milliseconds, const StopToken &, std::vector<std::string> &) override;
  Status open(const std::string &, std::chrono::milliseconds, const StopToken &) override;
  Status configure(const ThermalConfiguration &, std::chrono::milliseconds, const StopToken &) override;
  Status readback(std::chrono::milliseconds, const StopToken &, ThermalReadback &) override;
  Status arm(std::chrono::milliseconds, const StopToken &) override;
  FrameResult next_frame(std::chrono::milliseconds, const StopToken &) override;
  Status recover(std::chrono::milliseconds, const StopToken &) override;
  Status stop(std::chrono::milliseconds) noexcept override;
  LifecycleState state() const noexcept override {return state_;}
  std::uint64_t release_count() const noexcept {return release_count_->load();}
  std::uint64_t segment_index() const noexcept {return segment_index_;}
  FakeThermalNodeMap & nodes() noexcept {return nodes_;}
private:
  Status preflight(std::chrono::milliseconds, const StopToken &) const;
  FakeThermalNodeMap nodes_;
  LifecycleState state_{LifecycleState::idle};
  ThermalConfiguration requested_{};
  std::uint64_t frame_id_{0};
  std::uint64_t segment_index_{0};
  std::shared_ptr<std::atomic_uint64_t> release_count_;
};
}  // namespace ppbng_thermal
