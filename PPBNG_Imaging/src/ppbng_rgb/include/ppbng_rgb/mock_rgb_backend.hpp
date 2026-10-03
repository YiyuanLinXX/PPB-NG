#pragma once
#include "ppbng_rgb/rgb_adapter.hpp"
namespace ppbng_rgb
{
struct FakeRgbNodeMap {bool discoverable{true}; bool control_access{true}; bool force_readback_mismatch{false}; std::uint64_t incomplete_every_n_frames{0}; RgbConfiguration values{};};
class MockRgbBackend final:public IRgbBackend
{
public:
  explicit MockRgbBackend(FakeRgbNodeMap={});
  Status discover(std::chrono::milliseconds,const StopToken&,std::vector<std::string>&)override;
  Status open(const std::string&,std::chrono::milliseconds,const StopToken&)override;
  Status configure(const RgbConfiguration&,std::chrono::milliseconds,const StopToken&)override;
  Status readback(std::chrono::milliseconds,const StopToken&,RgbReadback&)override;
  Status arm(std::chrono::milliseconds,const StopToken&)override;
  FrameResult next_frame(std::chrono::milliseconds,const StopToken&)override;
  Status recover(std::chrono::milliseconds,const StopToken&)override;
  Status stop(std::chrono::milliseconds)noexcept override;
  LifecycleState state()const noexcept override{return state_;}
  std::uint64_t release_count()const noexcept{return release_count_->load();} std::uint64_t segment_index()const noexcept{return segment_index_;}
private:
  Status preflight(std::chrono::milliseconds,const StopToken&)const;
  FakeRgbNodeMap nodes_; LifecycleState state_{LifecycleState::idle}; RgbConfiguration requested_{}; std::uint64_t frame_id_{0},segment_index_{0}; std::shared_ptr<std::atomic_uint64_t> release_count_;
};
} // namespace ppbng_rgb
