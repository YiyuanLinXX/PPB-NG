#include "ppbng_thermal/mock_thermal_backend.hpp"

namespace ppbng_thermal
{
MockThermalBackend::MockThermalBackend(FakeThermalNodeMap nodes) : nodes_(std::move(nodes)),
  release_count_(std::make_shared<std::atomic_uint64_t>(0)) {}
Status MockThermalBackend::preflight(const std::chrono::milliseconds timeout, const StopToken & stop) const
{if (stop.stop_requested()) return {ErrorCode::cancelled, "stop requested"}; if (timeout.count() <= 0) return {ErrorCode::timeout, "finite positive timeout required"}; return {};}
Status MockThermalBackend::discover(std::chrono::milliseconds timeout, const StopToken & stop, std::vector<std::string> & ids)
{auto s=preflight(timeout,stop); if(!s.ok())return s; if(!nodes_.discoverable)return {ErrorCode::not_found,"mock absent"}; ids={"mock-a6701"}; state_=LifecycleState::discovered; return {};}
Status MockThermalBackend::open(const std::string & id, std::chrono::milliseconds timeout, const StopToken & stop)
{auto s=preflight(timeout,stop); if(!s.ok())return s; if(state_!=LifecycleState::discovered||id!="mock-a6701")return {ErrorCode::invalid_state,"discover before open"}; if(nodes_.access!=AccessMode::control)return {ErrorCode::access_denied,"control access required"}; state_=LifecycleState::open; return {};}
Status MockThermalBackend::configure(const ThermalConfiguration & c, std::chrono::milliseconds timeout, const StopToken & stop)
{auto s=preflight(timeout,stop); if(!s.ok())return s; if(state_!=LifecycleState::open)return {ErrorCode::invalid_state,"open before configure"}; requested_=c; nodes_.values=c; if(nodes_.force_readback_mismatch)nodes_.values.transport_height=512; ThermalReadback r; s=readback(timeout,stop,r); if(!s.ok())return s; if(configuration_hash(c)!=configuration_hash(r))return {ErrorCode::readback_mismatch,"configuration hash mismatch"}; s=validate_a6701_contract(r); if(!s.ok())return s; state_=LifecycleState::configured; return {};}
Status MockThermalBackend::readback(std::chrono::milliseconds timeout, const StopToken & stop, ThermalReadback & r)
{auto s=preflight(timeout,stop); if(!s.ok())return s; if(state_!=LifecycleState::open&&state_!=LifecycleState::configured&&state_!=LifecycleState::armed&&state_!=LifecycleState::streaming)return {ErrorCode::invalid_state,"device not open"}; static_cast<ThermalConfiguration&>(r)=nodes_.values; r.ready=nodes_.ready; r.fpa_cold=nodes_.fpa_cold; r.access=nodes_.access; return {};}
Status MockThermalBackend::calibration_snapshot(
  std::chrono::milliseconds timeout, const StopToken & stop, ThermalCalibrationSnapshot & snapshot)
{auto s=preflight(timeout,stop);if(!s.ok())return s;if(state_!=LifecycleState::open&&state_!=LifecycleState::configured&&state_!=LifecycleState::armed&&state_!=LifecycleState::streaming)return {ErrorCode::invalid_state,"device not open"};snapshot=nodes_.calibration;return {};}
Status MockThermalBackend::arm(std::chrono::milliseconds timeout, const StopToken & stop)
{auto s=preflight(timeout,stop); if(!s.ok())return s; if(state_!=LifecycleState::configured)return {ErrorCode::invalid_state,"configure before arm"}; ThermalReadback r; s=readback(timeout,stop,r); if(!s.ok())return s; s=validate_a6701_contract(r); if(!s.ok())return s; state_=LifecycleState::armed; return {};}
FrameResult MockThermalBackend::next_frame(std::chrono::milliseconds timeout, const StopToken & stop)
{auto s=preflight(timeout,stop); if(!s.ok())return {s,{}}; if(state_!=LifecycleState::armed&&state_!=LifecycleState::streaming)return {{ErrorCode::invalid_state,"arm before stream"},{}}; state_=LifecycleState::streaming; ++frame_id_; const auto received=std::chrono::steady_clock::now(); const bool complete=nodes_.incomplete_every_n_frames==0||frame_id_%nodes_.incomplete_every_n_frames!=0; auto storage=std::make_shared<std::vector<std::byte>>(nodes_.values.payload_bytes); auto count=release_count_; FrameInfo info{frame_id_,segment_index_,storage->size(),complete,frame_id_*1000000U,received};info.nuc_status_valid=nodes_.nuc_status_valid;info.nuc_active=nodes_.nuc_active;info.correction_auto_in_progress=nodes_.correction_auto_in_progress;info.correction_status=nodes_.correction_status;info.correction_status_text=nodes_.correction_status_text;info.flag_state=nodes_.flag_state;FrameLease lease(info,storage,[count](){++(*count);}); return {complete?Status{}:Status{ErrorCode::incomplete_frame,"mock incomplete frame"},std::move(lease)};}
Status MockThermalBackend::recover(std::chrono::milliseconds timeout, const StopToken & stop)
{auto s=preflight(timeout,stop); if(!s.ok())return s; if(state_!=LifecycleState::faulted&&state_!=LifecycleState::streaming&&state_!=LifecycleState::armed)return {ErrorCode::invalid_state,"recover requires active/fault state"}; ++segment_index_; frame_id_=0; state_=LifecycleState::open; return {};}
Status MockThermalBackend::stop(std::chrono::milliseconds timeout) noexcept
{if(timeout.count()<=0)return {ErrorCode::timeout,"finite positive timeout required"}; state_=LifecycleState::stopped; return {};}
}  // namespace ppbng_thermal
