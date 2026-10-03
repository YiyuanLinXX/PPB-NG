#pragma once
#include "ppbng_core/frame_context_association.hpp"
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
namespace ppbng_core {
struct ContextFrameIdentity {
  std::string session_id;
  std::string device_id;
  std::uint32_t segment_id{};
  std::uint64_t sample_sequence{};
  std::int64_t utc_ns{};
  std::uint64_t host_ns{};
  std::string trigger_channel;
  std::uint64_t trigger_sequence{};
  std::uint64_t pps_sequence{};
  std::uint64_t controller_tick{};
  std::uint64_t controller_ticks_per_second{};
  std::uint64_t time_uncertainty_ns{};
  std::uint8_t time_status{};
  std::string time_detail;
  bool camera_frame_id_valid{};
  std::uint64_t camera_frame_id{};
  bool camera_timestamp_valid{};
  std::uint64_t camera_timestamp{};
  bool thermal_nuc_state_valid{};
  bool thermal_nuc_active{};
  std::uint64_t thermal_nuc_state_sequence{};
  std::uint64_t thermal_nuc_observed_host_monotonic_ns{};
  bool thermal_correction_auto_in_progress{};
  std::string thermal_correction_status;
  std::string thermal_flag_state;
  std::string thermal_nuc_detail;
};
struct ContextWriterOptions {std::filesystem::path session_root;std::string stem{"frame_context"};std::uint64_t fail_after_records{~std::uint64_t{0}};};
struct ContextWriterResult {bool ok{};std::string detail;};
class FrameContextWriter {public:static std::pair<ContextWriterResult,std::unique_ptr<FrameContextWriter>>create(ContextWriterOptions);~FrameContextWriter();ContextWriterResult append(const ContextFrameIdentity&,const FrameContextResult&);ContextWriterResult flush();ContextWriterResult close();private:explicit FrameContextWriter(ContextWriterOptions o):options_(std::move(o)){}ContextWriterResult open();ContextWriterOptions options_;std::ofstream stream_;std::uint64_t records_{};bool closed_{};};
}
