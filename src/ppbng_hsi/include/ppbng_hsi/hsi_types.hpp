#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace ppbng_hsi
{

enum class CameraKind : std::uint8_t {fx10e = 0, swir = 1};
enum class HsiState : std::uint8_t
{
  disconnected = 0,
  connected = 1,
  configured = 2,
  shutter_closed = 3,
  dark_collecting = 4,
  ready = 5,
  streaming = 6,
  recovering = 7,
  fault = 8,
};
enum class TimeStatus : std::uint8_t {unsynced = 0, holdover = 1, locked = 2};
enum class CaptureKind : std::uint8_t {dark = 0, sample = 1};
// CONSISTENT_UNVERIFIED means the observed counters have a stable delta, but
// that evidence alone cannot detect a missed trigger followed by FIFO shift.
enum class AssociationStatus : std::uint8_t
{
  unverified = 0,
  matched = 1,
  unmatched = 2,
  consistent_unverified = 3,
};
enum class LineStatus : std::uint8_t
{
  produced = 0,
  not_ready = 1,
  invalid_trigger = 2,
  disconnected = 3,
  fault = 4,
};

struct HsiConfig
{
  CameraKind kind{CameraKind::fx10e};
  std::string device_id;
  std::string trigger_channel;
  // "Internal" means software-controlled continuous acquisition: ROS calls
  // Acquisition.Start/Stop while the camera generates line timing internally.
  // "External" means one physical trigger is expected for each acquired line.
  std::string trigger_mode{"External"};
  std::uint32_t spatial_samples{0};
  std::uint32_t spectral_bands{0};
  double line_rate_hz{0.0};
  double exposure_us{0.0};
};

struct TriggerEvent
{
  std::string channel;
  std::uint64_t channel_sequence{0};
  std::uint64_t pps_sequence{0};
  std::uint64_t offset_ticks{0};
  std::uint64_t ticks_per_second{0};
  std::int64_t utc_time_ns{0};
  TimeStatus time_status{TimeStatus::unsynced};
  std::uint64_t uncertainty_ns{0};
};

struct EnviLayout
{
  std::uint32_t samples{0};
  std::uint64_t lines{0};
  std::uint32_t bands{0};
  std::uint32_t header_offset{0};
  std::uint32_t data_type{12};
  std::string interleave{"bil"};
  std::uint8_t byte_order{0};
  std::uint64_t bytes_per_line{0};
  std::string raw_filename;
  std::string header_filename;
  std::string timestamp_filename;
  std::string index_filename;
};

struct LineIndexEntry
{
  std::uint32_t segment_id{0};
  std::uint32_t sdk_segment_id{0};
  CaptureKind capture_kind{CaptureKind::sample};
  std::uint64_t segment_line_index{0};
  std::uint64_t camera_line_sequence{0};
  std::uint64_t trigger_sequence{0};
  std::uint64_t pps_sequence{0};
  std::int64_t utc_time_ns{0};
  TimeStatus time_status{TimeStatus::unsynced};
  std::uint64_t uncertainty_ns{0};
  // Captured at the SDK callback/queue producer boundary, never synthesized by polling.
  std::uint64_t host_receive_monotonic_ns{0};
  std::uint64_t raw_file_offset_bytes{0};
  std::uint64_t payload_size_bytes{0};
  bool sequence_gap_before{false};
  std::uint64_t missing_trigger_count{0};
  AssociationStatus association_status{AssociationStatus::unverified};
  bool association_anchor_valid{false};
  std::int64_t frame_trigger_delta{0};
  std::uint64_t association_anchor_frame{0};
  std::uint64_t association_anchor_trigger{0};
};

struct LineRecord
{
  std::string device_id;
  CameraKind camera_kind{CameraKind::fx10e};
  TriggerEvent trigger;
  LineIndexEntry index;
  std::vector<std::uint16_t> pixels;
};

struct OperationResult
{
  bool success{false};
  std::string message;
};

struct LineResult
{
  LineStatus status{LineStatus::not_ready};
  std::optional<LineRecord> line;
  std::string message;
};

}  // namespace ppbng_hsi
