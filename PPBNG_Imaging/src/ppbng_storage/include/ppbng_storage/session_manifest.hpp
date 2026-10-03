#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace ppbng_storage
{

enum class ManifestState : std::uint8_t {created, recording, finalized, faulted};

struct DeviceManifest
{
  std::string role;
  std::string backend;
  std::string model;
  std::string identity;
  std::string sdk_version;
  std::string firmware_version;
  std::vector<std::pair<std::string, std::string>> requested_settings;
  std::vector<std::pair<std::string, std::string>> actual_settings;
  bool status_observed{false};
  std::uint64_t latest_status_host_monotonic_ns{};
  std::uint8_t lifecycle_state{};
  std::uint8_t health{};
  std::uint32_t segment_id{};
  bool last_sample_valid{false};
  std::uint64_t last_sample_sequence{};
  std::uint64_t samples_received{};
  std::uint64_t samples_incomplete{};
  std::uint64_t samples_lost{};
  std::uint64_t samples_dropped{};
  std::uint32_t reconnect_attempts{};
  std::string active_config_hash;
  std::string latest_status_detail;
};

struct StorageManifest
{
  std::uint64_t planned_duration_seconds{};
  std::uint64_t estimated_bytes_per_second{};
  std::uint32_t capacity_headroom_basis_points{};
  std::uint64_t minimum_reserve_bytes{};
  bool start_available_valid{false};
  std::uint64_t start_available_bytes{};
  bool final_available_valid{false};
  std::uint64_t final_available_bytes{};
  bool actual_dataset_bytes_valid{false};
  std::uint64_t actual_dataset_bytes{};
  bool throughput_qualified{false};
  std::uint64_t qualified_durable_bytes_per_second{};
  std::string throughput_qualification_detail;
  std::string stop_reason;
};

struct DeviceRuntimeUpdate
{
  std::uint64_t status_host_monotonic_ns{};
  std::uint8_t lifecycle_state{};
  std::uint8_t health{};
  std::uint32_t segment_id{};
  bool last_sample_valid{false};
  std::uint64_t last_sample_sequence{};
  std::uint64_t samples_received{};
  std::uint64_t samples_incomplete{};
  std::uint64_t samples_lost{};
  std::uint64_t samples_dropped{};
  std::uint32_t reconnect_attempts{};
  std::string active_config_hash;
  std::vector<std::pair<std::string, std::string>> actual_settings;
  std::string detail;
};

struct DeviceRuntimeUpdateResult
{
  bool latest_status_applied{false};
  bool actual_settings_valid{true};
  std::string detail;
};

struct SessionManifest
{
  std::uint32_t schema_version{3};
  std::string session_id;
  std::string user_dataset_name;
  std::string machine_id;
  std::string configuration_snapshot;
  std::string created_utc;
  std::string finalized_utc;
  ManifestState state{ManifestState::created};
  bool simulation{false};
  bool hardware_enabled{false};
  std::vector<DeviceManifest> devices;
  std::vector<std::string> warnings;
  StorageManifest storage;
};

struct ManifestResult
{
  bool valid{false};
  std::string json;
  std::string detail;
};

ManifestResult serialize_manifest_json(const SessionManifest & manifest);
DeviceRuntimeUpdateResult apply_device_runtime_update(
  DeviceManifest & device, const DeviceRuntimeUpdate & update);

}  // namespace ppbng_storage
