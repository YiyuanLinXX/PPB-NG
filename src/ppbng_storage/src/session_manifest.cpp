#include "ppbng_storage/session_manifest.hpp"

#include <algorithm>
#include <iomanip>
#include <set>
#include <sstream>

namespace ppbng_storage
{
namespace
{

std::string escape_json(const std::string & input)
{
  std::ostringstream output;
  for (const unsigned char value : input) {
    switch (value) {
      case '"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (value < 0x20U) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0') <<
            static_cast<unsigned int>(value) << std::dec;
        } else {
          output << static_cast<char>(value);
        }
    }
  }
  return output.str();
}

const char * state_name(const ManifestState state) noexcept
{
  switch (state) {
    case ManifestState::created: return "created";
    case ManifestState::recording: return "recording";
    case ManifestState::finalized: return "finalized";
    case ManifestState::faulted: return "faulted";
  }
  return "invalid";
}

bool valid_settings(
  const std::vector<std::pair<std::string, std::string>> & settings,
  std::string & detail)
{
  std::set<std::string> keys;
  for (const auto & setting : settings) {
    if (setting.first.empty()) {
      detail = "device setting key is empty";
      return false;
    }
    if (!keys.insert(setting.first).second) {
      detail = "duplicate device setting key: " + setting.first;
      return false;
    }
  }
  return true;
}

void write_settings(
  std::ostringstream & output,
  const std::vector<std::pair<std::string, std::string>> & settings,
  const std::string & indent)
{
  output << "{";
  if (!settings.empty()) {
    output << "\n";
    for (std::size_t index = 0; index < settings.size(); ++index) {
      output << indent << "  \"" << escape_json(settings[index].first) << "\": \"" <<
        escape_json(settings[index].second) << "\"";
      output << (index + 1U == settings.size() ? "\n" : ",\n");
    }
    output << indent;
  }
  output << "}";
}

}  // namespace

DeviceRuntimeUpdateResult apply_device_runtime_update(
  DeviceManifest & device, const DeviceRuntimeUpdate & update)
{
  std::string settings_error;
  const bool settings_valid = valid_settings(update.actual_settings, settings_error);
  // An undated status may establish initial state, or follow another undated status in
  // arrival order, but it must never overwrite evidence carrying a real monotonic timestamp.
  const bool ordered = !device.status_observed || device.latest_status_host_monotonic_ns == 0U ||
    (update.status_host_monotonic_ns != 0U &&
    update.status_host_monotonic_ns >= device.latest_status_host_monotonic_ns);
  if (ordered) {
    device.status_observed = true;
    if (update.status_host_monotonic_ns != 0U) {
      device.latest_status_host_monotonic_ns = update.status_host_monotonic_ns;
    }
    device.lifecycle_state = update.lifecycle_state;
    device.health = update.health;
    device.last_sample_valid = update.last_sample_valid;
    device.latest_status_detail = update.detail;
    if (!update.active_config_hash.empty()) {
      device.active_config_hash = update.active_config_hash;
    }
    if (settings_valid) {
      for (const auto & setting : update.actual_settings) {
        const auto found = std::find_if(device.actual_settings.begin(), device.actual_settings.end(),
          [&setting](const auto & item) {return item.first == setting.first;});
        if (found == device.actual_settings.end()) {device.actual_settings.push_back(setting);}
        else {found->second = setting.second;}
      }
    }
  }
  device.segment_id = (std::max)(device.segment_id, update.segment_id);
  device.last_sample_sequence = (std::max)(
    device.last_sample_sequence, update.last_sample_sequence);
  device.samples_received = (std::max)(device.samples_received, update.samples_received);
  device.samples_incomplete = (std::max)(device.samples_incomplete, update.samples_incomplete);
  device.samples_lost = (std::max)(device.samples_lost, update.samples_lost);
  device.samples_dropped = (std::max)(device.samples_dropped, update.samples_dropped);
  device.reconnect_attempts = (std::max)(
    device.reconnect_attempts, update.reconnect_attempts);
  return {ordered, settings_valid, settings_valid ?
    (ordered ? "latest status and monotonic counters applied" :
    "stale status ignored; monotonic counters retained") : settings_error};
}

ManifestResult serialize_manifest_json(const SessionManifest & manifest)
{
  if (manifest.schema_version != 3U) {
    return {false, {}, "unsupported manifest schema version"};
  }
  if (manifest.session_id.empty() || manifest.user_dataset_name.empty() ||
    manifest.created_utc.empty())
  {
    return {false, {}, "session ID, dataset name, and creation time are required"};
  }
  if (manifest.hardware_enabled == manifest.simulation) {
    return {false, {}, "exactly one of simulation and hardware_enabled must be true"};
  }
  if (manifest.hardware_enabled &&
    (manifest.machine_id.empty() || manifest.configuration_snapshot.empty()))
  {
    return {false, {}, "hardware manifest requires machine identity and configuration snapshot"};
  }
  if ((manifest.state == ManifestState::finalized || manifest.state == ManifestState::faulted) &&
    manifest.finalized_utc.empty())
  {
    return {false, {}, "terminal manifest requires finalized_utc"};
  }
  if (manifest.hardware_enabled &&
    (manifest.storage.planned_duration_seconds == 0U ||
    manifest.storage.estimated_bytes_per_second == 0U ||
    !manifest.storage.start_available_valid))
  {
    return {false, {}, "hardware manifest requires a storage plan and start capacity evidence"};
  }

  std::set<std::string> roles;
  for (const auto & device : manifest.devices) {
    if (device.role.empty() || device.backend.empty()) {
      return {false, {}, "device role and backend are required"};
    }
    if (!roles.insert(device.role).second) {
      return {false, {}, "duplicate device role: " + device.role};
    }
    std::string detail;
    if (!valid_settings(device.requested_settings, detail) ||
      !valid_settings(device.actual_settings, detail))
    {
      return {false, {}, detail};
    }
  }

  std::ostringstream output;
  output << "{\n";
  output << "  \"schema_version\": " << manifest.schema_version << ",\n";
  output << "  \"session_id\": \"" << escape_json(manifest.session_id) << "\",\n";
  output << "  \"user_dataset_name\": \"" << escape_json(manifest.user_dataset_name) << "\",\n";
  output << "  \"machine_id\": \"" << escape_json(manifest.machine_id) << "\",\n";
  output << "  \"configuration_snapshot\": \"" <<
    escape_json(manifest.configuration_snapshot) << "\",\n";
  output << "  \"created_utc\": \"" << escape_json(manifest.created_utc) << "\",\n";
  output << "  \"finalized_utc\": \"" << escape_json(manifest.finalized_utc) << "\",\n";
  output << "  \"state\": \"" << state_name(manifest.state) << "\",\n";
  output << "  \"simulation\": " << (manifest.simulation ? "true" : "false") << ",\n";
  output << "  \"hardware_enabled\": " << (manifest.hardware_enabled ? "true" : "false") << ",\n";
  output << "  \"devices\": [";
  if (!manifest.devices.empty()) {
    output << "\n";
  }
  for (std::size_t index = 0; index < manifest.devices.size(); ++index) {
    const auto & device = manifest.devices[index];
    output << "    {\n";
    output << "      \"role\": \"" << escape_json(device.role) << "\",\n";
    output << "      \"backend\": \"" << escape_json(device.backend) << "\",\n";
    output << "      \"model\": \"" << escape_json(device.model) << "\",\n";
    output << "      \"identity\": \"" << escape_json(device.identity) << "\",\n";
    output << "      \"sdk_version\": \"" << escape_json(device.sdk_version) << "\",\n";
    output << "      \"firmware_version\": \"" << escape_json(device.firmware_version) << "\",\n";
    output << "      \"requested_settings\": ";
    write_settings(output, device.requested_settings, "      ");
    output << ",\n      \"actual_settings\": ";
    write_settings(output, device.actual_settings, "      ");
    output << ",\n      \"runtime\": {\n";
    output << "        \"status_observed\": " <<
      (device.status_observed ? "true" : "false") << ",\n";
    output << "        \"latest_status_host_monotonic_ns\": " <<
      device.latest_status_host_monotonic_ns << ",\n";
    output << "        \"lifecycle_state\": " <<
      static_cast<unsigned int>(device.lifecycle_state) << ",\n";
    output << "        \"health\": " << static_cast<unsigned int>(device.health) << ",\n";
    output << "        \"segment_id\": " << device.segment_id << ",\n";
    output << "        \"last_sample_valid\": " <<
      (device.last_sample_valid ? "true" : "false") << ",\n";
    output << "        \"last_sample_sequence\": " << device.last_sample_sequence << ",\n";
    output << "        \"samples_received\": " << device.samples_received << ",\n";
    output << "        \"samples_incomplete\": " << device.samples_incomplete << ",\n";
    output << "        \"samples_lost\": " << device.samples_lost << ",\n";
    output << "        \"samples_dropped\": " << device.samples_dropped << ",\n";
    output << "        \"reconnect_attempts\": " << device.reconnect_attempts << ",\n";
    output << "        \"active_config_hash\": \"" <<
      escape_json(device.active_config_hash) << "\",\n";
    output << "        \"latest_status_detail\": \"" <<
      escape_json(device.latest_status_detail) << "\"\n";
    output << "      }";
    output << "\n    }" << (index + 1U == manifest.devices.size() ? "\n" : ",\n");
  }
  output << "  ],\n";
  output << "  \"storage\": {\n";
  output << "    \"planned_duration_seconds\": " <<
    manifest.storage.planned_duration_seconds << ",\n";
  output << "    \"estimated_bytes_per_second\": " <<
    manifest.storage.estimated_bytes_per_second << ",\n";
  output << "    \"capacity_headroom_basis_points\": " <<
    manifest.storage.capacity_headroom_basis_points << ",\n";
  output << "    \"minimum_reserve_bytes\": " <<
    manifest.storage.minimum_reserve_bytes << ",\n";
  output << "    \"start_available_valid\": " <<
    (manifest.storage.start_available_valid ? "true" : "false") << ",\n";
  output << "    \"start_available_bytes\": " << manifest.storage.start_available_bytes << ",\n";
  output << "    \"final_available_valid\": " <<
    (manifest.storage.final_available_valid ? "true" : "false") << ",\n";
  output << "    \"final_available_bytes\": " << manifest.storage.final_available_bytes << ",\n";
  output << "    \"actual_dataset_bytes_valid\": " <<
    (manifest.storage.actual_dataset_bytes_valid ? "true" : "false") << ",\n";
  output << "    \"actual_dataset_bytes\": " << manifest.storage.actual_dataset_bytes << ",\n";
  output << "    \"throughput_qualified\": " <<
    (manifest.storage.throughput_qualified ? "true" : "false") << ",\n";
  output << "    \"qualified_durable_bytes_per_second\": " <<
    manifest.storage.qualified_durable_bytes_per_second << ",\n";
  output << "    \"throughput_qualification_detail\": \"" <<
    escape_json(manifest.storage.throughput_qualification_detail) << "\",\n";
  output << "    \"stop_reason\": \"" << escape_json(manifest.storage.stop_reason) << "\"\n";
  output << "  },\n";
  output << "  \"warnings\": [";
  for (std::size_t index = 0; index < manifest.warnings.size(); ++index) {
    output << (index == 0 ? "" : ", ") << "\"" << escape_json(manifest.warnings[index]) << "\"";
  }
  output << "]\n}\n";
  return {true, output.str(), {}};
}

}  // namespace ppbng_storage
