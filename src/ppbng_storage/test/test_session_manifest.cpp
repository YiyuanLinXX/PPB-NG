#include "ppbng_storage/session_manifest.hpp"
#include "ppbng_storage/json_escape.hpp"

#include <gtest/gtest.h>

TEST(JsonEscape, PreservesNdjsonRecordBoundariesAndEscapesControlBytes)
{
  std::string input = "SDK \"fault\"\\path\nnext\t";
  input.push_back(static_cast<char>(0x01));
  EXPECT_EQ(
    ppbng_storage::json_escape(input),
    "SDK \\\"fault\\\"\\\\path\\nnext\\t\\u0001");
}

TEST(SessionManifest, SerializesTraceableSettingsAndEscapesJson)
{
  ppbng_storage::SessionManifest manifest;
  manifest.session_id = "session-1";
  manifest.user_dataset_name = "plot \"west\"\nrow2";
  manifest.created_utc = "20260825T200000Z";
  manifest.simulation = true;
  ppbng_storage::DeviceManifest thermal;
  thermal.role = "thermal";
  thermal.backend = "mock";
  thermal.model = "A6701";
  thermal.requested_settings = {{"transport_height", "513"}, {"image_height", "512"}};
  thermal.actual_settings = {{"payload_bytes", "656640"}};
  thermal.status_observed = true;
  thermal.latest_status_host_monotonic_ns = 123456U;
  thermal.lifecycle_state = 5U;
  thermal.health = 2U;
  thermal.segment_id = 3U;
  thermal.last_sample_valid = true;
  thermal.last_sample_sequence = 42U;
  thermal.samples_received = 42U;
  thermal.samples_incomplete = 1U;
  thermal.samples_lost = 2U;
  thermal.samples_dropped = 3U;
  thermal.reconnect_attempts = 1U;
  thermal.active_config_hash = "sha256:abc";
  thermal.latest_status_detail = "recovered \"once\"";
  manifest.devices.push_back(thermal);

  const auto result = ppbng_storage::serialize_manifest_json(manifest);
  ASSERT_TRUE(result.valid) << result.detail;
  EXPECT_NE(result.json.find("plot \\\"west\\\"\\nrow2"), std::string::npos);
  EXPECT_NE(result.json.find("\"transport_height\": \"513\""), std::string::npos);
  EXPECT_NE(result.json.find("\"payload_bytes\": \"656640\""), std::string::npos);
  EXPECT_NE(result.json.find("\"schema_version\": 3"), std::string::npos);
  EXPECT_NE(result.json.find("\"samples_received\": 42"), std::string::npos);
  EXPECT_NE(result.json.find("\"samples_dropped\": 3"), std::string::npos);
  EXPECT_NE(result.json.find("recovered \\\"once\\\""), std::string::npos);
  EXPECT_NE(result.json.find("\"storage\""), std::string::npos);
}

TEST(SessionManifest, RejectsLegacySchemaWithoutSilentDowngrade)
{
  ppbng_storage::SessionManifest manifest;
  manifest.schema_version = 2U;
  manifest.session_id = "session";
  manifest.user_dataset_name = "plot";
  manifest.created_utc = "now";
  manifest.simulation = true;
  EXPECT_FALSE(ppbng_storage::serialize_manifest_json(manifest).valid);
}

TEST(SessionManifest, RuntimeUpdatesPreserveLatestStatusAndMonotonicCounters)
{
  ppbng_storage::DeviceManifest device;
  ppbng_storage::DeviceRuntimeUpdate first;
  first.status_host_monotonic_ns = 100U;
  first.lifecycle_state = 5U;
  first.health = 1U;
  first.samples_received = 10U;
  first.actual_settings = {{"payload_bytes", "656640"}};
  first.detail = "streaming";
  const auto applied = ppbng_storage::apply_device_runtime_update(device, first);
  EXPECT_TRUE(applied.latest_status_applied);
  EXPECT_TRUE(applied.actual_settings_valid);

  auto stale = first;
  stale.status_host_monotonic_ns = 90U;
  stale.samples_received = 12U;
  stale.actual_settings = {{"payload_bytes", "wrong"}};
  stale.detail = "stale";
  const auto ignored = ppbng_storage::apply_device_runtime_update(device, stale);
  EXPECT_FALSE(ignored.latest_status_applied);
  EXPECT_EQ(device.samples_received, 12U);
  EXPECT_EQ(device.latest_status_detail, "streaming");
  ASSERT_EQ(device.actual_settings.size(), 1U);
  EXPECT_EQ(device.actual_settings[0].second, "656640");

  auto malformed = first;
  malformed.status_host_monotonic_ns = 110U;
  malformed.health = 3U;
  malformed.actual_settings = {{"gain", "1"}, {"gain", "2"}};
  const auto rejected_settings = ppbng_storage::apply_device_runtime_update(device, malformed);
  EXPECT_TRUE(rejected_settings.latest_status_applied);
  EXPECT_FALSE(rejected_settings.actual_settings_valid);
  EXPECT_EQ(device.health, 3U);
  EXPECT_EQ(device.actual_settings.size(), 1U);
}

TEST(SessionManifest, UndatedStatusCannotOverwriteTimestampedEvidence)
{
  ppbng_storage::DeviceManifest device;
  ppbng_storage::DeviceRuntimeUpdate timestamped;
  timestamped.status_host_monotonic_ns = 100U;
  timestamped.lifecycle_state = 5U;
  timestamped.health = 1U;
  timestamped.samples_received = 10U;
  timestamped.actual_settings = {{"payload_bytes", "656640"}};
  timestamped.detail = "verified streaming state";
  ASSERT_TRUE(ppbng_storage::apply_device_runtime_update(device, timestamped).
    latest_status_applied);

  ppbng_storage::DeviceRuntimeUpdate undated;
  undated.status_host_monotonic_ns = 0U;
  undated.lifecycle_state = 0U;
  undated.health = 3U;
  undated.samples_received = 12U;
  undated.actual_settings = {{"payload_bytes", "unverified"}};
  undated.detail = "undated late status";
  const auto result = ppbng_storage::apply_device_runtime_update(device, undated);

  EXPECT_FALSE(result.latest_status_applied);
  EXPECT_EQ(device.latest_status_host_monotonic_ns, 100U);
  EXPECT_EQ(device.lifecycle_state, 5U);
  EXPECT_EQ(device.health, 1U);
  EXPECT_EQ(device.latest_status_detail, "verified streaming state");
  EXPECT_EQ(device.samples_received, 12U);
  ASSERT_EQ(device.actual_settings.size(), 1U);
  EXPECT_EQ(device.actual_settings[0].second, "656640");
}

TEST(SessionManifest, HardwareModeRequiresTraceableStorageAdmissionEvidence)
{
  ppbng_storage::SessionManifest manifest;
  manifest.session_id = "session";
  manifest.user_dataset_name = "plot";
  manifest.created_utc = "now";
  manifest.hardware_enabled = true;
  EXPECT_FALSE(ppbng_storage::serialize_manifest_json(manifest).valid);
  manifest.machine_id = "industrial-pc-1";
  manifest.configuration_snapshot = "machine_config.snapshot.yaml";
  manifest.storage.planned_duration_seconds = 7200U;
  manifest.storage.estimated_bytes_per_second = 162531840U;
  manifest.storage.start_available_valid = true;
  manifest.storage.start_available_bytes = 2'000'000'000'000ULL;
  const auto result = ppbng_storage::serialize_manifest_json(manifest);
  EXPECT_TRUE(result.valid) << result.detail;
  EXPECT_NE(result.json.find("\"configuration_snapshot\": \"machine_config.snapshot.yaml\""),
    std::string::npos);
}

TEST(SessionManifest, RequiresExactlyOneOperatingMode)
{
  ppbng_storage::SessionManifest manifest;
  manifest.session_id = "session";
  manifest.user_dataset_name = "plot";
  manifest.created_utc = "now";
  EXPECT_FALSE(ppbng_storage::serialize_manifest_json(manifest).valid);
  manifest.simulation = true;
  manifest.hardware_enabled = true;
  EXPECT_FALSE(ppbng_storage::serialize_manifest_json(manifest).valid);
}

TEST(SessionManifest, RejectsDuplicateRolesAndSettings)
{
  ppbng_storage::SessionManifest manifest;
  manifest.session_id = "session";
  manifest.user_dataset_name = "plot";
  manifest.created_utc = "now";
  manifest.simulation = true;
  ppbng_storage::DeviceManifest device;
  device.role = "rgb";
  device.backend = "mock";
  device.requested_settings = {{"gain", "1"}, {"gain", "2"}};
  manifest.devices = {device};
  EXPECT_FALSE(ppbng_storage::serialize_manifest_json(manifest).valid);
  device.requested_settings.clear();
  manifest.devices = {device, device};
  EXPECT_FALSE(ppbng_storage::serialize_manifest_json(manifest).valid);
}

TEST(SessionManifest, TerminalStateRequiresFinalizationTime)
{
  ppbng_storage::SessionManifest manifest;
  manifest.session_id = "session";
  manifest.user_dataset_name = "plot";
  manifest.created_utc = "now";
  manifest.simulation = true;
  manifest.state = ppbng_storage::ManifestState::finalized;
  EXPECT_FALSE(ppbng_storage::serialize_manifest_json(manifest).valid);
  manifest.finalized_utc = "later";
  EXPECT_TRUE(ppbng_storage::serialize_manifest_json(manifest).valid);
}
