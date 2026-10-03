#include "ppbng_storage/manifest_verifier.hpp"

#include "ppbng_storage/session_manifest.hpp"

#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

namespace
{

class TempDirectory
{
public:
  TempDirectory()
  {
    path = std::filesystem::temp_directory_path() /
      ("ppbng_manifest_verify_" + std::to_string(++sequence));
    std::filesystem::create_directory(path);
  }
  ~TempDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }
  std::filesystem::path path;
  static inline std::uint64_t sequence{0U};
};

void write_text(const std::filesystem::path & path, const std::string & value)
{
  std::ofstream output(path, std::ios::binary);
  output.write(value.data(), static_cast<std::streamsize>(value.size()));
}

ppbng_storage::SessionManifest manifest()
{
  ppbng_storage::SessionManifest value;
  value.session_id = "session-id";
  value.user_dataset_name = "field-run";
  value.machine_id = "ppbng-industrial-pc";
  value.configuration_snapshot = "schema_version: 1\n";
  value.created_utc = "20260826T120000Z";
  value.finalized_utc = "20260826T140000Z";
  value.state = ppbng_storage::ManifestState::finalized;
  value.hardware_enabled = true;
  value.simulation = false;
  value.storage.planned_duration_seconds = 7200U;
  value.storage.estimated_bytes_per_second = 1U;
  value.storage.start_available_valid = true;
  value.storage.final_available_valid = true;
  value.storage.actual_dataset_bytes_valid = true;
  for (const std::string role : {
      "fx10e", "swir", "rgb", "thermal", "gnss", "rsm400", "timing", "context"})
  {
    ppbng_storage::DeviceManifest device;
    device.role = role;
    device.backend = "test";
    value.devices.push_back(std::move(device));
  }
  return value;
}

std::string serialize(const ppbng_storage::SessionManifest & value)
{
  const auto result = ppbng_storage::serialize_manifest_json(value);
  EXPECT_TRUE(result.valid) << result.detail;
  return result.json;
}

}  // namespace

TEST(ManifestVerifier, AcceptsFinalizedSchemaThreeHardwareManifest)
{
  TempDirectory temp;
  const auto path = temp.path / "manifest.json";
  write_text(path, serialize(manifest()));
  const auto result = ppbng_storage::verify_session_manifest_file(path);
  EXPECT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.state, "finalized");
  EXPECT_TRUE(result.hardware_enabled);
  EXPECT_FALSE(result.simulation);
  EXPECT_EQ(result.device_count, 8U);
  EXPECT_EQ(result.session_id, "session-id");
}

TEST(ManifestVerifier, RejectsNonterminalManifestAndMissingFinalEvidence)
{
  TempDirectory temp;
  const auto path = temp.path / "manifest.json";
  auto value = manifest();
  value.state = ppbng_storage::ManifestState::recording;
  value.finalized_utc.clear();
  write_text(path, serialize(value));
  EXPECT_FALSE(ppbng_storage::verify_session_manifest_file(path).success);

  value = manifest();
  value.storage.actual_dataset_bytes_valid = false;
  write_text(path, serialize(value));
  const auto missing = ppbng_storage::verify_session_manifest_file(path);
  EXPECT_FALSE(missing.success);
  EXPECT_NE(missing.message.find("dataset byte evidence"), std::string::npos);
}

TEST(ManifestVerifier, RejectsMissingOrDuplicateDeviceRoles)
{
  TempDirectory temp;
  const auto path = temp.path / "manifest.json";
  auto value = manifest();
  value.devices.pop_back();
  write_text(path, serialize(value));
  const auto missing = ppbng_storage::verify_session_manifest_file(path);
  EXPECT_FALSE(missing.success);
  EXPECT_NE(missing.message.find("context"), std::string::npos);

  value = manifest();
  auto duplicate_text = serialize(value);
  const std::string context_role = "\"role\": \"context\"";
  const auto context_position = duplicate_text.find(context_role);
  ASSERT_NE(context_position, std::string::npos);
  duplicate_text.replace(
    context_position, context_role.size(), "\"role\": \"timing\"");
  write_text(path, duplicate_text);
  const auto duplicate = ppbng_storage::verify_session_manifest_file(path);
  EXPECT_FALSE(duplicate.success);
  EXPECT_NE(duplicate.message.find("duplicate"), std::string::npos);
}

TEST(ManifestVerifier, RejectsDuplicateTopLevelKeysEvenWhenJsonSyntaxIsValid)
{
  TempDirectory temp;
  const auto path = temp.path / "manifest.json";
  auto text = serialize(manifest());
  const auto insertion = text.find("\n");
  ASSERT_NE(insertion, std::string::npos);
  text.insert(insertion + 1U, "  \"schema_version\": 3,\n");
  write_text(path, text);
  const auto result = ppbng_storage::verify_session_manifest_file(path);
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("duplicate"), std::string::npos);
}

TEST(ManifestVerifier, SupportsExplicitRoleSetForSimulationDatasets)
{
  TempDirectory temp;
  const auto path = temp.path / "manifest.json";
  auto value = manifest();
  value.hardware_enabled = false;
  value.simulation = true;
  value.machine_id.clear();
  value.configuration_snapshot.clear();
  value.devices.resize(1U);
  value.devices.front().role = "simulated";
  write_text(path, serialize(value));
  const auto result = ppbng_storage::verify_session_manifest_file(path, {"simulated"});
  EXPECT_TRUE(result.success) << result.message;
  EXPECT_TRUE(result.simulation);
}
