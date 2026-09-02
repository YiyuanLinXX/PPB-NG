#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace ppbng_storage
{

struct ManifestVerificationResult
{
  bool success{false};
  std::string state;
  bool simulation{false};
  bool hardware_enabled{false};
  std::uint64_t device_count{0};
  std::string session_id;
  std::string message;
};

// Verifies the schema-3 session manifest written by SessionManifest. This is a
// semantic check in addition to strict JSON syntax validation. Duplicate object
// keys are rejected and every expected production role must appear exactly once.
ManifestVerificationResult verify_session_manifest_file(
  const std::filesystem::path & path,
  const std::vector<std::string> & expected_roles = {
    "fx10e", "swir", "rgb", "thermal", "gnss", "rsm400", "timing", "context"}) noexcept;

}  // namespace ppbng_storage
