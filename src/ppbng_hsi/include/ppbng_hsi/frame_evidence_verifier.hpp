#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

namespace ppbng_hsi
{

struct FrameEvidenceVerification
{
  bool success{false};
  std::uint64_t compared_records{0};
  std::string message;
};

// Performs an exact, streaming comparison between frame_context.ndjson and
// the authoritative per-stream association/timestamp sidecars. The source
// files and context records must already have passed their structural/CRC
// verifiers. Memory use is independent of sample count.
FrameEvidenceVerification verify_dataset_frame_evidence(
  const std::filesystem::path & session_root,
  const std::string & session_id) noexcept;

}  // namespace ppbng_hsi
