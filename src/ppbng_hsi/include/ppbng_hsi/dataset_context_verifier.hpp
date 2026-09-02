#pragma once

#include "ppbng_storage/segment_verifier.hpp"

#include <cstdint>
#include <filesystem>
#include <string>

namespace ppbng_hsi
{

struct DatasetContextVerification
{
  bool success{false};
  std::uint64_t expected_records{0};
  std::uint64_t context_records{0};
  std::uint64_t unavailable_gnss_records{0};
  std::uint64_t unavailable_rsm_records{0};
  std::string message;
};

// Cross-checks frame_context.ndjson against every committed RGB/thermal
// ppbseg sample and every HSI index row whose capture_kind is sample. Inputs
// are expected to have passed their format/CRC verifiers first.
DatasetContextVerification verify_dataset_frame_context(
  const std::filesystem::path & session_root,
  const std::string & session_id,
  const ppbng_storage::SegmentSetVerificationResult & segment_set) noexcept;

}  // namespace ppbng_hsi
