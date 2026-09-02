#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>

#include "ppbng_storage/segment_verifier.hpp"

namespace ppbng_storage
{

inline constexpr std::size_t kAssociationPendingHardCap = 4096U;

struct AssociationVerificationResult
{
  bool success{false};
  std::uint64_t expected_samples{0};
  std::uint64_t pending_records{0};
  std::uint64_t terminal_records{0};
  std::uint64_t matched_records{0};
  std::uint64_t degraded_records{0};
  std::string message;
};

// Verifies <stream>_association.ndjson against committed ppbseg samples. The
// complete RGB+thermal segment set must already be structurally valid. Memory
// used for lifecycle state is bounded by maximum_pending_records, which may be
// lowered but never raised above kAssociationPendingHardCap.
AssociationVerificationResult verify_camera_association(
  const std::filesystem::path & session_root,
  const std::string & stream_name,
  std::size_t maximum_pending_records = kAssociationPendingHardCap) noexcept;

// Uses an already checksum-verified stream summary so callers validating a
// complete dataset do not reread very large camera payloads.
AssociationVerificationResult verify_camera_association(
  const std::filesystem::path & session_root,
  const SegmentStreamVerification & stream,
  std::size_t maximum_pending_records = kAssociationPendingHardCap) noexcept;

}  // namespace ppbng_storage
