#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace ppbng_storage
{

inline constexpr std::uint32_t kMaximumQualificationDurationSeconds = 60U;
inline constexpr std::uint32_t kMaximumQualificationBlockMiB = 64U;
inline constexpr std::uint32_t kMaximumQualificationTestMiB = 16U * 1024U;
inline constexpr std::uint64_t kQualificationReserveBytes = 100ULL * 1024ULL * 1024ULL * 1024ULL;

struct DurableWriteQualificationOptions
{
  bool explicitly_confirmed{false};
  bool show_help{false};
  std::filesystem::path output_root;
  std::uint32_t duration_seconds{10U};
  std::uint32_t block_mib{8U};
  std::uint32_t maximum_test_mib{4096U};
};

struct DurableWriteQualificationPlan
{
  std::uint64_t block_bytes{};
  std::uint64_t maximum_test_bytes{};
  std::uint64_t reserve_bytes{kQualificationReserveBytes};
  std::uint64_t required_available_bytes{};
};

struct QualificationValidation
{
  bool valid{false};
  std::string detail;
  DurableWriteQualificationPlan plan;
};

struct QualificationArgumentResult
{
  bool parsed{false};
  std::string detail;
  DurableWriteQualificationOptions options;
};

QualificationArgumentResult parse_durable_write_qualification_arguments(
  const std::vector<std::string> & arguments);
QualificationValidation validate_durable_write_qualification_options(
  const DurableWriteQualificationOptions & options) noexcept;

}  // namespace ppbng_storage
