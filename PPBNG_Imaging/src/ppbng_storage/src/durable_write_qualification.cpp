#include "ppbng_storage/durable_write_qualification.hpp"

#include <charconv>
#include <limits>
#include <set>
#include <string_view>

namespace ppbng_storage
{
namespace
{
bool parse_u32(const std::string & text, std::uint32_t & value) noexcept
{
  if (text.empty()) {return false;}
  std::uint32_t parsed{};
  const auto result = std::from_chars(text.data(), text.data() + text.size(), parsed);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {return false;}
  value = parsed;
  return true;
}
}  // namespace

QualificationArgumentResult parse_durable_write_qualification_arguments(
  const std::vector<std::string> & arguments)
{
  QualificationArgumentResult result;
  std::set<std::string> seen;
  for (std::size_t index = 0U; index < arguments.size(); ++index) {
    const auto & argument = arguments[index];
    if (argument == "--help" || argument == "-h") {
      if (!seen.insert("help").second) {return {false, "duplicate --help", {}};}
      result.options.show_help = true;
      continue;
    }
    if (argument == "--confirm-durable-write-qualification") {
      if (!seen.insert("confirm").second) {return {false, "duplicate confirmation switch", {}};}
      result.options.explicitly_confirmed = true;
      continue;
    }
    const bool has_value = argument == "--output-root" || argument == "--duration-seconds" ||
      argument == "--block-mib" || argument == "--maximum-test-mib";
    if (!has_value) {return {false, "unknown argument: " + argument, {}};}
    if (!seen.insert(argument).second) {return {false, "duplicate argument: " + argument, {}};}
    if (++index >= arguments.size()) {return {false, "missing value for " + argument, {}};}
    const auto & value = arguments[index];
    if (argument == "--output-root") {
      if (value.empty()) {return {false, "--output-root must not be empty", {}};}
      result.options.output_root = std::filesystem::u8path(value);
      continue;
    }
    std::uint32_t parsed{};
    if (!parse_u32(value, parsed)) {return {false, "invalid unsigned integer for " + argument, {}};}
    if (argument == "--duration-seconds") {result.options.duration_seconds = parsed;}
    else if (argument == "--block-mib") {result.options.block_mib = parsed;}
    else {result.options.maximum_test_mib = parsed;}
  }
  result.parsed = true;
  return result;
}

QualificationValidation validate_durable_write_qualification_options(
  const DurableWriteQualificationOptions & options) noexcept
{
  if (options.show_help) {return {false, "help does not authorize a write", {}};}
  if (!options.explicitly_confirmed) {
    return {false, "explicit --confirm-durable-write-qualification is required", {}};
  }
  if (options.output_root.empty()) {return {false, "output_root is required", {}};}
  if (options.duration_seconds == 0U ||
    options.duration_seconds > kMaximumQualificationDurationSeconds)
  {
    return {false, "duration_seconds must be within 1..60", {}};
  }
  if (options.block_mib == 0U || options.block_mib > kMaximumQualificationBlockMiB) {
    return {false, "block_mib must be within 1..64", {}};
  }
  if (options.maximum_test_mib == 0U ||
    options.maximum_test_mib > kMaximumQualificationTestMiB)
  {
    return {false, "maximum_test_mib must be within 1..16384", {}};
  }
  if (options.maximum_test_mib < options.block_mib) {
    return {false, "maximum_test_mib must be at least one block", {}};
  }
  constexpr std::uint64_t mib = 1024ULL * 1024ULL;
  DurableWriteQualificationPlan plan;
  plan.block_bytes = static_cast<std::uint64_t>(options.block_mib) * mib;
  plan.maximum_test_bytes = static_cast<std::uint64_t>(options.maximum_test_mib) * mib;
  if (plan.maximum_test_bytes > (std::numeric_limits<std::uint64_t>::max)() -
    plan.reserve_bytes)
  {
    return {false, "capacity requirement overflow", {}};
  }
  plan.required_available_bytes = plan.maximum_test_bytes + plan.reserve_bytes;
  return {true, "qualification parameters are bounded and explicitly authorized", plan};
}

}  // namespace ppbng_storage
