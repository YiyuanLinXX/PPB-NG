#include "ppbng_storage/durable_write_qualification.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace
{
using ppbng_storage::DurableWriteQualificationOptions;

TEST(DurableWriteQualification, RefusesEveryImplicitOrIncompleteInvocation)
{
  EXPECT_FALSE(ppbng_storage::validate_durable_write_qualification_options({}).valid);
  DurableWriteQualificationOptions options;
  options.output_root = "D:/data";
  EXPECT_FALSE(ppbng_storage::validate_durable_write_qualification_options(options).valid);
  options.explicitly_confirmed = true;
  EXPECT_TRUE(ppbng_storage::validate_durable_write_qualification_options(options).valid);
}

TEST(DurableWriteQualification, EnforcesHardDurationBlockAndTotalCaps)
{
  DurableWriteQualificationOptions options;
  options.explicitly_confirmed = true;
  options.output_root = "D:/data";
  options.duration_seconds = 61U;
  EXPECT_FALSE(ppbng_storage::validate_durable_write_qualification_options(options).valid);
  options.duration_seconds = 10U;
  options.block_mib = 65U;
  EXPECT_FALSE(ppbng_storage::validate_durable_write_qualification_options(options).valid);
  options.block_mib = 8U;
  options.maximum_test_mib = 16385U;
  EXPECT_FALSE(ppbng_storage::validate_durable_write_qualification_options(options).valid);
  options.maximum_test_mib = 4U;
  EXPECT_FALSE(ppbng_storage::validate_durable_write_qualification_options(options).valid);
}

TEST(DurableWriteQualification, CapacityPlanAlwaysIncludesFixedHundredGibReserve)
{
  DurableWriteQualificationOptions options;
  options.explicitly_confirmed = true;
  options.output_root = "D:/data";
  options.block_mib = 4U;
  options.maximum_test_mib = 1024U;
  const auto result = ppbng_storage::validate_durable_write_qualification_options(options);
  ASSERT_TRUE(result.valid) << result.detail;
  EXPECT_EQ(result.plan.block_bytes, 4ULL * 1024ULL * 1024ULL);
  EXPECT_EQ(result.plan.maximum_test_bytes, 1024ULL * 1024ULL * 1024ULL);
  EXPECT_EQ(result.plan.reserve_bytes, 100ULL * 1024ULL * 1024ULL * 1024ULL);
  EXPECT_EQ(result.plan.required_available_bytes,
    result.plan.maximum_test_bytes + result.plan.reserve_bytes);
}

TEST(DurableWriteQualification, ParserRejectsUnknownDuplicateAndMissingValues)
{
  EXPECT_FALSE(ppbng_storage::parse_durable_write_qualification_arguments(
    {"--surprise"}).parsed);
  EXPECT_FALSE(ppbng_storage::parse_durable_write_qualification_arguments(
    {"--output-root"}).parsed);
  EXPECT_FALSE(ppbng_storage::parse_durable_write_qualification_arguments(
    {"--block-mib", "8", "--block-mib", "16"}).parsed);
  EXPECT_FALSE(ppbng_storage::parse_durable_write_qualification_arguments(
    {"--duration-seconds", "-1"}).parsed);
}

TEST(DurableWriteQualification, ParserAcceptsOnlyExplicitBoundedForm)
{
  const auto parsed = ppbng_storage::parse_durable_write_qualification_arguments({
    "--confirm-durable-write-qualification", "--output-root", "D:/PPBNG_DATA",
    "--duration-seconds", "20", "--block-mib", "16", "--maximum-test-mib", "8192"});
  ASSERT_TRUE(parsed.parsed) << parsed.detail;
  EXPECT_TRUE(parsed.options.explicitly_confirmed);
  EXPECT_EQ(parsed.options.output_root.u8string(), "D:/PPBNG_DATA");
  EXPECT_EQ(parsed.options.duration_seconds, 20U);
  EXPECT_EQ(parsed.options.block_mib, 16U);
  EXPECT_EQ(parsed.options.maximum_test_mib, 8192U);
  EXPECT_TRUE(ppbng_storage::validate_durable_write_qualification_options(parsed.options).valid);
}

TEST(DurableWriteQualification, HelpNeverCountsAsWriteAuthorization)
{
  const auto parsed = ppbng_storage::parse_durable_write_qualification_arguments({"--help"});
  ASSERT_TRUE(parsed.parsed);
  EXPECT_TRUE(parsed.options.show_help);
  EXPECT_FALSE(ppbng_storage::validate_durable_write_qualification_options(parsed.options).valid);
}
}  // namespace
