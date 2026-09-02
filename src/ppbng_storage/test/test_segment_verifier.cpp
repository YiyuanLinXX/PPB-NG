#include "ppbng_storage/segment_verifier.hpp"

#include "ppbng_storage/segment_io.hpp"

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace
{

class TempSession
{
public:
  TempSession()
  {
    root = std::filesystem::temp_directory_path() /
      ("ppbng_segment_verify_" + std::to_string(++sequence));
    std::filesystem::create_directories(root / "segments");
  }
  ~TempSession()
  {
    std::error_code error;
    std::filesystem::remove_all(root, error);
  }
  std::filesystem::path root;
  static inline std::uint64_t sequence{0U};
};

ppbng_storage::FrameEnvelope envelope(const std::uint64_t sample_id)
{
  ppbng_storage::FrameEnvelope value;
  value.sample_id = sample_id;
  value.trigger_id = sample_id + 100U;
  value.pps_sequence = 9U;
  value.utc_nanoseconds = static_cast<std::int64_t>(sample_id * 1'000U);
  value.time_quality = ppbng_storage::TimeQuality::locked;
  return value;
}

std::vector<std::byte> payload(const std::size_t size)
{
  std::vector<std::byte> result(size);
  for (std::size_t index = 0; index < size; ++index) {
    result[index] = static_cast<std::byte>(index & 0xffU);
  }
  return result;
}

void write_segment(
  const std::filesystem::path & root, const std::string & name,
  const std::vector<std::uint64_t> & samples)
{
  auto created = ppbng_storage::SegmentWriter::create(
    root, std::filesystem::path("segments") / name);
  ASSERT_TRUE(created.first.ok()) << created.first.detail;
  const auto bytes = payload(5U);
  for (const auto sample : samples) {
    ASSERT_TRUE(created.second->append(envelope(sample), bytes.data(), bytes.size()).ok());
  }
}

}  // namespace

TEST(SegmentVerifier, ReportsVerifiedCountsAndSampleRange)
{
  TempSession session;
  auto created = ppbng_storage::SegmentWriter::create(
    session.root, "segments/rgb_000000.ppbseg");
  ASSERT_TRUE(created.first.ok());
  const auto first = payload(11U);
  const auto second = payload(17U);
  ASSERT_TRUE(created.second->append(envelope(5U), first.data(), first.size()).ok());
  ASSERT_TRUE(created.second->append(envelope(6U), second.data(), second.size()).ok());
  created.second.reset();

  const auto result = ppbng_storage::verify_segment_file(
    session.root, "segments/rgb_000000.ppbseg");
  EXPECT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.record_count, 2U);
  EXPECT_EQ(result.payload_bytes, 28U);
  EXPECT_EQ(result.first_sample_id, 5U);
  EXPECT_EQ(result.last_sample_id, 6U);
}

TEST(SegmentVerifier, RejectsPayloadCorruptionWithoutChangingFile)
{
  TempSession session;
  const auto relative = std::filesystem::path("segments/thermal_000000.ppbseg");
  auto created = ppbng_storage::SegmentWriter::create(session.root, relative);
  ASSERT_TRUE(created.first.ok());
  const auto bytes = payload(32U);
  ASSERT_TRUE(created.second->append(envelope(1U), bytes.data(), bytes.size()).ok());
  created.second.reset();

  const auto path = session.root / relative;
  const auto original_size = std::filesystem::file_size(path);
  std::fstream stream(path, std::ios::binary | std::ios::in | std::ios::out);
  stream.seekg(16 + 72 + 3);
  char value = 0;
  stream.read(&value, 1);
  stream.clear();
  stream.seekp(16 + 72 + 3);
  value ^= 0x40;
  stream.write(&value, 1);
  stream.close();

  const auto result = ppbng_storage::verify_segment_file(session.root, relative);
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("payload checksum"), std::string::npos);
  EXPECT_EQ(std::filesystem::file_size(path), original_size);
}

TEST(SegmentVerifier, RejectsNonIncreasingOrZeroSampleIds)
{
  TempSession duplicate_session;
  auto duplicate = ppbng_storage::SegmentWriter::create(
    duplicate_session.root, "segments/duplicate.ppbseg");
  ASSERT_TRUE(duplicate.first.ok());
  const auto bytes = payload(4U);
  ASSERT_TRUE(duplicate.second->append(envelope(3U), bytes.data(), bytes.size()).ok());
  ASSERT_TRUE(duplicate.second->append(envelope(3U), bytes.data(), bytes.size()).ok());
  duplicate.second.reset();
  EXPECT_FALSE(ppbng_storage::verify_segment_file(
    duplicate_session.root, "segments/duplicate.ppbseg").success);

  TempSession zero_session;
  auto zero = ppbng_storage::SegmentWriter::create(
    zero_session.root, "segments/zero.ppbseg");
  ASSERT_TRUE(zero.first.ok());
  ASSERT_TRUE(zero.second->append(envelope(0U), bytes.data(), bytes.size()).ok());
  zero.second.reset();
  EXPECT_FALSE(ppbng_storage::verify_segment_file(
    zero_session.root, "segments/zero.ppbseg").success);
}

TEST(SegmentVerifier, RejectsUnsafePathAndAcceptsHeaderOnlySegment)
{
  TempSession session;
  auto created = ppbng_storage::SegmentWriter::create(
    session.root, "segments/empty.ppbseg");
  ASSERT_TRUE(created.first.ok());
  created.second.reset();
  const auto empty = ppbng_storage::verify_segment_file(
    session.root, "segments/empty.ppbseg");
  EXPECT_TRUE(empty.success) << empty.message;
  EXPECT_EQ(empty.record_count, 0U);

  const auto unsafe = ppbng_storage::verify_segment_file(session.root, "../outside.ppbseg");
  EXPECT_FALSE(unsafe.success);
  EXPECT_NE(unsafe.message.find("beneath segments"), std::string::npos);
}

TEST(SegmentVerifier, RejectsSampleGapWithinOneFile)
{
  TempSession session;
  write_segment(session.root, "rgb_000000.ppbseg", {1U, 3U});
  const auto result = ppbng_storage::verify_segment_file(
    session.root, "segments/rgb_000000.ppbseg");
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("contiguous"), std::string::npos);
}

TEST(SegmentSetVerifier, AcceptsTwoCompleteContiguousStreams)
{
  TempSession session;
  write_segment(session.root, "rgb_000000.ppbseg", {1U, 2U});
  write_segment(session.root, "rgb_000001.ppbseg", {3U, 4U});
  write_segment(session.root, "thermal_000000.ppbseg", {1U});
  write_segment(session.root, "thermal_000001.ppbseg", {2U, 3U});
  const auto result = ppbng_storage::verify_segment_set(session.root);
  ASSERT_TRUE(result.success) << result.message;
  ASSERT_EQ(result.streams.size(), 2U);
  EXPECT_EQ(result.streams[0].stream_name, "rgb");
  EXPECT_EQ(result.streams[0].segment_count, 2U);
  EXPECT_EQ(result.streams[0].record_count, 4U);
  EXPECT_EQ(result.streams[1].stream_name, "thermal");
  EXPECT_EQ(result.streams[1].last_sample_id, 3U);
}

TEST(SegmentSetVerifier, RejectsMissingStreamSegmentGapAndSampleGap)
{
  TempSession missing;
  write_segment(missing.root, "rgb_000000.ppbseg", {1U});
  EXPECT_NE(ppbng_storage::verify_segment_set(missing.root).message.find("thermal"),
    std::string::npos);

  TempSession segment_gap;
  write_segment(segment_gap.root, "rgb_000000.ppbseg", {1U});
  write_segment(segment_gap.root, "rgb_000002.ppbseg", {2U});
  const auto gap = ppbng_storage::verify_segment_set(segment_gap.root, {"rgb"});
  EXPECT_FALSE(gap.success);
  EXPECT_NE(gap.message.find("segment index"), std::string::npos);

  TempSession sample_gap;
  write_segment(sample_gap.root, "rgb_000000.ppbseg", {1U});
  write_segment(sample_gap.root, "rgb_000001.ppbseg", {3U});
  const auto samples = ppbng_storage::verify_segment_set(sample_gap.root, {"rgb"});
  EXPECT_FALSE(samples.success);
  EXPECT_NE(samples.message.find("sample discontinuity"), std::string::npos);
}

TEST(SegmentSetVerifier, RejectsEmptyMiddleMalformedAndUnexpectedStreams)
{
  TempSession empty_middle;
  write_segment(empty_middle.root, "rgb_000000.ppbseg", {1U});
  write_segment(empty_middle.root, "rgb_000001.ppbseg", {});
  write_segment(empty_middle.root, "rgb_000002.ppbseg", {2U});
  const auto empty = ppbng_storage::verify_segment_set(empty_middle.root, {"rgb"});
  EXPECT_FALSE(empty.success);
  EXPECT_NE(empty.message.find("empty non-final"), std::string::npos);

  TempSession malformed;
  write_segment(malformed.root, "rgb_1.ppbseg", {1U});
  EXPECT_FALSE(ppbng_storage::verify_segment_set(malformed.root, {"rgb"}).success);

  TempSession unexpected;
  write_segment(unexpected.root, "rgb_000000.ppbseg", {1U});
  write_segment(unexpected.root, "other_000000.ppbseg", {1U});
  const auto extra = ppbng_storage::verify_segment_set(unexpected.root, {"rgb"});
  EXPECT_FALSE(extra.success);
  EXPECT_NE(extra.message.find("unexpected"), std::string::npos);
}
