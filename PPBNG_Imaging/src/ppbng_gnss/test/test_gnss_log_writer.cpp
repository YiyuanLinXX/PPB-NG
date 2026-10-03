#include "ppbng_gnss/gnss_log_writer.hpp"
#include "ppbng_gnss/um982_protocol.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

namespace
{
constexpr char kGga[] =
  "$GNGGA,123519.00,4250.000000,N,07630.000000,W,4,20,0.7,100.000,M,-30.000,M,0.8,0001*64";

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    path = std::filesystem::temp_directory_path() /
      ("ppbng-gnss-writer-" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directory(path);
  }
  ~TemporaryDirectory() {std::error_code ec; std::filesystem::remove_all(path, ec);}
  std::filesystem::path path;
};

ppbng_gnss::ReceivedSentence gga_sentence()
{
  ppbng_gnss::ReceivedSentence value;
  value.raw_line = kGga;
  value.host_receive_time = std::chrono::system_clock::time_point{} +
    std::chrono::duration_cast<std::chrono::system_clock::duration>(std::chrono::nanoseconds(123000));
  value.kind = ppbng_gnss::SentenceKind::gga;
  value.gga = ppbng_gnss::parse_gga(kGga);
  return value;
}

std::string read_text(const std::filesystem::path & path)
{
  std::ifstream stream(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}
}  // namespace

TEST(GnssLogWriter, AppendsRawTimingEpochParseStatusAndStructuredGga)
{
  TemporaryDirectory temp;
  ppbng_gnss::GnssLogWriterOptions options{temp.path, "um982", 1U};
  auto [created, writer] = ppbng_gnss::GnssLogWriter::create(options);
  ASSERT_TRUE(created.success) << created.detail;
  ASSERT_TRUE(writer->append(gga_sentence(), 67890U, 3U).success);
  ASSERT_TRUE(writer->close().success);
  const auto raw = read_text(temp.path / "um982.sentences.jsonl");
  EXPECT_NE(raw.find("\"host_system_ns\":123000"), std::string::npos);
  EXPECT_NE(raw.find("\"host_monotonic_ns\":67890"), std::string::npos);
  EXPECT_NE(raw.find("\"connection_epoch\":3"), std::string::npos);
  EXPECT_NE(raw.find("\"parse_status\":\"gga_valid\""), std::string::npos);
  const auto structured = read_text(temp.path / "um982.gga.jsonl");
  EXPECT_NE(structured.find("\"quality\":4"), std::string::npos);
  EXPECT_NE(structured.find("\"latitude_deg\""), std::string::npos);
}

TEST(GnssLogWriter, PreservesParseErrorRawSentence)
{
  TemporaryDirectory temp;
  auto [created, writer] = ppbng_gnss::GnssLogWriter::create({temp.path, "um982", 10U});
  ASSERT_TRUE(created.success);
  ppbng_gnss::ReceivedSentence sentence;
  sentence.raw_line = "$GNGGA,bad*00";
  sentence.host_receive_time = std::chrono::system_clock::now();
  sentence.kind = ppbng_gnss::SentenceKind::parse_error;
  sentence.parse_error = "checksum mismatch";
  ASSERT_TRUE(writer->append(sentence, 11U, 2U).success);
  ASSERT_TRUE(writer->close().success);
  const auto raw = read_text(temp.path / "um982.sentences.jsonl");
  EXPECT_NE(raw.find("parse_error"), std::string::npos);
  EXPECT_NE(raw.find("checksum mismatch"), std::string::npos);
  EXPECT_NE(raw.find("$GNGGA,bad*00"), std::string::npos);
}

TEST(GnssLogWriter, EscapesArbitrarySerialBytesAsValidAsciiJson)
{
  TemporaryDirectory temp;
  auto [created, writer] = ppbng_gnss::GnssLogWriter::create({temp.path, "um982", 10U});
  ASSERT_TRUE(created.success);
  ppbng_gnss::ReceivedSentence sentence;
  sentence.raw_line = std::string("partial-") + static_cast<char>(0xff) + "-line";
  sentence.host_receive_time = std::chrono::system_clock::now();
  sentence.kind = ppbng_gnss::SentenceKind::other;
  ASSERT_TRUE(writer->append(sentence, 11U, 1U).success);
  ASSERT_TRUE(writer->close().success);
  const auto raw = read_text(temp.path / "um982.sentences.jsonl");
  EXPECT_NE(raw.find("partial-\\u00ff-line"), std::string::npos);
  EXPECT_EQ(raw.find(static_cast<char>(0xff)), std::string::npos);
}

TEST(GnssLogWriter, ExclusiveCreateRejectsDuplicateAndDoesNotAppendTruncatedTail)
{
  TemporaryDirectory temp;
  const auto raw_path = temp.path / "um982.sentences.jsonl";
  {std::ofstream truncated(raw_path); truncated << "partial";}
  const auto initial = std::filesystem::file_size(raw_path);
  auto [created, writer] = ppbng_gnss::GnssLogWriter::create({temp.path, "um982", 10U});
  EXPECT_FALSE(created.success);
  EXPECT_EQ(writer, nullptr);
  EXPECT_EQ(std::filesystem::file_size(raw_path), initial);
}

TEST(GnssLogWriter, ReportsInjectedFailureWithoutPartialSecondRecord)
{
  TemporaryDirectory temp;
  ppbng_gnss::GnssLogWriterOptions options{temp.path, "um982", 10U, 1U};
  auto [created, writer] = ppbng_gnss::GnssLogWriter::create(options);
  ASSERT_TRUE(created.success);
  ASSERT_TRUE(writer->append(gga_sentence(), 1U, 1U).success);
  EXPECT_FALSE(writer->append(gga_sentence(), 2U, 1U).success);
  ASSERT_TRUE(writer->close().success);
  const auto raw = read_text(temp.path / "um982.sentences.jsonl");
  EXPECT_EQ(std::count(raw.begin(), raw.end(), '\n'), 1);
}

TEST(GnssLogWriter, RecoverySegmentExclusiveCreateAfterFinalizingPreviousSegment)
{
  TemporaryDirectory temp;
  auto [first_created, first] = ppbng_gnss::GnssLogWriter::create({temp.path, "um982", 10U});
  ASSERT_TRUE(first_created.success);
  ASSERT_TRUE(first->append(gga_sentence(), 1U, 1U).success);
  ASSERT_TRUE(first->close().success);
  first.reset();

  auto [next_created, next] = ppbng_gnss::GnssLogWriter::create(
    {temp.path, "um982_segment_1", 10U});
  ASSERT_TRUE(next_created.success);
  ASSERT_TRUE(next->append(gga_sentence(), 2U, 2U).success);
  ASSERT_TRUE(next->close().success);
  EXPECT_TRUE(std::filesystem::exists(temp.path / "um982.sentences.jsonl"));
  EXPECT_TRUE(std::filesystem::exists(temp.path / "um982_segment_1.sentences.jsonl"));
  EXPECT_NE(read_text(temp.path / "um982_segment_1.sentences.jsonl").find(
    "\"connection_epoch\":2"), std::string::npos);
}
