#include "ppbng_hsi/envi_segment_writer.hpp"
#include "ppbng_hsi/envi_part_verifier.hpp"
#include "ppbng_hsi/pending_trigger_matcher.hpp"
#include "ppbng_storage/segment_io.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

namespace
{
class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    path = std::filesystem::temp_directory_path() /
      ("ppbng-hsi-writer-" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directory(path);
  }
  ~TemporaryDirectory() {std::error_code ec; std::filesystem::remove_all(path, ec);}
  std::filesystem::path path;
};

ppbng_hsi::HsiConfig config()
{
  ppbng_hsi::HsiConfig value;
  value.kind = ppbng_hsi::CameraKind::fx10e;
  value.device_id = "fx-test";
  value.trigger_channel = "fx-trigger";
  value.spatial_samples = 2U;
  value.spectral_bands = 2U;
  value.line_rate_hz = 120.0;
  value.exposure_us = 1000.0;
  return value;
}

ppbng_hsi::LineRecord line(std::uint32_t segment, std::uint64_t sequence,
  ppbng_hsi::CaptureKind capture = ppbng_hsi::CaptureKind::sample)
{
  ppbng_hsi::LineRecord value;
  value.device_id = "fx-test";
  value.camera_kind = ppbng_hsi::CameraKind::fx10e;
  value.index.segment_id = segment;
  value.index.segment_line_index = sequence - 1U;
  value.index.capture_kind = capture;
  value.index.camera_line_sequence = sequence;
  value.index.trigger_sequence = sequence + 100U;
  value.index.pps_sequence = 7U;
  value.trigger.channel = "fx-trigger";
  value.trigger.channel_sequence = value.index.trigger_sequence;
  value.trigger.pps_sequence = value.index.pps_sequence;
  value.trigger.offset_ticks = 250U + sequence;
  value.trigger.ticks_per_second = 1'000'000U;
  value.index.utc_time_ns = 1'700'000'000'000'000'000LL + static_cast<std::int64_t>(sequence);
  value.index.time_status = ppbng_hsi::TimeStatus::locked;
  value.index.uncertainty_ns = 50U;
  value.index.host_receive_monotonic_ns = 987654321U + sequence;
  value.pixels = {1U, 2U, 3U, 4U};
  return value;
}

std::string read_text(const std::filesystem::path & path)
{
  std::ifstream stream(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

class MatcherWriterAdapter final : public ppbng_hsi::IHsiAdapter
{
public:
  ppbng_hsi::CameraKind kind() const noexcept override {return config_.kind;}
  ppbng_hsi::HsiState state() const noexcept override {return ppbng_hsi::HsiState::streaming;}
  const ppbng_hsi::HsiConfig & config() const noexcept override {return config_;}
  std::uint32_t segment_id() const noexcept override {return sdk_segment;}
  ppbng_hsi::OperationResult connect() override {return {true, {}};}
  ppbng_hsi::OperationResult configure(const ppbng_hsi::HsiConfig &) override {return {true, {}};}
  ppbng_hsi::OperationResult close_shutter() override {return {true, {}};}
  ppbng_hsi::OperationResult begin_dark_capture(std::size_t) override {return {true, {}};}
  ppbng_hsi::OperationResult open_shutter() override {return {true, {}};}
  ppbng_hsi::OperationResult start_streaming() override {return {true, {}};}
  ppbng_hsi::OperationResult stop_streaming() override {return {true, {}};}
  ppbng_hsi::OperationResult recover() override {return {true, {}};}

  ppbng_hsi::LineResult on_trigger(const ppbng_hsi::TriggerEvent & trigger) override
  {
    ppbng_hsi::LineRecord result;
    result.device_id = config_.device_id;
    result.camera_kind = config_.kind;
    result.trigger = trigger;
    result.index.segment_id = sdk_segment;
    result.index.capture_kind = ppbng_hsi::CaptureKind::sample;
    result.index.segment_line_index = adapter_line++;
    result.index.camera_line_sequence = camera_sequence;
    result.index.trigger_sequence = trigger.channel_sequence;
    result.index.pps_sequence = trigger.pps_sequence;
    result.index.utc_time_ns = trigger.utc_time_ns;
    result.index.time_status = trigger.time_status;
    result.index.uncertainty_ns = trigger.uncertainty_ns;
    result.index.host_receive_monotonic_ns = 1'000U + camera_sequence;
    if (previous_camera != 0U && camera_sequence > previous_camera + 1U) {
      result.index.sequence_gap_before = true;
      result.index.missing_trigger_count = camera_sequence - previous_camera - 1U;
    }
    previous_camera = camera_sequence;
    result.pixels = {1U, 2U, 3U, 4U};
    return {ppbng_hsi::LineStatus::produced, std::move(result), "produced"};
  }

  ppbng_hsi::HsiConfig config_{::config()};
  std::uint32_t sdk_segment{7U};
  std::uint64_t adapter_line{40U};
  std::uint64_t camera_sequence{100U};
  std::uint64_t previous_camera{0U};
};

ppbng_hsi::TriggerEvent matcher_trigger(const std::uint64_t sequence)
{
  ppbng_hsi::TriggerEvent result;
  result.channel = "fx-trigger";
  result.channel_sequence = sequence;
  result.pps_sequence = 9U;
  result.offset_ticks = sequence;
  result.ticks_per_second = 1'000'000U;
  result.utc_time_ns = 1'700'000'000'000'000'000LL + static_cast<std::int64_t>(sequence);
  result.time_status = ppbng_hsi::TimeStatus::locked;
  result.uncertainty_ns = 25U;
  return result;
}
}  // namespace

TEST(EnviSegmentWriter, PersistsEnviRawHeaderAndExplicitDarkSampleIndex)
{
  TemporaryDirectory temp;
  ppbng_hsi::EnviWriterOptions options{temp.path, "fx10e", 1024U, 1U};
  auto [created, writer] = ppbng_hsi::EnviSegmentWriter::create(options, config());
  ASSERT_TRUE(created.success) << created.message;
  ASSERT_TRUE(writer->append(line(0U, 1U, ppbng_hsi::CaptureKind::dark)).success);
  auto matched = line(0U, 2U);
  matched.index.association_status = ppbng_hsi::AssociationStatus::matched;
  matched.index.association_anchor_valid = true;
  matched.index.frame_trigger_delta = 50;
  matched.index.association_anchor_frame = 100U;
  matched.index.association_anchor_trigger = 50U;
  ASSERT_TRUE(writer->append(matched).success);
  ASSERT_TRUE(writer->close().success);
  EXPECT_EQ(std::filesystem::file_size(temp.path / "fx10e_segment_0_part_0.raw"), 16U);
  const auto header = read_text(temp.path / "fx10e_segment_0_part_0.hdr");
  EXPECT_NE(header.find("lines = 2"), std::string::npos);
  EXPECT_NE(header.find("interleave = bil"), std::string::npos);
  const auto index = read_text(temp.path / "fx10e_segment_0_part_0.index.csv");
  const std::uint16_t first_payload[] = {1U, 2U, 3U, 4U};
  const auto first_crc = ppbng_storage::payload_crc32(
    reinterpret_cast<const std::byte *>(first_payload), sizeof(first_payload));
  EXPECT_NE(index.find("payload_bytes,payload_crc32"), std::string::npos);
  EXPECT_NE(index.find(",8," + std::to_string(first_crc) + ","), std::string::npos);
  EXPECT_NE(index.find(",dark,"), std::string::npos);
  EXPECT_NE(index.find(",sample,"), std::string::npos);
  EXPECT_NE(index.find(",MATCHED,1,50,100,50"), std::string::npos);
  const auto timestamps = read_text(temp.path / "fx10e_segment_0_part_0.timestamps.csv");
  EXPECT_NE(timestamps.find("association_status"), std::string::npos);
  EXPECT_NE(timestamps.find("host_receive_monotonic_ns"), std::string::npos);
  EXPECT_NE(timestamps.find("controller_tick,controller_ticks_per_second"), std::string::npos);
  EXPECT_NE(timestamps.find("987654323"), std::string::npos);
  EXPECT_NE(timestamps.find(",252,1000000,"), std::string::npos);
  EXPECT_NE(timestamps.find(",MATCHED,1,50,100,50"), std::string::npos);
}

TEST(EnviPartVerifier, AcceptsAbsoluteControllerTickForUnsyncedNoPpsLine)
{
  TemporaryDirectory temp;
  auto [created, writer] = ppbng_hsi::EnviSegmentWriter::create(
    {temp.path, "fx10e", 1024U, 1U}, config());
  ASSERT_TRUE(created.success) << created.message;
  auto value = line(0U, 1U);
  value.index.pps_sequence = 0U;
  value.trigger.pps_sequence = 0U;
  value.trigger.offset_ticks = value.trigger.ticks_per_second * 7U + 123U;
  value.index.utc_time_ns = 0;
  value.index.time_status = ppbng_hsi::TimeStatus::unsynced;
  ASSERT_TRUE(writer->append(value).success);
  ASSERT_TRUE(writer->close().success);
  const auto result = ppbng_hsi::verify_envi_part(
    temp.path, "fx10e_segment_0_part_0");
  EXPECT_TRUE(result.success) << result.message;
}

TEST(EnviPartVerifier, AcceptsInternalContinuousLineWithoutInventedTriggerOrUtc)
{
  TemporaryDirectory temp;
  auto internal_config = config();
  internal_config.trigger_mode = "Internal";
  auto [created, writer] = ppbng_hsi::EnviSegmentWriter::create(
    {temp.path, "fx10e", 1024U, 1U}, internal_config);
  ASSERT_TRUE(created.success) << created.message;
  auto value = line(0U, 1U);
  value.trigger.channel = "internal:fx-test";
  value.trigger.channel_sequence = 0U;
  value.trigger.pps_sequence = 0U;
  value.trigger.offset_ticks = 0U;
  value.trigger.ticks_per_second = 0U;
  value.index.trigger_sequence = 0U;
  value.index.pps_sequence = 0U;
  value.index.utc_time_ns = 0;
  value.index.time_status = ppbng_hsi::TimeStatus::unsynced;
  value.index.uncertainty_ns = 0U;
  value.index.association_status = ppbng_hsi::AssociationStatus::unverified;
  ASSERT_TRUE(writer->append(value).success);
  ASSERT_TRUE(writer->close().success);
  const auto result = ppbng_hsi::verify_envi_part(
    temp.path, "fx10e_segment_0_part_0");
  EXPECT_TRUE(result.success) << result.message;
}

TEST(EnviSegmentWriter, RollsOverBySizeAndOnSourceSegmentRecovery)
{
  TemporaryDirectory temp;
  ppbng_hsi::EnviWriterOptions options{temp.path, "fx10e", 8U, 100U};
  auto [created, writer] = ppbng_hsi::EnviSegmentWriter::create(options, config());
  ASSERT_TRUE(created.success);
  ASSERT_TRUE(writer->append(line(3U, 1U)).success);
  ASSERT_TRUE(writer->append(line(3U, 2U)).success);
  EXPECT_EQ(writer->current_part(), 1U);
  ASSERT_TRUE(writer->append(line(4U, 3U)).success);
  EXPECT_EQ(writer->current_source_segment(), 4U);
  EXPECT_EQ(writer->current_part(), 0U);
  ASSERT_TRUE(writer->close().success);
  EXPECT_TRUE(std::filesystem::exists(temp.path / "fx10e_segment_3_part_0.raw"));
  EXPECT_TRUE(std::filesystem::exists(temp.path / "fx10e_segment_3_part_1.raw"));
  EXPECT_TRUE(std::filesystem::exists(temp.path / "fx10e_segment_4_part_0.raw"));
}

TEST(EnviSegmentWriter, RejectsDuplicateOrTruncatedExistingPartWithoutOverwrite)
{
  TemporaryDirectory temp;
  const auto raw = temp.path / "fx10e_segment_0_part_0.raw";
  {std::ofstream truncated(raw, std::ios::binary); truncated << "tail";}
  const auto original_size = std::filesystem::file_size(raw);
  ppbng_hsi::EnviWriterOptions options{temp.path, "fx10e", 1024U, 100U};
  auto [created, writer] = ppbng_hsi::EnviSegmentWriter::create(options, config());
  ASSERT_TRUE(created.success);
  EXPECT_FALSE(writer->append(line(0U, 1U)).success);
  EXPECT_EQ(std::filesystem::file_size(raw), original_size);
}

TEST(EnviSegmentWriter, ReportsInjectedWriteFailureBeforeAppendingTail)
{
  TemporaryDirectory temp;
  ppbng_hsi::EnviWriterOptions options{temp.path, "fx10e", 1024U, 100U, 8U};
  auto [created, writer] = ppbng_hsi::EnviSegmentWriter::create(options, config());
  ASSERT_TRUE(created.success);
  ASSERT_TRUE(writer->append(line(0U, 1U)).success);
  EXPECT_FALSE(writer->append(line(0U, 2U)).success);
  ASSERT_TRUE(writer->close().success);
  EXPECT_EQ(std::filesystem::file_size(temp.path / "fx10e_segment_0_part_0.raw"), 8U);
}

TEST(EnviSegmentWriter, ExplicitRecoveryBoundaryFinalizesSidecarsBeforeNewSegment)
{
  TemporaryDirectory temp;
  ppbng_hsi::EnviWriterOptions options{temp.path, "fx10e", 1024U, 100U};
  auto [created, writer] = ppbng_hsi::EnviSegmentWriter::create(options, config());
  ASSERT_TRUE(created.success);
  ASSERT_TRUE(writer->append(line(2U, 1U)).success);
  ASSERT_TRUE(writer->finalize_current_segment().success);
  EXPECT_TRUE(std::filesystem::exists(temp.path / "fx10e_segment_2_part_0.hdr"));
  EXPECT_NE(read_text(temp.path / "fx10e_segment_2_part_0.hdr").find("lines = 1"),
    std::string::npos);

  ASSERT_TRUE(writer->append(line(3U, 2U)).success);
  ASSERT_TRUE(writer->close().success);
  EXPECT_TRUE(std::filesystem::exists(temp.path / "fx10e_segment_3_part_0.raw"));
  EXPECT_TRUE(std::filesystem::exists(temp.path / "fx10e_segment_3_part_0.index.csv"));
  EXPECT_TRUE(std::filesystem::exists(temp.path / "fx10e_segment_3_part_0.timestamps.csv"));
}

TEST(EnviPartVerifier, AcceptsWriterOutputAndReportsExactExtent)
{
  TemporaryDirectory temp;
  ppbng_hsi::EnviWriterOptions options{temp.path, "fx10e", 1024U, 1U};
  auto [created, writer] = ppbng_hsi::EnviSegmentWriter::create(options, config());
  ASSERT_TRUE(created.success);
  ASSERT_TRUE(writer->append(line(0U, 1U)).success);
  ASSERT_TRUE(writer->append(line(0U, 2U)).success);
  ASSERT_TRUE(writer->close().success);
  const auto result = ppbng_hsi::verify_envi_part(temp.path, "fx10e_segment_0_part_0");
  EXPECT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.line_count, 2U);
  EXPECT_EQ(result.payload_bytes, 16U);
}

TEST(EnviPartVerifier, DetectsPayloadCorruption)
{
  TemporaryDirectory temp;
  ppbng_hsi::EnviWriterOptions options{temp.path, "fx10e", 1024U, 1U};
  auto [created, writer] = ppbng_hsi::EnviSegmentWriter::create(options, config());
  ASSERT_TRUE(created.success);
  ASSERT_TRUE(writer->append(line(0U, 1U)).success);
  ASSERT_TRUE(writer->close().success);
  std::fstream raw(temp.path / "fx10e_segment_0_part_0.raw",
    std::ios::binary | std::ios::in | std::ios::out);
  char byte{};
  raw.read(&byte, 1);
  byte ^= 0x5A;
  raw.seekp(0);
  raw.write(&byte, 1);
  raw.close();
  const auto result = ppbng_hsi::verify_envi_part(temp.path, "fx10e_segment_0_part_0");
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("CRC32 mismatch"), std::string::npos);
}

TEST(EnviPartVerifier, QuickModeChecksStructureAndSampledFirstIntervalAndLastCrc)
{
  TemporaryDirectory temp;
  ppbng_hsi::EnviWriterOptions writer_options{temp.path, "fx10e", 1024U, 1U};
  auto [created, writer] = ppbng_hsi::EnviSegmentWriter::create(writer_options, config());
  ASSERT_TRUE(created.success);
  for (std::uint64_t sequence = 1U; sequence <= 5U; ++sequence) {
    ASSERT_TRUE(writer->append(line(0U, sequence)).success);
  }
  ASSERT_TRUE(writer->close().success);

  std::uint64_t progress_calls = 0U;
  ppbng_hsi::EnviVerificationOptions options;
  options.full_payload_crc = false;
  options.quick_crc_interval_lines = 2U;
  options.progress_interval_bytes = 8U;
  options.progress = [&](const std::string &, std::uint64_t, std::uint64_t) {++progress_calls;};
  const auto result = ppbng_hsi::verify_envi_part(
    temp.path, "fx10e_segment_0_part_0", options);
  EXPECT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.line_count, 5U);
  EXPECT_EQ(result.crc_lines_checked, 3U);
  EXPECT_GT(progress_calls, 0U);

  std::fstream raw(temp.path / "fx10e_segment_0_part_0.raw",
    std::ios::binary | std::ios::in | std::ios::out);
  raw.seekg(4 * 8);
  char byte{};
  raw.read(&byte, 1);
  byte ^= 0x5A;
  raw.seekp(4 * 8);
  raw.write(&byte, 1);
  raw.close();
  const auto corrupted = ppbng_hsi::verify_envi_part(
    temp.path, "fx10e_segment_0_part_0", options);
  EXPECT_FALSE(corrupted.success);
  EXPECT_NE(corrupted.message.find("CRC32 mismatch"), std::string::npos);
}

TEST(EnviPartVerifier, DetectsTruncatedOrUnindexedRawData)
{
  TemporaryDirectory temp;
  ppbng_hsi::EnviWriterOptions options{temp.path, "fx10e", 1024U, 1U};
  auto [created, writer] = ppbng_hsi::EnviSegmentWriter::create(options, config());
  ASSERT_TRUE(created.success);
  ASSERT_TRUE(writer->append(line(0U, 1U)).success);
  ASSERT_TRUE(writer->close().success);
  const auto raw = temp.path / "fx10e_segment_0_part_0.raw";
  std::filesystem::resize_file(raw, std::filesystem::file_size(raw) - 1U);
  const auto result = ppbng_hsi::verify_envi_part(temp.path, "fx10e_segment_0_part_0");
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("invalid or inconsistent"), std::string::npos);
}

TEST(EnviPartVerifier, RejectsInvalidPersistedControllerTiming)
{
  TemporaryDirectory temp;
  ppbng_hsi::EnviWriterOptions options{temp.path, "fx10e", 1024U, 1U};
  auto [created, writer] = ppbng_hsi::EnviSegmentWriter::create(options, config());
  ASSERT_TRUE(created.success);
  ASSERT_TRUE(writer->append(line(0U, 1U)).success);
  ASSERT_TRUE(writer->close().success);
  const auto path = temp.path / "fx10e_segment_0_part_0.timestamps.csv";
  auto timestamps = read_text(path);
  const auto timing = timestamps.find(",251,1000000,");
  ASSERT_NE(timing, std::string::npos);
  timestamps.replace(timing, std::string(",251,1000000,").size(), ",1000000,1000000,");
  {std::ofstream output(path, std::ios::binary); output << timestamps;}
  const auto result = ppbng_hsi::verify_envi_part(temp.path, "fx10e_segment_0_part_0");
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("invalid or inconsistent"), std::string::npos);
}

TEST(EnviPartVerifier, RequiresTimestampCompanionAndMatchingHeaderGeometry)
{
  TemporaryDirectory missing_timestamp;
  ppbng_hsi::EnviWriterOptions options{missing_timestamp.path, "fx10e", 1024U, 1U};
  auto [created, writer] = ppbng_hsi::EnviSegmentWriter::create(options, config());
  ASSERT_TRUE(created.success);
  ASSERT_TRUE(writer->append(line(0U, 1U)).success);
  ASSERT_TRUE(writer->close().success);
  std::filesystem::remove(missing_timestamp.path /
    "fx10e_segment_0_part_0.timestamps.csv");
  const auto missing = ppbng_hsi::verify_envi_part(
    missing_timestamp.path, "fx10e_segment_0_part_0");
  EXPECT_FALSE(missing.success);
  EXPECT_NE(missing.message.find("companion"), std::string::npos);

  TemporaryDirectory bad_header;
  options = {bad_header.path, "fx10e", 1024U, 1U};
  auto [created_header, header_writer] =
    ppbng_hsi::EnviSegmentWriter::create(options, config());
  ASSERT_TRUE(created_header.success);
  ASSERT_TRUE(header_writer->append(line(0U, 1U)).success);
  ASSERT_TRUE(header_writer->close().success);
  auto header = read_text(bad_header.path / "fx10e_segment_0_part_0.hdr");
  const auto position = header.find("lines = 1");
  ASSERT_NE(position, std::string::npos);
  header.replace(position, std::string("lines = 1").size(), "lines = 9");
  {std::ofstream output(bad_header.path / "fx10e_segment_0_part_0.hdr", std::ios::binary);
    output << header;}
  const auto mismatch = ppbng_hsi::verify_envi_part(
    bad_header.path, "fx10e_segment_0_part_0");
  EXPECT_FALSE(mismatch.success);
  EXPECT_NE(mismatch.message.find("geometry"), std::string::npos);
}

TEST(EnviDatasetVerifier, AcceptsTwoStreamsWithContiguousParts)
{
  TemporaryDirectory temp;
  for (const std::string stem : {"fx10e", "swir"}) {
    ppbng_hsi::EnviWriterOptions options{temp.path, stem, 8U, 1U};
    auto [created, writer] = ppbng_hsi::EnviSegmentWriter::create(options, config());
    ASSERT_TRUE(created.success);
    ASSERT_TRUE(writer->append(line(0U, 1U)).success);
    ASSERT_TRUE(writer->append(line(0U, 2U)).success);
    ASSERT_TRUE(writer->close().success);
  }
  const auto result = ppbng_hsi::verify_envi_dataset(temp.path);
  ASSERT_TRUE(result.success) << result.message;
  ASSERT_EQ(result.streams.size(), 2U);
  EXPECT_EQ(result.streams[0].part_count, 2U);
  EXPECT_EQ(result.streams[0].line_count, 2U);
  EXPECT_EQ(result.streams[1].part_count, 2U);

  const auto fx_only = ppbng_hsi::verify_envi_dataset(temp.path, {"fx10e"}, false);
  ASSERT_TRUE(fx_only.success) << fx_only.message;
  ASSERT_EQ(fx_only.streams.size(), 1U);
  EXPECT_EQ(fx_only.streams.front().stream_name, "fx10e");
}

TEST(EnviDatasetVerifier, RejectsMissingStreamAndPartGap)
{
  TemporaryDirectory missing;
  ppbng_hsi::EnviWriterOptions options{missing.path, "fx10e", 1024U, 1U};
  auto [created, writer] = ppbng_hsi::EnviSegmentWriter::create(options, config());
  ASSERT_TRUE(created.success);
  ASSERT_TRUE(writer->append(line(0U, 1U)).success);
  ASSERT_TRUE(writer->close().success);
  const auto missing_result = ppbng_hsi::verify_envi_dataset(missing.path);
  EXPECT_FALSE(missing_result.success);
  EXPECT_NE(missing_result.message.find("swir"), std::string::npos);

  TemporaryDirectory gap;
  options = {gap.path, "fx10e", 8U, 1U};
  auto [gap_created, gap_writer] = ppbng_hsi::EnviSegmentWriter::create(options, config());
  ASSERT_TRUE(gap_created.success);
  ASSERT_TRUE(gap_writer->append(line(0U, 1U)).success);
  ASSERT_TRUE(gap_writer->append(line(0U, 2U)).success);
  ASSERT_TRUE(gap_writer->close().success);
  for (const std::string suffix : {".raw", ".index.csv", ".timestamps.csv", ".hdr"}) {
    std::filesystem::rename(
      gap.path / ("fx10e_segment_0_part_1" + suffix),
      gap.path / ("fx10e_segment_0_part_2" + suffix));
  }
  const auto gap_result = ppbng_hsi::verify_envi_dataset(gap.path, {"fx10e"});
  EXPECT_FALSE(gap_result.success);
  EXPECT_NE(gap_result.message.find("part index"), std::string::npos);
}

TEST(EnviDatasetVerifier, RejectsCrossPartSourceLineOrCameraEvidenceGap)
{
  TemporaryDirectory temp;
  ppbng_hsi::EnviWriterOptions options{temp.path, "fx10e", 8U, 1U};
  auto [created, writer] = ppbng_hsi::EnviSegmentWriter::create(options, config());
  ASSERT_TRUE(created.success);
  ASSERT_TRUE(writer->append(line(0U, 1U)).success);
  ASSERT_TRUE(writer->append(line(0U, 3U)).success);
  ASSERT_TRUE(writer->close().success);
  const auto result = ppbng_hsi::verify_envi_dataset(temp.path, {"fx10e"});
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("cross-part"), std::string::npos);
}

TEST(EnviDatasetVerifier, MatcherBoundariesResetSourceLineAndDoNotDoubleIncrementSegment)
{
  TemporaryDirectory temp;
  ppbng_hsi::EnviWriterOptions options{temp.path, "fx10e", 8U, 1U};
  auto [created, writer] = ppbng_hsi::EnviSegmentWriter::create(options, config());
  ASSERT_TRUE(created.success);
  ppbng_hsi::PendingTriggerMatcher matcher(8U, 1'000U);
  MatcherWriterAdapter adapter;

  ASSERT_TRUE(matcher.enqueue(matcher_trigger(1U), 1U).accepted);
  auto first = matcher.poll(adapter, 2U);
  ASSERT_TRUE(first.line);
  EXPECT_EQ(first.line->index.segment_id, 0U);
  EXPECT_EQ(first.line->index.segment_line_index, 0U);
  ASSERT_TRUE(writer->append(*first.line).success);

  ++adapter.camera_sequence;
  ASSERT_TRUE(matcher.enqueue(matcher_trigger(2U), 3U).accepted);
  auto second = matcher.poll(adapter, 4U);
  ASSERT_TRUE(second.line);
  EXPECT_EQ(second.line->index.segment_id, 0U);
  EXPECT_EQ(second.line->index.segment_line_index, 1U);
  ASSERT_TRUE(writer->append(*second.line).success);

  // The trigger gap and SDK frame gap describe one boundary, not two.
  adapter.camera_sequence += 2U;
  ASSERT_TRUE(matcher.enqueue(matcher_trigger(4U), 5U).accepted);
  auto after_gap = matcher.poll(adapter, 6U);
  ASSERT_TRUE(after_gap.line);
  EXPECT_EQ(after_gap.line->index.segment_id, 1U);
  EXPECT_EQ(after_gap.line->index.segment_line_index, 0U);
  ASSERT_TRUE(writer->append(*after_gap.line).success);

  // An explicit recovery boundary coincident with an SDK segment change is also one boundary.
  matcher.start_new_segment();
  ++adapter.sdk_segment;
  adapter.camera_sequence = 500U;
  ASSERT_TRUE(matcher.enqueue(matcher_trigger(10U), 7U).accepted);
  auto recovered = matcher.poll(adapter, 8U);
  ASSERT_TRUE(recovered.line);
  EXPECT_EQ(recovered.line->index.segment_id, 2U);
  EXPECT_EQ(recovered.line->index.segment_line_index, 0U);
  ASSERT_TRUE(writer->append(*recovered.line).success);

  ASSERT_TRUE(writer->close().success);
  const auto verified = ppbng_hsi::verify_envi_dataset(temp.path, {"fx10e"});
  EXPECT_TRUE(verified.success) << verified.message;
  ASSERT_EQ(verified.streams.size(), 1U);
  EXPECT_EQ(verified.streams.front().segment_count, 3U);
  EXPECT_EQ(verified.streams.front().line_count, 4U);
}

TEST(EnviDatasetVerifier, RejectsOrphanNonIndexCompanions)
{
  for (const std::string suffix : {".raw", ".hdr", ".timestamps.csv"}) {
    TemporaryDirectory temp;
    ppbng_hsi::EnviWriterOptions options{temp.path, "fx10e", 1024U, 1U};
    auto [created, writer] = ppbng_hsi::EnviSegmentWriter::create(options, config());
    ASSERT_TRUE(created.success);
    ASSERT_TRUE(writer->append(line(0U, 1U)).success);
    ASSERT_TRUE(writer->close().success);
    std::filesystem::copy_file(
      temp.path / ("fx10e_segment_0_part_0" + suffix),
      temp.path / ("fx10e_segment_1_part_0" + suffix));

    const auto result = ppbng_hsi::verify_envi_dataset(temp.path, {"fx10e"});
    EXPECT_FALSE(result.success) << suffix;
    EXPECT_NE(result.message.find("orphan or incomplete"), std::string::npos) << suffix;
  }
}
