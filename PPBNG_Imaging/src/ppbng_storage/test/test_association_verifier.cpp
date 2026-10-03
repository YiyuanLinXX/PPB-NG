#include "ppbng_storage/association_verifier.hpp"

#include "ppbng_storage/segment_io.hpp"

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <limits>
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
      ("ppbng_association_verify_" + std::to_string(++sequence));
    std::filesystem::create_directories(root / "segments");
  }
  ~TempSession() {std::error_code error; std::filesystem::remove_all(root, error);}
  std::filesystem::path root;
  static inline std::uint64_t sequence{0U};
};

void segment(
  const std::filesystem::path & root, const std::string & stream,
  const std::uint64_t count)
{
  auto created = ppbng_storage::SegmentWriter::create(
    root, "segments/" + stream + "_000000.ppbseg");
  ASSERT_TRUE(created.first.ok()) << created.first.detail;
  const std::vector<std::byte> payload(4U, std::byte{0x2a});
  for (std::uint64_t sample = 1U; sample <= count; ++sample) {
    ppbng_storage::FrameEnvelope envelope;
    envelope.sample_id = sample;
    ASSERT_TRUE(created.second->append(envelope, payload.data(), payload.size()).ok());
  }
}

void two_part_segment(const std::filesystem::path & root, const std::string & stream)
{
  const std::vector<std::byte> payload(4U, std::byte{0x2a});
  for (std::uint64_t part = 0U; part < 2U; ++part) {
    const auto suffix = part == 0U ? "_000000.ppbseg" : "_000001.ppbseg";
    auto created = ppbng_storage::SegmentWriter::create(
      root, "segments/" + stream + suffix);
    ASSERT_TRUE(created.first.ok()) << created.first.detail;
    ppbng_storage::FrameEnvelope envelope;
    envelope.sample_id = part + 1U;
    ASSERT_TRUE(created.second->append(envelope, payload.data(), payload.size()).ok());
  }
}

void write(const std::filesystem::path & path, const std::string & text)
{
  std::ofstream output(path, std::ios::binary);
  output << text;
}

std::string pending(const std::uint64_t sample, const std::uint64_t segment_id = 0U)
{
  return "{\"state\":\"PENDING\",\"segment\":" + std::to_string(segment_id) +
    ",\"sample\":" + std::to_string(sample) + ",\"sdk_frame\":" +
    std::to_string(100U + sample) + ",\"frame_host_monotonic_ns\":" +
    std::to_string(1000U + sample) + ",\"camera_timestamp_ns\":" +
    std::to_string(3000U + sample) +
    ",\"trigger_valid\":false,\"detail\":\"committed\"}\n";
}

std::string terminal(
  const std::uint64_t sample, const std::string & state,
  const bool trigger, const std::uint64_t segment_id = 0U)
{
  std::string result = "{\"state\":\"" + state + "\",\"segment\":" +
    std::to_string(segment_id) + ",\"sample\":" + std::to_string(sample) +
    ",\"sdk_frame\":" + std::to_string(100U + sample) +
    ",\"frame_host_monotonic_ns\":" + std::to_string(1000U + sample) +
    ",\"camera_timestamp_ns\":" + std::to_string(3000U + sample) +
    ",\"trigger_valid\":" + (trigger ? "true" : "false");
  if (trigger) {
    result += ",\"trigger_sequence\":" + std::to_string(200U + sample) +
      ",\"pps_sequence\":3,\"controller_tick\":10,\"ticks_per_second\":1000,"
      "\"utc_ns\":1700000000000000000,\"time_status\":2,\"uncertainty_ns\":50,"
      "\"trigger_receipt_monotonic_ns\":2000";
  }
  return result + ",\"detail\":\"settled\"}\n";
}

void complete_segments(TempSession & session, const std::uint64_t count = 2U)
{
  segment(session.root, "rgb", count);
  segment(session.root, "thermal", count);
}

ppbng_storage::SegmentStreamVerification verified_stream(
  const TempSession & session, const std::string & name)
{
  const auto set = ppbng_storage::verify_segment_set(session.root);
  EXPECT_TRUE(set.success) << set.message;
  for (const auto & stream : set.streams) {
    if (stream.stream_name == name) {return stream;}
  }
  ADD_FAILURE() << "verified stream not found: " << name;
  return {};
}
}  // namespace

TEST(AssociationVerifier, MatchesOnePendingAndTerminalPerCommittedSample)
{
  TempSession session;
  complete_segments(session);
  const auto path = session.root / "segments/rgb_association.ndjson";
  write(path, "{\"state\":\"SEGMENT_RESET\",\"segment\":0,\"detail\":\"start\"}\n" +
    pending(1U) + terminal(1U, "MATCHED", true) + pending(2U) +
    terminal(2U, "UNMATCHED", false));
  const auto result = ppbng_storage::verify_camera_association(session.root, "rgb");
  EXPECT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.expected_samples, 2U);
  EXPECT_EQ(result.pending_records, 2U);
  EXPECT_EQ(result.terminal_records, 2U);
  EXPECT_EQ(result.matched_records, 1U);
}

TEST(AssociationVerifier, StorageFileRolloverDoesNotCreateLogicalCameraSegment)
{
  TempSession session;
  two_part_segment(session.root, "rgb");
  segment(session.root, "thermal", 1U);
  write(session.root / "segments/rgb_association.ndjson",
    "{\"state\":\"SEGMENT_RESET\",\"segment\":0,\"detail\":\"start\"}\n" +
    pending(1U) + terminal(1U, "UNMATCHED", false) +
    pending(2U) + terminal(2U, "UNMATCHED", false));
  const auto result = ppbng_storage::verify_camera_association(session.root, "rgb");
  EXPECT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.expected_samples, 2U);
}

TEST(AssociationVerifier, AcceptsAbsoluteControllerTickForUnsyncedNoPpsEvidence)
{
  TempSession session;
  complete_segments(session, 1U);
  auto settled = terminal(1U, "MATCHED", true);
  const auto tick = settled.find("\"controller_tick\":10");
  ASSERT_NE(tick, std::string::npos);
  settled.replace(tick, std::string("\"controller_tick\":10").size(),
    "\"controller_tick\":7000123");
  const auto status = settled.find("\"time_status\":2");
  ASSERT_NE(status, std::string::npos);
  settled.replace(status, std::string("\"time_status\":2").size(), "\"time_status\":0");
  const auto utc = settled.find("\"utc_ns\":1700000000000000000");
  ASSERT_NE(utc, std::string::npos);
  settled.replace(utc, std::string("\"utc_ns\":1700000000000000000").size(),
    "\"utc_ns\":0");
  write(session.root / "segments/rgb_association.ndjson", pending(1U) + settled);
  const auto result = ppbng_storage::verify_camera_association(session.root, "rgb");
  EXPECT_TRUE(result.success) << result.message;
}

TEST(AssociationVerifier, RejectsMissingDuplicateOrOutOfOrderLifecycle)
{
  TempSession missing;
  complete_segments(missing, 1U);
  write(missing.root / "segments/rgb_association.ndjson", pending(1U));
  EXPECT_NE(ppbng_storage::verify_camera_association(missing.root, "rgb").message.find("missing"),
    std::string::npos);

  TempSession duplicate;
  complete_segments(duplicate, 1U);
  write(duplicate.root / "segments/rgb_association.ndjson",
    pending(1U) + pending(1U) + terminal(1U, "MATCHED", true));
  EXPECT_NE(ppbng_storage::verify_camera_association(duplicate.root, "rgb").message.find("duplicate"),
    std::string::npos);

  TempSession out_of_order;
  complete_segments(out_of_order, 1U);
  write(out_of_order.root / "segments/rgb_association.ndjson",
    terminal(1U, "MATCHED", true) + pending(1U));
  EXPECT_NE(ppbng_storage::verify_camera_association(out_of_order.root, "rgb").message.find("PENDING"),
    std::string::npos);
}

TEST(AssociationVerifier, RejectsAssociationLifecycleSegmentMismatchAndInvalidTriggerEvidence)
{
  TempSession mismatch;
  complete_segments(mismatch, 1U);
  write(mismatch.root / "segments/rgb_association.ndjson",
    pending(1U, 1U) + terminal(1U, "UNMATCHED", false, 0U));
  EXPECT_NE(ppbng_storage::verify_camera_association(mismatch.root, "rgb").message.find("PENDING"),
    std::string::npos);

  TempSession bad_trigger;
  complete_segments(bad_trigger, 1U);
  auto invalid = terminal(1U, "MATCHED", true);
  const auto position = invalid.find("\"ticks_per_second\":1000");
  ASSERT_NE(position, std::string::npos);
  invalid.replace(position, std::string("\"ticks_per_second\":1000").size(),
    "\"ticks_per_second\":0");
  write(bad_trigger.root / "segments/rgb_association.ndjson", pending(1U) + invalid);
  EXPECT_NE(ppbng_storage::verify_camera_association(bad_trigger.root, "rgb").message.find("trigger"),
    std::string::npos);

  TempSession bad_utc;
  complete_segments(bad_utc, 1U);
  auto invalid_utc = terminal(1U, "MATCHED", true);
  const auto utc_position = invalid_utc.find("\"utc_ns\":1700000000000000000");
  ASSERT_NE(utc_position, std::string::npos);
  invalid_utc.replace(utc_position, std::string("\"utc_ns\":1700000000000000000").size(),
    "\"utc_ns\":1.5");
  write(bad_utc.root / "segments/rgb_association.ndjson", pending(1U) + invalid_utc);
  EXPECT_NE(ppbng_storage::verify_camera_association(bad_utc.root, "rgb").message.find("trigger"),
    std::string::npos);
}

TEST(AssociationVerifier, EnforcesTerminalTriggerSemanticsAndEventDetail)
{
  TempSession matched_without_trigger;
  complete_segments(matched_without_trigger, 1U);
  write(matched_without_trigger.root / "segments/rgb_association.ndjson",
    pending(1U) + terminal(1U, "MATCHED", false));
  EXPECT_NE(ppbng_storage::verify_camera_association(
      matched_without_trigger.root, "rgb").message.find("trigger_valid"), std::string::npos);

  TempSession unmatched_with_trigger;
  complete_segments(unmatched_with_trigger, 1U);
  write(unmatched_with_trigger.root / "segments/rgb_association.ndjson",
    pending(1U) + terminal(1U, "UNMATCHED", true));
  EXPECT_NE(ppbng_storage::verify_camera_association(
      unmatched_with_trigger.root, "rgb").message.find("trigger_valid"), std::string::npos);

  TempSession empty_event;
  complete_segments(empty_event, 1U);
  write(empty_event.root / "segments/rgb_association.ndjson",
    "{\"state\":\"DEGRADED\",\"segment\":0,\"detail\":\"\"}\n" + pending(1U) +
    terminal(1U, "UNMATCHED", false));
  EXPECT_NE(ppbng_storage::verify_camera_association(
      empty_event.root, "rgb").message.find("event"), std::string::npos);
}

TEST(AssociationVerifier, RejectsTruncatedFinalRecordAndUnknownState)
{
  TempSession truncated;
  complete_segments(truncated, 1U);
  auto text = pending(1U) + terminal(1U, "UNMATCHED", false);
  text.pop_back();
  write(truncated.root / "segments/rgb_association.ndjson", text);
  EXPECT_NE(ppbng_storage::verify_camera_association(truncated.root, "rgb").message.find("newline"),
    std::string::npos);

  TempSession unknown;
  complete_segments(unknown, 1U);
  write(unknown.root / "segments/rgb_association.ndjson",
    "{\"state\":\"MAGIC\",\"segment\":0}\n");
  EXPECT_NE(ppbng_storage::verify_camera_association(unknown.root, "rgb").message.find("unknown"),
    std::string::npos);
}

TEST(AssociationVerifier, RejectsCameraTimestampMismatchAcrossLifecycle)
{
  TempSession session;
  complete_segments(session, 1U);
  auto settled = terminal(1U, "MATCHED", true);
  const auto position = settled.find("\"camera_timestamp_ns\":3001");
  ASSERT_NE(position, std::string::npos);
  settled.replace(position, std::string("\"camera_timestamp_ns\":3001").size(),
    "\"camera_timestamp_ns\":9999");
  write(session.root / "segments/rgb_association.ndjson", pending(1U) + settled);
  EXPECT_NE(ppbng_storage::verify_camera_association(session.root, "rgb").message.find("PENDING"),
    std::string::npos);
}

TEST(AssociationVerifier, EnforcesExplicitPendingHardCapAndReleasesSettledSlots)
{
  TempSession at_cap;
  complete_segments(at_cap, 2U);
  write(at_cap.root / "segments/rgb_association.ndjson",
    pending(1U) + pending(2U) + terminal(2U, "MATCHED", true) +
    terminal(1U, "MATCHED", true));
  const auto stream = verified_stream(at_cap, "rgb");
  EXPECT_TRUE(ppbng_storage::verify_camera_association(at_cap.root, stream, 2U).success);
  const auto exceeded = ppbng_storage::verify_camera_association(at_cap.root, stream, 1U);
  EXPECT_FALSE(exceeded.success);
  EXPECT_NE(exceeded.message.find("hard cap"), std::string::npos);

  TempSession released;
  complete_segments(released, 2U);
  write(released.root / "segments/rgb_association.ndjson",
    pending(1U) + terminal(1U, "MATCHED", true) + pending(2U) +
    terminal(2U, "UNMATCHED", false));
  EXPECT_TRUE(ppbng_storage::verify_camera_association(
      released.root, verified_stream(released, "rgb"), 1U).success);
  EXPECT_NE(ppbng_storage::verify_camera_association(
      released.root, verified_stream(released, "rgb"), 0U).message.find("positive"),
    std::string::npos);
  EXPECT_NE(ppbng_storage::verify_camera_association(
      released.root, verified_stream(released, "rgb"),
      ppbng_storage::kAssociationPendingHardCap + 1U).message.find("hard cap"),
    std::string::npos);
}

TEST(AssociationVerifier, RejectsSparseOrHugeSampleWithoutSampleIndexedAllocation)
{
  TempSession session;
  complete_segments(session, 1U);
  write(session.root / "segments/rgb_association.ndjson",
    pending((std::numeric_limits<std::uint64_t>::max)()));
  const auto huge = ppbng_storage::verify_camera_association(
    session.root, verified_stream(session, "rgb"), 1U);
  EXPECT_FALSE(huge.success);
  EXPECT_NE(huge.message.find("invalid frame association"), std::string::npos);

  ppbng_storage::SegmentStreamVerification sparse;
  sparse.stream_name = "rgb";
  sparse.segment_count = 1U;
  sparse.record_count = 1U;
  sparse.last_sample_id = (std::numeric_limits<std::uint64_t>::max)();
  sparse.files.push_back({0U, 1U, sparse.last_sample_id, sparse.last_sample_id});
  const auto summary = ppbng_storage::verify_camera_association(session.root, sparse, 1U);
  EXPECT_FALSE(summary.success);
  EXPECT_NE(summary.message.find("sparse"), std::string::npos);
}
