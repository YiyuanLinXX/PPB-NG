#include "ppbng_hsi/dataset_context_verifier.hpp"

#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

namespace
{
struct Temp
{
  std::filesystem::path root = std::filesystem::temp_directory_path() /
    ("ppbng_context_verify_" + std::to_string(++sequence));
  static inline std::uint64_t sequence{};
  Temp() {std::filesystem::create_directories(root / "segments");}
  ~Temp() {std::error_code error; std::filesystem::remove_all(root, error);}
};

void write(const std::filesystem::path & path, const std::string & value)
{
  std::ofstream output(path, std::ios::binary);
  output << value;
}

void hsi_index(const Temp & temp, const std::string & stream)
{
  write(temp.root / (stream + "_segment_0_part_0.index.csv"),
    "segment,sdk_segment,part,part_line,source_segment_line,capture_kind,raw_offset,"
    "payload_bytes,payload_crc32,sequence_gap_before,missing_trigger_count,"
    "association_status,anchor_valid,frame_trigger_delta,anchor_frame,anchor_trigger\n"
    "0,0,0,0,0,dark,0,2,0,0,0,MATCHED,1,0,1,1\n"
    "0,0,0,1,1,sample,2,2,0,0,0,MATCHED,1,0,2,2\n");
  write(temp.root / (stream + "_segment_0_part_0.timestamps.csv"),
    "segment,sdk_segment,part,part_line,camera_sequence,host_receive_monotonic_ns,"
    "trigger_sequence,pps_sequence,controller_tick,controller_ticks_per_second,utc_ns,time_status,"
    "uncertainty_ns,association_status,anchor_valid,frame_trigger_delta,anchor_frame,anchor_trigger\n"
    "0,0,0,0,4,9,10,2,3,100,1700000000000000000,locked,9,MATCHED,1,0,1,1\n"
    "0,0,0,1,5,10,11,2,3,100,1700000000000000000,locked,9,MATCHED,1,0,2,2\n");
}

void camera_association(const Temp & temp, const std::string & stream)
{
  const std::string frame = "\"segment\":0,\"sample\":1,\"sdk_frame\":5,"
    "\"camera_timestamp_ns\":7,\"frame_host_monotonic_ns\":10";
  write(temp.root / "segments" / (stream + "_association.ndjson"),
    "{\"state\":\"PENDING\"," + frame +
    ",\"trigger_valid\":false,\"detail\":\"committed\"}\n"
    "{\"state\":\"UNMATCHED\"," + frame +
    ",\"trigger_valid\":false,\"detail\":\"expired\"}\n");
}

ppbng_storage::SegmentSetVerificationResult segments()
{
  ppbng_storage::SegmentSetVerificationResult result;
  result.success = true;
  for (const std::string name : {"rgb", "thermal"}) {
    ppbng_storage::SegmentStreamVerification stream;
    stream.stream_name = name;
    stream.segment_count = 1U;
    stream.record_count = 1U;
    stream.last_sample_id = 1U;
    stream.files.push_back({0U, 1U, 1U, 1U});
    result.streams.push_back(stream);
  }
  return result;
}

std::string context(const std::string & stream, const std::uint64_t sample,
  const std::string & session = "session", const std::string & trigger_channel = "")
{
  const bool camera = stream == "rgb" || stream == "thermal";
  const std::string detail = camera ?
    "UNMATCHED; canonical UTC withheld; raw evidence is retained in association sidecar" :
    "association=MATCHED; raw_trigger_time_status=LOCKED; raw_trigger_utc=preserved";
  return "{\"session_id\":\"" + session + "\",\"device_id\":\"" + stream +
    "\",\"segment\":0,\"sample\":" + std::to_string(sample) +
    ",\"frame_utc_ns\":" + (camera ? "0" : "1700000000000000000") +
    ",\"frame_host_ns\":10,\"trigger_channel\":\"" +
    (trigger_channel.empty() ? stream : trigger_channel) +
    "\",\"trigger_sequence\":" + (camera ? "0" : "11") +
    ",\"pps_sequence\":" + (camera ? "0" : "2") +
    ",\"controller_tick\":" + (camera ? "0" : "3") +
    ",\"controller_ticks_per_second\":" + (camera ? "0" : "100") +
    ",\"time_status\":" + (camera ? "0" : "2") +
    ",\"time_uncertainty_ns\":" + (camera ? "0" : "9") +
    ",\"time_detail\":\"" + detail + "\",\"camera_frame_id_valid\":true,"
    "\"camera_frame_id\":5,\"camera_timestamp_valid\":" +
    (camera ? "true" : "false") + ",\"camera_timestamp\":" + (camera ? "7" : "0") + ","
    "\"gnss_status\":\"UNAVAILABLE\",\"gnss_detail\":\"no bracket\","
    "\"rsm_status\":\"UNAVAILABLE\",\"rsm_detail\":\"no bracket\"}\n";
}

void fixture(Temp & temp)
{
  hsi_index(temp, "fx10e");
  hsi_index(temp, "swir");
  camera_association(temp, "rgb");
  camera_association(temp, "thermal");
}

std::string replace_once(std::string value, const std::string & from, const std::string & to)
{
  const auto position = value.find(from);
  EXPECT_NE(position, std::string::npos);
  if (position != std::string::npos) {value.replace(position, from.size(), to);}
  return value;
}

std::string valid_contexts()
{
  return context("rgb", 1U) + context("thermal", 1U) +
    context("fx10e", 1U) + context("swir", 1U);
}
}  // namespace

TEST(DatasetContextVerifier, RequiresExactlyOneContextForEverySceneSample)
{
  Temp temp;
  fixture(temp);
  write(temp.root / "segments/frame_context.ndjson", valid_contexts());
  const auto result = ppbng_hsi::verify_dataset_frame_context(temp.root, "session", segments());
  EXPECT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.expected_records, 4U);
  EXPECT_EQ(result.context_records, 4U);
  EXPECT_EQ(result.unavailable_gnss_records, 4U);
}

TEST(DatasetContextVerifier, InternalHsiChannelDoesNotReplaceStableDeviceIdentity)
{
  Temp temp;
  fixture(temp);
  write(temp.root / "segments/frame_context.ndjson",
    context("rgb", 1U) + context("thermal", 1U) +
    context("fx10e", 1U, "session", "internal:fx10e") +
    context("swir", 1U, "session", "internal:swir"));
  const auto result = ppbng_hsi::verify_dataset_frame_context(temp.root, "session", segments());
  EXPECT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.context_records, 4U);
}

TEST(DatasetContextVerifier, RejectsMissingDuplicateUnexpectedAndWrongSessionRecords)
{
  Temp missing;
  fixture(missing);
  write(missing.root / "segments/frame_context.ndjson",
    context("rgb", 1U) + context("thermal", 1U) + context("fx10e", 1U));
  EXPECT_NE(ppbng_hsi::verify_dataset_frame_context(
      missing.root, "session", segments()).message.find("lack"), std::string::npos);

  Temp duplicate;
  fixture(duplicate);
  write(duplicate.root / "segments/frame_context.ndjson",
    context("rgb", 1U) + context("rgb", 1U) + context("thermal", 1U) +
    context("fx10e", 1U) + context("swir", 1U));
  EXPECT_NE(ppbng_hsi::verify_dataset_frame_context(
      duplicate.root, "session", segments()).message.find("duplicate"), std::string::npos);

  Temp unexpected;
  fixture(unexpected);
  write(unexpected.root / "segments/frame_context.ndjson", context("magic", 1U));
  EXPECT_NE(ppbng_hsi::verify_dataset_frame_context(
      unexpected.root, "session", segments()).message.find("unexpected"), std::string::npos);

  Temp wrong_session;
  fixture(wrong_session);
  write(wrong_session.root / "segments/frame_context.ndjson", context("rgb", 1U, "other"));
  EXPECT_NE(ppbng_hsi::verify_dataset_frame_context(
      wrong_session.root, "session", segments()).message.find("evidence"), std::string::npos);
}

TEST(DatasetContextVerifier, RejectsExactCameraAndHsiEvidenceMutations)
{
  Temp camera_timestamp;
  fixture(camera_timestamp);
  write(camera_timestamp.root / "segments/frame_context.ndjson",
    replace_once(valid_contexts(), "\"camera_timestamp\":7", "\"camera_timestamp\":8"));
  EXPECT_FALSE(ppbng_hsi::verify_dataset_frame_context(
      camera_timestamp.root, "session", segments()).success);

  Temp hsi_tick;
  fixture(hsi_tick);
  write(hsi_tick.root / "segments/frame_context.ndjson",
    context("rgb", 1U) + context("thermal", 1U) +
    replace_once(context("fx10e", 1U), "\"controller_tick\":3", "\"controller_tick\":4") +
    context("swir", 1U));
  EXPECT_FALSE(ppbng_hsi::verify_dataset_frame_context(hsi_tick.root, "session", segments()).success);

  Temp hsi_utc;
  fixture(hsi_utc);
  write(hsi_utc.root / "segments/frame_context.ndjson",
    context("rgb", 1U) + context("thermal", 1U) +
    replace_once(context("fx10e", 1U), "1700000000000000000", "1700000000000000001") +
    context("swir", 1U));
  EXPECT_FALSE(ppbng_hsi::verify_dataset_frame_context(hsi_utc.root, "session", segments()).success);

  Temp hsi_detail;
  fixture(hsi_detail);
  write(hsi_detail.root / "segments/frame_context.ndjson",
    context("rgb", 1U) + context("thermal", 1U) +
    replace_once(context("fx10e", 1U), "raw_trigger_utc=preserved", "raw_trigger_utc=unavailable") +
    context("swir", 1U));
  EXPECT_FALSE(ppbng_hsi::verify_dataset_frame_context(
      hsi_detail.root, "session", segments()).success);
}
