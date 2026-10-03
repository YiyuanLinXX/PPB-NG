#include "ppbng_timing/timing_event_writer.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <vector>

namespace {

class UniqueTemp {
 public:
  UniqueTemp() {
    root = std::filesystem::temp_directory_path() /
        ("ppbng_timing_" + std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(root / "session" / "segments");
  }
  ~UniqueTemp() { std::error_code ignored; std::filesystem::remove_all(root, ignored); }
  std::filesystem::path root;
};

TEST(TimingSessionBinding, IsIdempotentAndRejectsEscapeWithoutCreatingFiles) {
  UniqueTemp temp;
  ppbng_timing::TimingSessionBinding binding(temp.root);
  const auto accepted = binding.prepare("req-1", "session-1",
                                        (temp.root / "session").string(), false);
  EXPECT_TRUE(accepted.accepted);
  EXPECT_FALSE(accepted.duplicate);
  EXPECT_TRUE(binding.bound());
  EXPECT_TRUE(std::filesystem::is_empty(temp.root / "session" / "segments"));
  const auto duplicate = binding.prepare("req-1", "session-1",
                                         (temp.root / "session").string(), false);
  EXPECT_TRUE(duplicate.accepted);
  EXPECT_TRUE(duplicate.duplicate);
  EXPECT_FALSE(binding.prepare("req-1", "other", (temp.root / "session").string(), false).accepted);
  std::filesystem::create_directories(temp.root / "second" / "segments");
  EXPECT_FALSE(binding.prepare("req-new", "session-2",
                               (temp.root / "second").string(), false).accepted);
  EXPECT_EQ(binding.session_id(), "session-1");
  EXPECT_FALSE(binding.prepare("req-2", "escape", temp.root.parent_path().string(), false).accepted);
}

TEST(TimingEventWriter, ExclusiveAppendPersistsUnknownUtcSentinelAndTrigger) {
  UniqueTemp temp;
  const auto session = temp.root / "session";
  auto created = ppbng_timing::TimingEventWriter::create(session);
  ASSERT_TRUE(created.first.ok()) << created.first.detail;
  const ppbng_timing::PpsAnchor anchor{7U, 3U,
      (std::numeric_limits<std::int64_t>::min)(), 1'000'000U,
      ppbng_timing::TimeLock::kLocked};
  EXPECT_TRUE(created.second->append_pps(anchor, 55U).ok());
  const ppbng_timing::TriggerEvent trigger{7U, 1U, ppbng_timing::Channel::kFx10e,
      1U, 3U, 10U, 1'000'000U, ppbng_timing::TimeLock::kLocked};
  EXPECT_TRUE(created.second->append_trigger(trigger, 66U).ok());
  EXPECT_TRUE(created.second->flush().ok());
  EXPECT_EQ(created.second->records_written(), 2U);

  const auto path = session / "segments" / "timing_events.bin";
  std::ifstream input(path, std::ios::binary);
  const std::vector<std::uint8_t> bytes(
      (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  ASSERT_GT(bytes.size(), 50U);
  EXPECT_EQ(std::string(bytes.begin(), bytes.begin() + 4), "PTLG");
  EXPECT_EQ(bytes[8], 1U);   // PPS record
  EXPECT_EQ(bytes[42], 0U);  // utc_valid=false
  EXPECT_EQ(bytes[50], 0x80U); // little-endian INT64_MIN high byte

  const auto collision = ppbng_timing::TimingEventWriter::create(session);
  EXPECT_EQ(collision.first.code, ppbng_timing::TimingLogCode::already_exists);
}

TEST(TimingEventWriter, RejectsUnsafePathAndReportsInjectedWriteFailure) {
  UniqueTemp temp;
  const auto session = temp.root / "session";
  EXPECT_EQ(ppbng_timing::TimingEventWriter::create(session, "../escape.bin").first.code,
            ppbng_timing::TimingLogCode::unsafe_path);
  auto writer = ppbng_timing::TimingEventWriter::create(
      session, "segments/fail.bin", {8U, false});
  ASSERT_TRUE(writer.first.ok());
  const ppbng_timing::PpsAnchor anchor{7U, 1U,
      (std::numeric_limits<std::int64_t>::min)(), 1U,
      ppbng_timing::TimeLock::kUnsynced};
  EXPECT_EQ(writer.second->append_pps(anchor, 2U).code,
            ppbng_timing::TimingLogCode::write_failed);
  auto flush_writer = ppbng_timing::TimingEventWriter::create(
      session, "segments/fail_flush.bin",
      {(std::numeric_limits<std::uint64_t>::max)(), true});
  ASSERT_TRUE(flush_writer.first.ok());
  EXPECT_EQ(flush_writer.second->flush().code,
            ppbng_timing::TimingLogCode::flush_failed);
}

}  // namespace
