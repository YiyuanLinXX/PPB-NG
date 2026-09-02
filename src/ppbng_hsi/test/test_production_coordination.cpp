#include "ppbng_hsi/pending_trigger_matcher.hpp"
#include "ppbng_hsi/session_binding.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>

namespace
{
class TempSession
{
public:
  TempSession()
  {
    root = std::filesystem::temp_directory_path() /
      ("ppbng-hsi-binding-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    session = root / "dataset";
    std::filesystem::create_directories(session / "segments");
  }
  ~TempSession() {std::error_code ec; std::filesystem::remove_all(root, ec);}
  std::filesystem::path root, session;
};

ppbng_hsi::TriggerEvent trigger(std::uint64_t sequence)
{
  ppbng_hsi::TriggerEvent value;
  value.channel = "fx"; value.channel_sequence = sequence;
  value.ticks_per_second = 100U; value.offset_ticks = 1U;
  return value;
}

class DelayedAdapter final : public ppbng_hsi::IHsiAdapter
{
public:
  ppbng_hsi::CameraKind kind() const noexcept override {return ppbng_hsi::CameraKind::fx10e;}
  ppbng_hsi::HsiState state() const noexcept override {return ppbng_hsi::HsiState::streaming;}
  const ppbng_hsi::HsiConfig & config() const noexcept override {return config_;}
  std::uint32_t segment_id() const noexcept override {return 0U;}
  ppbng_hsi::OperationResult connect() override {return {true, {}};}
  ppbng_hsi::OperationResult configure(const ppbng_hsi::HsiConfig &) override {return {true, {}};}
  ppbng_hsi::OperationResult close_shutter() override {return {true, {}};}
  ppbng_hsi::OperationResult begin_dark_capture(std::size_t) override {return {true, {}};}
  ppbng_hsi::OperationResult open_shutter() override {return {true, {}};}
  ppbng_hsi::OperationResult start_streaming() override {return {true, {}};}
  ppbng_hsi::OperationResult stop_streaming() override {return {true, {}};}
  ppbng_hsi::OperationResult recover() override {return {true, {}};}
  ppbng_hsi::LineResult on_trigger(const ppbng_hsi::TriggerEvent & value) override
  {
    ++calls;
    if (disconnected) {
      return {ppbng_hsi::LineStatus::disconnected, std::nullopt, "transport disconnected"};
    }
    if (!frame_available) {return {ppbng_hsi::LineStatus::not_ready, std::nullopt, "late frame"};}
    ppbng_hsi::LineRecord line;
    line.index.trigger_sequence = value.channel_sequence;
    line.index.camera_line_sequence = frame_number;
    line.index.segment_id = sdk_segment;
    return {ppbng_hsi::LineStatus::produced, line, "matched"};
  }
  bool frame_available{false};
  bool disconnected{false};
  unsigned calls{0U};
  std::uint64_t frame_number{100U};
  std::uint32_t sdk_segment{7U};
private:
  ppbng_hsi::HsiConfig config_{};
};
}  // namespace

TEST(PendingTriggerMatcher, RetainsTriggerUntilLaterFrameAndMatchesBySequenceOrder)
{
  ppbng_hsi::PendingTriggerMatcher matcher(4U, 100U);
  DelayedAdapter adapter;
  ASSERT_TRUE(matcher.enqueue(trigger(10U), 1000U).accepted);
  EXPECT_EQ(matcher.poll(adapter, 1050U).status, ppbng_hsi::TriggerMatchStatus::waiting_for_frame);
  EXPECT_EQ(matcher.pending(), 1U);
  adapter.frame_available = true;
  const auto matched = matcher.poll(adapter, 1060U);
  ASSERT_EQ(matched.status, ppbng_hsi::TriggerMatchStatus::produced);
  EXPECT_EQ(matched.line->index.trigger_sequence, 10U);
  EXPECT_EQ(matched.line->index.association_status, ppbng_hsi::AssociationStatus::unverified);
  EXPECT_FALSE(matched.line->index.association_anchor_valid);
  ASSERT_TRUE(matcher.enqueue(trigger(11U), 1070U).accepted);
  adapter.frame_number = 101U;
  const auto anchored = matcher.poll(adapter, 1071U);
  ASSERT_EQ(anchored.status, ppbng_hsi::TriggerMatchStatus::produced);
  EXPECT_EQ(anchored.line->index.association_status,
    ppbng_hsi::AssociationStatus::consistent_unverified);
  EXPECT_FALSE(anchored.line->index.association_anchor_valid);
  EXPECT_EQ(anchored.line->index.frame_trigger_delta, 90);
  EXPECT_EQ(anchored.line->index.association_anchor_frame, 100U);
  EXPECT_EQ(anchored.line->index.association_anchor_trigger, 10U);
  EXPECT_EQ(anchored.line->index.sdk_segment_id, 7U);
  EXPECT_EQ(matcher.pending(), 0U);
}

TEST(PendingTriggerMatcher, DisconnectIsRecoverableAndNewSegmentResetsAssociation)
{
  ppbng_hsi::PendingTriggerMatcher matcher(8U, 1000U);
  DelayedAdapter adapter; adapter.frame_available = true;
  ASSERT_TRUE(matcher.enqueue(trigger(10U), 1U).accepted);
  adapter.frame_number = 100U;
  const auto first = matcher.poll(adapter, 2U);
  ASSERT_TRUE(first.line);
  const auto first_segment = first.line->index.segment_id;

  ASSERT_TRUE(matcher.enqueue(trigger(11U), 3U).accepted);
  adapter.disconnected = true;
  EXPECT_EQ(matcher.poll(adapter, 4U).status, ppbng_hsi::TriggerMatchStatus::disconnected);

  matcher.start_new_segment();
  adapter.disconnected = false;
  adapter.frame_number = 500U;
  ASSERT_TRUE(matcher.enqueue(trigger(50U), 5U).accepted);
  const auto recovered = matcher.poll(adapter, 6U);
  ASSERT_TRUE(recovered.line);
  EXPECT_GT(recovered.line->index.segment_id, first_segment);
  EXPECT_EQ(recovered.line->index.association_status,
    ppbng_hsi::AssociationStatus::unverified);
  EXPECT_FALSE(recovered.line->index.association_anchor_valid);
}

TEST(PendingTriggerMatcher, DeltaMismatchAndFrameGapAreUnmatchedAndStartNewSegments)
{
  ppbng_hsi::PendingTriggerMatcher matcher(8U, 1000U);
  DelayedAdapter adapter; adapter.frame_available = true;
  ASSERT_TRUE(matcher.enqueue(trigger(1U), 1U).accepted);
  adapter.frame_number = 101U;
  ASSERT_EQ(matcher.poll(adapter, 2U).line->index.association_status,
    ppbng_hsi::AssociationStatus::unverified);
  ASSERT_TRUE(matcher.enqueue(trigger(2U), 3U).accepted);
  adapter.frame_number = 102U;
  ASSERT_EQ(matcher.poll(adapter, 4U).line->index.association_status,
    ppbng_hsi::AssociationStatus::consistent_unverified);
  const auto anchored_segment = matcher.source_segment();

  ASSERT_TRUE(matcher.enqueue(trigger(3U), 5U).accepted);
  adapter.frame_number = 104U;
  const auto gap = matcher.poll(adapter, 6U);
  ASSERT_TRUE(gap.line);
  EXPECT_EQ(gap.line->index.association_status, ppbng_hsi::AssociationStatus::unmatched);
  EXPECT_GT(gap.line->index.segment_id, anchored_segment);
  EXPECT_FALSE(gap.line->index.association_anchor_valid);

  ASSERT_TRUE(matcher.enqueue(trigger(4U), 7U).accepted);
  adapter.frame_number = 105U;
  const auto candidate = matcher.poll(adapter, 8U);
  EXPECT_EQ(candidate.line->index.association_status, ppbng_hsi::AssociationStatus::unverified);
  ASSERT_TRUE(matcher.enqueue(trigger(5U), 9U).accepted);
  adapter.frame_number = 107U;  // delta changes and frame 106 is absent
  const auto mismatch = matcher.poll(adapter, 10U);
  EXPECT_EQ(mismatch.line->index.association_status, ppbng_hsi::AssociationStatus::unmatched);
  EXPECT_GT(mismatch.line->index.segment_id, gap.line->index.segment_id);

  ASSERT_TRUE(matcher.enqueue(trigger(6U), 11U).accepted);
  adapter.frame_number = 106U;  // frame-id regression
  const auto regression = matcher.poll(adapter, 12U);
  EXPECT_EQ(regression.line->index.association_status, ppbng_hsi::AssociationStatus::unmatched);
  EXPECT_GT(regression.line->index.segment_id, mismatch.line->index.segment_id);
}

TEST(PendingTriggerMatcher, MatchedRequiresExplicitHardwareEvidenceConfirmation)
{
  ppbng_hsi::PendingTriggerMatcher matcher(8U, 1000U, true);
  DelayedAdapter adapter; adapter.frame_available = true;
  ASSERT_TRUE(matcher.enqueue(trigger(10U), 1U).accepted);
  adapter.frame_number = 100U;
  EXPECT_EQ(matcher.poll(adapter, 2U).line->index.association_status,
    ppbng_hsi::AssociationStatus::unverified);
  ASSERT_TRUE(matcher.enqueue(trigger(11U), 3U).accepted);
  adapter.frame_number = 101U;
  const auto confirmed = matcher.poll(adapter, 4U);
  EXPECT_EQ(confirmed.line->index.association_status, ppbng_hsi::AssociationStatus::matched);
  EXPECT_TRUE(confirmed.line->index.association_anchor_valid);
}

TEST(PendingTriggerMatcher, TriggerGapStartsUnverifiedReanchorSegment)
{
  ppbng_hsi::PendingTriggerMatcher matcher(8U, 1000U);
  DelayedAdapter adapter; adapter.frame_available = true;
  ASSERT_TRUE(matcher.enqueue(trigger(20U), 1U).accepted);
  adapter.frame_number = 200U;
  const auto first = matcher.poll(adapter, 2U);
  ASSERT_TRUE(matcher.enqueue(trigger(22U), 3U).accepted);
  adapter.frame_number = 202U;
  const auto after_gap = matcher.poll(adapter, 4U);
  EXPECT_EQ(after_gap.line->index.association_status, ppbng_hsi::AssociationStatus::unverified);
  EXPECT_GT(after_gap.line->index.segment_id, first.line->index.segment_id);
}

TEST(PendingTriggerMatcher, ExpiredMissingFrameAndOverflowAreExplicit)
{
  ppbng_hsi::PendingTriggerMatcher matcher(1U, 100U);
  DelayedAdapter adapter;
  ASSERT_TRUE(matcher.enqueue(trigger(1U), 1000U).accepted);
  const auto overflow = matcher.enqueue(trigger(2U), 1001U);
  EXPECT_FALSE(overflow.accepted); EXPECT_TRUE(overflow.overflow);
  EXPECT_EQ(matcher.overflow_count(), 1U);
  EXPECT_EQ(matcher.poll(adapter, 1101U).status, ppbng_hsi::TriggerMatchStatus::expired);
  EXPECT_EQ(matcher.expired_count(), 1U);
  ASSERT_TRUE(matcher.enqueue(trigger(3U), 1102U).accepted);
  adapter.frame_available = true;
  const auto after_overflow_and_expiry = matcher.poll(adapter, 1103U);
  ASSERT_TRUE(after_overflow_and_expiry.line);
  EXPECT_EQ(after_overflow_and_expiry.line->index.association_status,
    ppbng_hsi::AssociationStatus::unverified);
}

TEST(SessionBinding, EnforcesRootSubdirectoriesAndFullLifetimeIdempotency)
{
  TempSession temp;
  ppbng_hsi::SessionBinding binding;
  binding.set_allowed_output_root(temp.root);
  ASSERT_TRUE(binding.prepare("r1", "s1", temp.session.string(), false, true).accepted);
  EXPECT_EQ(binding.directory(), std::filesystem::weakly_canonical(temp.session / "segments"));
  const auto replay = binding.prepare("r1", "s1", temp.session.string(), false, true);
  EXPECT_TRUE(replay.accepted); EXPECT_TRUE(replay.duplicate);
  EXPECT_FALSE(binding.prepare("r1", "changed", temp.session.string(), false, true).accepted);
  auto session2 = temp.root / "dataset2";
  std::filesystem::create_directories(session2 / "segments");
  ASSERT_TRUE(binding.prepare("r2", "s2", session2.string(), false, true).accepted);
  EXPECT_TRUE(binding.prepare("r1", "s1", temp.session.string(), false, true).duplicate);
  EXPECT_FALSE(binding.prepare("r3", "s3", session2.string(), true, false).accepted);
  EXPECT_FALSE(binding.prepare("r4", "s4", temp.root.string(), false, true).accepted);
}
