#include "ppbng_rgb/mock_rgb_backend.hpp"
#include "ppbng_rgb/runtime_contract.hpp"
#include "ppbng_rgb/start_cleanup_guard.hpp"
#include "ppbng_rgb/sequence_association.hpp"
#include "ppbng_rgb/recovery_supervisor.hpp"
#include <gtest/gtest.h>
using namespace ppbng_rgb;
namespace {Status configured(MockRgbBackend&b,StopToken t){std::vector<std::string>ids;auto s=b.discover(std::chrono::milliseconds(5),t,ids);if(!s.ok())return s;s=b.open(ids[0],std::chrono::milliseconds(5),t);if(!s.ok())return s;return b.configure({},std::chrono::milliseconds(5),t);}}

TEST(RgbAdapter, LifecycleProducesRawBayerFrames)
{
  StopSource s;MockRgbBackend b;EXPECT_EQ(b.arm(std::chrono::milliseconds(5),s.token()).code,ErrorCode::invalid_state);ASSERT_TRUE(configured(b,s.token()).ok());RgbReadback r;ASSERT_TRUE(b.readback(std::chrono::milliseconds(5),s.token(),r).ok());EXPECT_TRUE(validate_raw_bayer_contract(r).ok());ASSERT_TRUE(b.arm(std::chrono::milliseconds(5),s.token()).ok());auto f=b.next_frame(std::chrono::milliseconds(5),s.token());EXPECT_TRUE(f.status.ok());EXPECT_EQ(f.frame.info().pixel_format,"BayerRG8");EXPECT_EQ(f.frame.size(),12'288'000U);
  EXPECT_EQ(f.frame.info().camera_timestamp_ns,1'000'000U);EXPECT_GT(f.frame.info().host_receive_monotonic_ns,0U);
  EXPECT_TRUE(f.frame.info().chunk_data_valid);EXPECT_DOUBLE_EQ(f.frame.info().exposure_time_us,1000.0);
  EXPECT_TRUE(f.frame.info().white_balance_red_valid);EXPECT_DOUBLE_EQ(f.frame.info().white_balance_red,1.5);
}
TEST(RgbAdapter, RejectsNonBayerReadbackAndReadOnlyAccess)
{
  StopSource s;FakeRgbNodeMap m;m.force_readback_mismatch=true;MockRgbBackend mismatch(m);EXPECT_EQ(configured(mismatch,s.token()).code,ErrorCode::readback_mismatch);
  FakeRgbNodeMap denied;denied.control_access=false;MockRgbBackend d(denied);std::vector<std::string>ids;ASSERT_TRUE(d.discover(std::chrono::milliseconds(5),s.token(),ids).ok());EXPECT_EQ(d.open(ids[0],std::chrono::milliseconds(5),s.token()).code,ErrorCode::access_denied);
}
TEST(RgbAdapter, TimeoutCancellationAndConfigurationHash)
{
  StopSource s;MockRgbBackend b;std::vector<std::string>ids;EXPECT_EQ(b.discover(std::chrono::milliseconds(0),s.token(),ids).code,ErrorCode::timeout);s.request_stop();EXPECT_EQ(b.discover(std::chrono::milliseconds(5),s.token(),ids).code,ErrorCode::cancelled);RgbConfiguration a,c;EXPECT_EQ(configuration_hash(a),configuration_hash(c));c.trigger_source="Line1";EXPECT_NE(configuration_hash(a),configuration_hash(c));
}
TEST(RgbAdapter, RequiresOneExactDeviceIdentity)
{
  std::string selected;
  EXPECT_EQ(select_unique_device_id("",{"123"},selected).code,ErrorCode::invalid_configuration);
  EXPECT_EQ(select_unique_device_id("123",{"456"},selected).code,ErrorCode::not_found);
  EXPECT_EQ(select_unique_device_id("123",{"123","123"},selected).code,ErrorCode::invalid_configuration);
  EXPECT_TRUE(select_unique_device_id("123",{"456","123"},selected).ok());
  EXPECT_EQ(selected,"123");
}
TEST(RgbAdapter, RejectsSoftwareOrNonFrameStartTrigger)
{
  RgbReadback r;
  r.trigger_source="Software";
  EXPECT_EQ(validate_raw_bayer_contract(r).code,ErrorCode::invalid_configuration);
  r.trigger_source="Line0";
  r.trigger_selector="AcquisitionStart";
  EXPECT_EQ(validate_raw_bayer_contract(r).code,ErrorCode::invalid_configuration);
}
TEST(RgbAdapter, LeaseExactlyOnceAndIncompleteStatus)
{
  StopSource s;FakeRgbNodeMap n;n.incomplete_every_n_frames=2;MockRgbBackend b(n);ASSERT_TRUE(configured(b,s.token()).ok());ASSERT_TRUE(b.arm(std::chrono::milliseconds(5),s.token()).ok());{auto one=b.next_frame(std::chrono::milliseconds(5),s.token());FrameLease moved=std::move(one.frame);moved.release();moved.release();}EXPECT_EQ(b.release_count(),1U);{auto two=b.next_frame(std::chrono::milliseconds(5),s.token());EXPECT_EQ(two.status.code,ErrorCode::incomplete_frame);EXPECT_FALSE(two.frame.info().complete);}EXPECT_EQ(b.release_count(),2U);
}
TEST(RgbAdapter, QueueBackpressureDropsNewestAndCounts)
{
  StopSource s;MockRgbBackend b;ASSERT_TRUE(configured(b,s.token()).ok());ASSERT_TRUE(b.arm(std::chrono::milliseconds(5),s.token()).ok());BoundedFrameQueue q(1);auto a=b.next_frame(std::chrono::milliseconds(5),s.token());auto c=b.next_frame(std::chrono::milliseconds(5),s.token());EXPECT_TRUE(q.try_push(std::move(a.frame)));EXPECT_FALSE(q.try_push(std::move(c.frame)));EXPECT_EQ(b.release_count(),1U);auto st=q.statistics();EXPECT_EQ(st.pushed,1U);EXPECT_EQ(st.dropped_newest,1U);auto p=q.pop(std::chrono::milliseconds(5),s.token());EXPECT_TRUE(p.status.ok());EXPECT_EQ(p.frame.info().frame_id,1U);
}
TEST(RgbAdapter, RecoveryStartsNewSegmentAndRequiresReadbackAgain)
{
  StopSource s;MockRgbBackend b;ASSERT_TRUE(configured(b,s.token()).ok());ASSERT_TRUE(b.arm(std::chrono::milliseconds(5),s.token()).ok());auto old=b.next_frame(std::chrono::milliseconds(5),s.token());EXPECT_EQ(old.frame.info().segment_index,0U);ASSERT_TRUE(b.recover(std::chrono::milliseconds(5),s.token()).ok());EXPECT_EQ(b.state(),LifecycleState::open);ASSERT_TRUE(b.configure({},std::chrono::milliseconds(5),s.token()).ok());ASSERT_TRUE(b.arm(std::chrono::milliseconds(5),s.token()).ok());auto fresh=b.next_frame(std::chrono::milliseconds(5),s.token());EXPECT_EQ(fresh.frame.info().segment_index,1U);EXPECT_EQ(fresh.frame.info().frame_id,1U);
}
TEST(RgbRuntimeContract, CarriesSdkAndHostTimingWithoutChangingRawGeometry)
{
  FrameInfo info{42,3,12'288'000,true,"BayerRG8",99'000,123'456};
  info.chunk_data_valid=true;info.chunk_frame_id_valid=true;info.chunk_frame_id=41;
  info.chunk_timestamp_valid=true;info.chunk_timestamp=98'000;
  info.exposure_time_valid=true;info.exposure_time_us=1234.5;
  info.gain_valid=true;info.gain_db=2.0;info.black_level_valid=true;info.black_level=0.5;
  info.white_balance_red_valid=true;info.white_balance_red=1.4;
  info.white_balance_blue_valid=true;info.white_balance_blue=1.2;
  info.exposure_auto="Continuous";info.gain_auto="Continuous";info.balance_white_auto="Off";
  RgbReadback readback;
  const auto d=describe_runtime_frame(info,readback);
  EXPECT_EQ(d.sdk_frame_counter,42U);EXPECT_EQ(d.camera_timestamp_ns,99'000U);
  EXPECT_EQ(d.host_receive_monotonic_ns,123'456U);EXPECT_TRUE(d.raw_payload_preserved);
  EXPECT_EQ(d.transport_width,4096U);EXPECT_EQ(d.payload_size_bytes,12'288'000U);
  EXPECT_EQ(d.validation_detail,"raw Bayer payload validated");
  EXPECT_TRUE(d.chunk_data_valid);EXPECT_EQ(d.chunk_frame_id,41U);EXPECT_EQ(d.chunk_timestamp,98'000U);
  EXPECT_DOUBLE_EQ(d.exposure_time_us,1234.5);
  EXPECT_DOUBLE_EQ(d.white_balance_blue,1.2);EXPECT_EQ(d.balance_white_auto,"Off");
}
TEST(RgbRuntimeContract, FailedStartCleanupRunsExactlyOnceUnlessCommitted)
{
  int releases=0;{StartCleanupGuard guard([&](){++releases;});}EXPECT_EQ(releases,1);
  {StartCleanupGuard guard([&](){++releases;});guard.commit();}EXPECT_EQ(releases,1);
}
TEST(RgbSequenceAssociation, HandlesEitherOrderGapsResetExpiryRegressionAndOverflow)
{
  SequenceAssociator a(2,100,true);a.reset(4);
  EXPECT_TRUE(a.frame({100,1,4,10}).settlements.empty());
  auto first=a.trigger({50,9,1,10,2,20,1000,2});ASSERT_EQ(first.settlements.size(),1U);EXPECT_EQ(first.settlements[0].trigger->sequence,50U);
  a.trigger({51,9,1,10,2,21,1001,2});a.trigger({52,9,1,10,2,22,1002,2});
  auto gap=a.frame({102,2,4,23});ASSERT_EQ(gap.settlements.size(),1U);EXPECT_EQ(gap.settlements[0].trigger->sequence,52U);
  EXPECT_FALSE(a.frame({101,3,4,24}).accepted);
  a.frame({103,4,4,30});auto reset=a.reset(5);ASSERT_EQ(reset.settlements.size(),1U);EXPECT_EQ(reset.settlements[0].kind,SettlementKind::unmatched);
  a.frame({1,5,5,100});auto expired=a.expire(200);ASSERT_EQ(expired.settlements.size(),1U);
  a.reset(6);a.frame({1,1,6,1});a.frame({2,2,6,2});EXPECT_FALSE(a.frame({3,3,6,3}).accepted);
}
TEST(RgbSequenceAssociation, MissedTriggerAmbiguityIsUnverifiedByDefault){
  SequenceAssociator a(4,100);a.reset(1);a.trigger({10,1,0,1000,0,1,0,0});auto r=a.frame({20,1,1,2});ASSERT_EQ(r.settlements.size(),1U);EXPECT_EQ(r.settlements[0].kind,SettlementKind::consistent_unverified);EXPECT_NE(r.settlements[0].detail.find("ambiguity"),std::string::npos);
}
TEST(RgbArmPreflight, IsPureAndFailClosed){RgbConfiguration c;c.width=10;c.height=2;c.row_stride_bytes=10;c.payload_bytes=20;c.pixel_format="BayerRG8";c.trigger_source="Line0";c.trigger_activation="RisingEdge";EXPECT_FALSE(validate_inert_arm_preflight(false,true,"id","rgb",c,8,100).ok());EXPECT_FALSE(validate_inert_arm_preflight(true,false,"id","rgb",c,8,100).ok());EXPECT_TRUE(validate_inert_arm_preflight(true,true,"id","rgb",c,8,100).ok());}
TEST(RgbRecoverySupervisor, ThresholdBackoffCapAndBoundedAttempts){
  RecoverySupervisor r({3,3,std::chrono::milliseconds(10),std::chrono::milliseconds(25)});
  EXPECT_FALSE(r.observe_timeout());EXPECT_FALSE(r.observe_timeout());EXPECT_TRUE(r.observe_timeout());
  r.begin_episode();auto a=r.next_attempt();EXPECT_TRUE(a.allowed);EXPECT_EQ(a.backoff.count(),10);
  a=r.next_attempt();EXPECT_EQ(a.backoff.count(),20);a=r.next_attempt();EXPECT_EQ(a.backoff.count(),25);
  a=r.next_attempt();EXPECT_FALSE(a.allowed);EXPECT_TRUE(a.latched);
  r.recovered();EXPECT_EQ(r.consecutive_timeouts(),0U);EXPECT_TRUE(r.next_attempt().allowed);
}
TEST(RgbRecoverySupervisor, GoodFrameClearsConsecutiveTimeouts){RecoverySupervisor r({2,1,std::chrono::milliseconds(1),std::chrono::milliseconds(1)});EXPECT_FALSE(r.observe_timeout());r.observe_frame();EXPECT_FALSE(r.observe_timeout());}
