#include "ppbng_thermal/mock_thermal_backend.hpp"
#include "ppbng_thermal/runtime_contract.hpp"
#include "ppbng_thermal/start_cleanup_guard.hpp"
#include "ppbng_thermal/sequence_association.hpp"
#include "ppbng_thermal/recovery_supervisor.hpp"
#include <gtest/gtest.h>

using namespace ppbng_thermal;
namespace {
Status bring_to_configured(MockThermalBackend & backend, StopToken token)
{
  std::vector<std::string> ids;
  auto status=backend.discover(std::chrono::milliseconds(5),token,ids); if(!status.ok())return status;
  status=backend.open(ids.front(),std::chrono::milliseconds(5),token); if(!status.ok())return status;
  return backend.configure({},std::chrono::milliseconds(5),token);
}
}

TEST(ThermalAdapter, EnforcesLifecycleAndA6701Readback)
{
  StopSource source; MockThermalBackend backend;
  EXPECT_EQ(backend.arm(std::chrono::milliseconds(5),source.token()).code,ErrorCode::invalid_state);
  ASSERT_TRUE(bring_to_configured(backend,source.token()).ok());
  ThermalReadback readback; ASSERT_TRUE(backend.readback(std::chrono::milliseconds(5),source.token(),readback).ok());
  EXPECT_TRUE(validate_a6701_contract(readback).ok());
  EXPECT_EQ(readback.transport_height,513U); EXPECT_EQ(readback.image_height,512U);
  EXPECT_EQ(readback.payload_bytes,656640U); EXPECT_EQ(readback.frame_sync_source,"External");
  EXPECT_EQ(readback.frame_sync_polarity,"ActiveHigh");
  EXPECT_EQ(readback.frame_sync_evidence,FrameSyncEvidence::unverified);
  EXPECT_TRUE(backend.arm(std::chrono::milliseconds(5),source.token()).ok());
  auto frame=backend.next_frame(std::chrono::milliseconds(5),source.token());
  EXPECT_TRUE(frame.status.ok()); EXPECT_EQ(frame.frame.info().payload_bytes,656640U);
  EXPECT_EQ(frame.frame.info().camera_timestamp_ns,1000000U);
  EXPECT_NE(frame.frame.info().host_receive_monotonic.time_since_epoch().count(),0);
}

TEST(ThermalAdapter, RejectsAccessTemperatureAndReadbackMismatch)
{
  StopSource source;
  FakeThermalNodeMap denied; denied.access=AccessMode::read_only; MockThermalBackend no_access(denied);
  std::vector<std::string> ids; ASSERT_TRUE(no_access.discover(std::chrono::milliseconds(5),source.token(),ids).ok());
  EXPECT_EQ(no_access.open(ids[0],std::chrono::milliseconds(5),source.token()).code,ErrorCode::access_denied);

  FakeThermalNodeMap warm; warm.fpa_cold=false; MockThermalBackend not_cold(warm);
  EXPECT_EQ(bring_to_configured(not_cold,source.token()).code,ErrorCode::not_cold);

  FakeThermalNodeMap mismatch; mismatch.force_readback_mismatch=true; MockThermalBackend bad(mismatch);
  EXPECT_EQ(bring_to_configured(bad,source.token()).code,ErrorCode::readback_mismatch);
}

TEST(ThermalAdapter, RequiresFiniteTimeoutAndHonorsCancellation)
{
  StopSource source; MockThermalBackend backend; std::vector<std::string> ids;
  EXPECT_EQ(backend.discover(std::chrono::milliseconds(0),source.token(),ids).code,ErrorCode::timeout);
  source.request_stop();
  EXPECT_EQ(backend.discover(std::chrono::milliseconds(5),source.token(),ids).code,ErrorCode::cancelled);
}

TEST(ThermalAdapter, FrameLeaseReleasesExactlyOnceAcrossMovesAndExplicitRelease)
{
  StopSource source; MockThermalBackend backend; ASSERT_TRUE(bring_to_configured(backend,source.token()).ok());
  ASSERT_TRUE(backend.arm(std::chrono::milliseconds(5),source.token()).ok());
  {auto result=backend.next_frame(std::chrono::milliseconds(5),source.token()); FrameLease moved=std::move(result.frame); moved.release(); moved.release();}
  EXPECT_EQ(backend.release_count(),1U);
  {auto result=backend.next_frame(std::chrono::milliseconds(5),source.token()); EXPECT_TRUE(result.frame);}
  EXPECT_EQ(backend.release_count(),2U);
}

TEST(ThermalAdapter, ReportsIncompleteFramesAndStillLeasesTheirBuffers)
{
  StopSource source; FakeThermalNodeMap nodes; nodes.incomplete_every_n_frames=2; MockThermalBackend backend(nodes);
  ASSERT_TRUE(bring_to_configured(backend,source.token()).ok()); ASSERT_TRUE(backend.arm(std::chrono::milliseconds(5),source.token()).ok());
  EXPECT_TRUE(backend.next_frame(std::chrono::milliseconds(5),source.token()).status.ok());
  auto incomplete=backend.next_frame(std::chrono::milliseconds(5),source.token());
  EXPECT_EQ(incomplete.status.code,ErrorCode::incomplete_frame); EXPECT_FALSE(incomplete.frame.info().complete);
  EXPECT_EQ(incomplete.frame.size(),656640U);
}

TEST(ThermalAdapter, BoundedQueueDropsNewestAndReleasesIt)
{
  StopSource source; MockThermalBackend backend; ASSERT_TRUE(bring_to_configured(backend,source.token()).ok()); ASSERT_TRUE(backend.arm(std::chrono::milliseconds(5),source.token()).ok());
  BoundedFrameQueue queue(1); auto first=backend.next_frame(std::chrono::milliseconds(5),source.token()); auto second=backend.next_frame(std::chrono::milliseconds(5),source.token());
  EXPECT_TRUE(queue.try_push(std::move(first.frame))); EXPECT_FALSE(queue.try_push(std::move(second.frame)));
  EXPECT_EQ(backend.release_count(),1U); auto stats=queue.statistics(); EXPECT_EQ(stats.backpressure_events,1U); EXPECT_EQ(stats.dropped_newest,1U);
  auto popped=queue.pop(std::chrono::milliseconds(5),source.token()); EXPECT_TRUE(popped.status.ok()); EXPECT_EQ(popped.frame.info().frame_id,1U);
}

TEST(ThermalAdapter, RecoveryRequiresReconfigureAndStartsNewSegment)
{
  StopSource source; MockThermalBackend backend; ASSERT_TRUE(bring_to_configured(backend,source.token()).ok()); ASSERT_TRUE(backend.arm(std::chrono::milliseconds(5),source.token()).ok());
  auto first=backend.next_frame(std::chrono::milliseconds(5),source.token()); EXPECT_EQ(first.frame.info().segment_index,0U);
  ASSERT_TRUE(backend.recover(std::chrono::milliseconds(5),source.token()).ok()); EXPECT_EQ(backend.segment_index(),1U); EXPECT_EQ(backend.state(),LifecycleState::open);
  ASSERT_TRUE(backend.configure({},std::chrono::milliseconds(5),source.token()).ok()); ASSERT_TRUE(backend.arm(std::chrono::milliseconds(5),source.token()).ok());
  auto recovered=backend.next_frame(std::chrono::milliseconds(5),source.token()); EXPECT_EQ(recovered.frame.info().segment_index,1U); EXPECT_EQ(recovered.frame.info().frame_id,1U);
}

TEST(ThermalAdapter, ConfigurationHashChangesForTimingOrLayout)
{
  ThermalConfiguration a,b; EXPECT_EQ(configuration_hash(a),configuration_hash(b));
  b.frame_sync_mode="Readout"; EXPECT_NE(configuration_hash(a),configuration_hash(b));
  b=a; b.frame_sync_polarity="ActiveLow"; EXPECT_NE(configuration_hash(a),configuration_hash(b));
  b=a; b.payload_bytes=1; EXPECT_NE(configuration_hash(a),configuration_hash(b));
}

TEST(ThermalAdapter, AcceptsOnlyExplicitSupportedSyncModesAndPolarity)
{
  ThermalReadback readback;
  readback.access=AccessMode::control;
  readback.ready=true;
  readback.fpa_cold=true;
  readback.frame_sync_mode="Readout";
  readback.frame_sync_polarity="ActiveLow";
  EXPECT_TRUE(validate_a6701_contract(readback).ok());

  readback.frame_sync_mode="FSSI";
  EXPECT_EQ(validate_a6701_contract(readback).code,ErrorCode::invalid_configuration);
  readback.frame_sync_mode="Integration";
  readback.frame_sync_polarity="Unknown";
  EXPECT_EQ(validate_a6701_contract(readback).code,ErrorCode::invalid_configuration);
}
TEST(ThermalRuntimeContract, PreservesOpaqueMetadataRowAndFullPayload)
{
  FrameInfo info{17,2,656640,true,8'000'000,
    std::chrono::steady_clock::time_point(std::chrono::nanoseconds(9'000'000))};
  ThermalReadback readback;readback.access=AccessMode::control;readback.ready=true;readback.fpa_cold=true;
  const auto d=describe_runtime_frame(info,readback);
  EXPECT_EQ(d.transport_width,640U);EXPECT_EQ(d.transport_height,513U);
  EXPECT_EQ(d.payload_size_bytes,656640U);EXPECT_EQ(d.auxiliary_data_offset_bytes,0U);
  EXPECT_EQ(d.auxiliary_data_length_bytes,1280U);EXPECT_EQ(d.image_data_offset_bytes,1280U);
  EXPECT_EQ(d.image_data_length_bytes,655360U);EXPECT_EQ(d.image_width,640U);EXPECT_EQ(d.image_height,512U);
  EXPECT_EQ(d.sdk_frame_counter,17U);EXPECT_EQ(d.camera_timestamp_ns,8'000'000U);
  EXPECT_EQ(d.host_receive_monotonic_ns,9'000'000U);EXPECT_TRUE(d.raw_payload_preserved);
  EXPECT_EQ(d.validation_detail,"full 640x513 payload preserved; first row remains opaque");
}
TEST(ThermalRuntimeContract, FailedStartCleanupRunsExactlyOnceUnlessCommitted)
{
  int releases=0;{StartCleanupGuard guard([&](){++releases;});}EXPECT_EQ(releases,1);
  {StartCleanupGuard guard([&](){++releases;});guard.commit();}EXPECT_EQ(releases,1);
}
TEST(ThermalSequenceAssociation, FrameFirstAndTriggerFirstMapOnlyByCounterDelta)
{
  SequenceAssociator a(3,50,true);a.reset(2);a.frame({700,1,2,1});
  auto x=a.trigger({20,7,4,100,3,2,123,2});ASSERT_EQ(x.settlements.size(),1U);
  a.trigger({21,7,5,100,3,3,124,2});auto y=a.frame({701,2,2,4});ASSERT_EQ(y.settlements.size(),1U);EXPECT_EQ(y.settlements[0].trigger->sequence,21U);
  EXPECT_FALSE(a.trigger({20,7,0,100,3,5,0,0}).accepted);
  a.frame({702,3,2,10});auto e=a.expire(60);ASSERT_EQ(e.settlements.size(),1U);EXPECT_EQ(e.settlements[0].kind,SettlementKind::unmatched);
}
TEST(ThermalSequenceAssociation, MissedTriggerAmbiguityIsUnverifiedByDefault){
  SequenceAssociator a(4,100);a.reset(1);a.frame({20,1,1,1});auto r=a.trigger({10,1,0,1000,0,2,0,0});ASSERT_EQ(r.settlements.size(),1U);EXPECT_EQ(r.settlements[0].kind,SettlementKind::consistent_unverified);EXPECT_NE(r.settlements[0].detail.find("ambiguity"),std::string::npos);
}
TEST(ThermalArmPreflight, IsPureAndFailClosed){ThermalConfiguration c;c.frame_sync_mode="Integration";c.frame_sync_polarity="ActiveHigh";EXPECT_FALSE(validate_inert_arm_preflight(false,true,"id","A6701","thermal",c,8,100).ok());EXPECT_FALSE(validate_inert_arm_preflight(true,false,"id","A6701","thermal",c,8,100).ok());EXPECT_TRUE(validate_inert_arm_preflight(true,true,"id","A6701","thermal",c,8,100).ok());}
TEST(ThermalRecoverySupervisor, ThresholdBackoffCapAndBoundedAttempts){RecoverySupervisor r({2,3,std::chrono::milliseconds(10),std::chrono::milliseconds(25)});EXPECT_FALSE(r.observe_timeout());EXPECT_TRUE(r.observe_timeout());r.begin_episode();EXPECT_EQ(r.next_attempt().backoff.count(),10);EXPECT_EQ(r.next_attempt().backoff.count(),20);EXPECT_EQ(r.next_attempt().backoff.count(),25);auto x=r.next_attempt();EXPECT_FALSE(x.allowed);EXPECT_TRUE(x.latched);}
TEST(ThermalRecoverySupervisor, FrameAndRecoveryResetState){RecoverySupervisor r({2,2,std::chrono::milliseconds(1),std::chrono::milliseconds(2)});r.observe_timeout();r.observe_frame();EXPECT_EQ(r.consecutive_timeouts(),0U);r.begin_episode();r.next_attempt();r.recovered();EXPECT_EQ(r.next_attempt().number,1U);}
