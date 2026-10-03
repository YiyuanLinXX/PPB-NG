#include "ppbng_storage/segment_io.hpp"
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <vector>
#include <gtest/gtest.h>

namespace
{
using namespace ppbng_storage;
class TempSession
{
public:
  TempSession(){auto n=std::chrono::steady_clock::now().time_since_epoch().count();root_=std::filesystem::temp_directory_path()/("ppbng_segment_test_"+std::to_string(n));if(!std::filesystem::create_directory(root_)||!std::filesystem::create_directory(root_/"segments"))throw std::runtime_error("temp create failed");}
  ~TempSession(){std::error_code e;std::filesystem::remove_all(root_,e);} const std::filesystem::path&root()const{return root_;}
private:std::filesystem::path root_;
};
FrameEnvelope envelope(std::uint64_t id){return{id,id+100,42,1'700'000'000'000'000'000LL+static_cast<std::int64_t>(id),TimeQuality::locked};}
std::vector<std::byte> pattern(std::size_t n){std::vector<std::byte>v(n);for(std::size_t i=0;i<n;++i)v[i]=std::byte((i*37U+11U)&0xffU);return v;}
}

TEST(SegmentIo, Crc32MatchesStandardIeeeKnownVectors)
{
  const std::array<std::byte,9> digits{
    std::byte{'1'},std::byte{'2'},std::byte{'3'},std::byte{'4'},std::byte{'5'},
    std::byte{'6'},std::byte{'7'},std::byte{'8'},std::byte{'9'}};
  EXPECT_EQ(payload_crc32(nullptr,0U),0U);
  EXPECT_EQ(payload_crc32(digits.data(),digits.size()),0xcbf43926U);
}

TEST(SegmentIo, RoundTripsBitExactA6701PayloadAndEnvelope)
{
  TempSession t;auto payload=pattern(656'640);auto created=SegmentWriter::create(t.root(),"segments/thermal_000000.ppbseg");ASSERT_TRUE(created.first.ok());ASSERT_TRUE(created.second->append(envelope(7),payload.data(),payload.size()).ok());auto checkpoint=created.second->checkpoint();ASSERT_TRUE(checkpoint.first.ok());EXPECT_EQ(checkpoint.second.committed_records,1U);created.second.reset();
  auto opened=SegmentReader::open(t.root(),"segments/thermal_000000.ppbseg");ASSERT_TRUE(opened.first.ok());SegmentFrame frame;ASSERT_TRUE(opened.second->next(frame).ok());EXPECT_EQ(frame.envelope.sample_id,7U);EXPECT_EQ(frame.envelope.trigger_id,107U);EXPECT_EQ(frame.envelope.pps_sequence,42U);EXPECT_EQ(frame.envelope.time_quality,TimeQuality::locked);EXPECT_EQ(frame.payload,payload);EXPECT_EQ(opened.second->next(frame).error,SegmentIoError::end_of_file);
}

TEST(SegmentIo, SupportsSmallRawBayerPayloadAndNeverOverwrites)
{
  TempSession t;auto payload=pattern(64);auto first=SegmentWriter::create(t.root(),"segments/rgb_000000.ppbseg");ASSERT_TRUE(first.first.ok());ASSERT_TRUE(first.second->append(envelope(1),payload.data(),payload.size()).ok());first.second.reset();auto duplicate=SegmentWriter::create(t.root(),"segments/rgb_000000.ppbseg");EXPECT_EQ(duplicate.first.error,SegmentIoError::already_exists);auto reader=SegmentReader::open(t.root(),"segments/rgb_000000.ppbseg");ASSERT_TRUE(reader.first.ok());SegmentFrame frame;ASSERT_TRUE(reader.second->next(frame).ok());EXPECT_EQ(frame.payload,payload);
}

TEST(SegmentIo, RejectsPathsOutsideSessionSegments)
{
  TempSession t;EXPECT_EQ(SegmentWriter::create(t.root(),"../escape.bin").first.error,SegmentIoError::unsafe_path);EXPECT_EQ(SegmentReader::open(t.root(),"absolute.bin").first.error,SegmentIoError::unsafe_path);
}

TEST(SegmentIo, RecoversTruncatedTailToLastCommittedBoundary)
{
  TempSession t;auto payload=pattern(128);std::uint64_t committed=0;{auto w=SegmentWriter::create(t.root(),"segments/truncated.ppbseg");ASSERT_TRUE(w.first.ok());ASSERT_TRUE(w.second->append(envelope(1),payload.data(),payload.size()).ok());committed=w.second->committed_bytes();ASSERT_TRUE(w.second->append(envelope(2),payload.data(),payload.size()).ok());}
  auto file=t.root()/"segments/truncated.ppbseg";std::filesystem::resize_file(file,committed+40);std::uint64_t recovered=0;auto status=recover_truncated_segment(t.root(),"segments/truncated.ppbseg",recovered);ASSERT_TRUE(status.ok())<<status.detail;EXPECT_EQ(recovered,committed);EXPECT_EQ(std::filesystem::file_size(file),committed);auto r=SegmentReader::open(t.root(),"segments/truncated.ppbseg");ASSERT_TRUE(r.first.ok());EXPECT_TRUE(r.second->scan_to_end().ok());
}

TEST(SegmentIo, ReportsChecksumCorruptionWithoutTruncating)
{
  TempSession t;auto payload=pattern(256);{auto w=SegmentWriter::create(t.root(),"segments/corrupt.ppbseg");ASSERT_TRUE(w.first.ok());ASSERT_TRUE(w.second->append(envelope(1),payload.data(),payload.size()).ok());}
  auto file=t.root()/"segments/corrupt.ppbseg";auto before=std::filesystem::file_size(file);std::fstream io(file,std::ios::binary|std::ios::in|std::ios::out);io.seekg(16+72+10);char value=0;io.read(&value,1);io.clear();io.seekp(16+72+10);value^=0x55;io.write(&value,1);io.close();auto r=SegmentReader::open(t.root(),"segments/corrupt.ppbseg");ASSERT_TRUE(r.first.ok());EXPECT_EQ(r.second->scan_to_end().error,SegmentIoError::checksum_error);std::uint64_t recovered=0;EXPECT_EQ(recover_truncated_segment(t.root(),"segments/corrupt.ppbseg",recovered).error,SegmentIoError::checksum_error);EXPECT_EQ(std::filesystem::file_size(file),before);
}

TEST(SegmentIo, InjectedWriteFailureLeavesRecoverableUncommittedTail)
{
  TempSession t;auto payload=pattern(256);{SegmentWriterOptions options;options.fail_after_total_bytes=16+72+31;auto w=SegmentWriter::create(t.root(),"segments/diskfull.ppbseg",options);ASSERT_TRUE(w.first.ok());EXPECT_EQ(w.second->append(envelope(1),payload.data(),payload.size()).error,SegmentIoError::write_failed);}
  std::uint64_t recovered=0;auto status=recover_truncated_segment(t.root(),"segments/diskfull.ppbseg",recovered);ASSERT_TRUE(status.ok());EXPECT_EQ(recovered,16U);EXPECT_EQ(std::filesystem::file_size(t.root()/"segments/diskfull.ppbseg"),16U);
}

TEST(SegmentIo, RolloverPolicyAccountsForEnvelopeAndOverflow)
{
  SegmentRolloverPolicy policy(1000);EXPECT_FALSE(policy.should_rollover(16,100));EXPECT_TRUE(policy.should_rollover(900,100));EXPECT_TRUE(policy.should_rollover(0,std::numeric_limits<std::uint64_t>::max()));
}
