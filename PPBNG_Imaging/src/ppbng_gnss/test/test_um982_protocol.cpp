#include "ppbng_gnss/um982_protocol.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr char kValidGga[] =
    "$GNGGA,123519.00,4250.000000,N,07630.000000,W,4,20,"
    "0.7,100.000,M,-30.000,M,0.8,0001*64";

constexpr char kValidHeading[] =
    "#UNIHEADINGA,97,GPS,FINE,2190,365174000,0,0,18,12;"
    "SOL_COMPUTED,NARROW_INT,1.0000,90.0000,10.0000,0.0000,"
    "0.1000,0.2000,\"0\",20,16,18,12,0,00,0,0*5cdb44d1";

TEST(Um982Protocol, ParsesRtkFixedGgaAndPreservesRawSentence) {
  const auto fix = ppbng_gnss::parse_gga(std::string(kValidGga) + "\r\n");
  EXPECT_EQ(fix.utc_hhmmss, "123519.00");
  EXPECT_NEAR(fix.latitude_deg, 42.8333333333, 1e-10);
  EXPECT_NEAR(fix.longitude_deg, -76.5, 1e-10);
  EXPECT_DOUBLE_EQ(fix.altitude_m, 100.0);
  EXPECT_EQ(fix.quality, 4);
  EXPECT_EQ(fix.satellites_used, 20);
  EXPECT_DOUBLE_EQ(fix.hdop, 0.7);
  ASSERT_TRUE(fix.has_differential_age);
  EXPECT_DOUBLE_EQ(fix.differential_age_sec, 0.8);
  EXPECT_EQ(fix.raw_sentence, kValidGga);
}

TEST(Um982Protocol, RejectsBadGgaChecksum) {
  std::string sentence(kValidGga);
  sentence.replace(sentence.size() - 2U, 2U, "00");
  EXPECT_THROW(ppbng_gnss::parse_gga(sentence), std::invalid_argument);
}

TEST(Um982Protocol, RejectsTruncatedGga) {
  EXPECT_THROW(
      ppbng_gnss::parse_gga("$GNGGA,123519.00,4250.0,N"),
      std::invalid_argument);
}

TEST(Um982Protocol, RejectsInvalidCoordinateMinutes) {
  const std::string payload =
      "GNGGA,123519.00,4260.000000,N,07630.000000,W,4,20,"
      "0.7,100.000,M,-30.000,M,0.8,0001";
  char checksum[3]{};
  std::snprintf(checksum, sizeof(checksum), "%02X", ppbng_gnss::nmea_checksum(payload));
  EXPECT_THROW(
      ppbng_gnss::parse_gga("$" + payload + "*" + checksum),
      std::invalid_argument);
}

TEST(Um982Protocol, ParsesUniheadingAndPreservesQualityFields) {
  const auto solution = ppbng_gnss::parse_uniheadinga(kValidHeading);
  EXPECT_EQ(solution.receiver_time.cpu_idle_percent, 97);
  EXPECT_EQ(solution.receiver_time.time_reference, "GPS");
  EXPECT_EQ(solution.receiver_time.time_status, "FINE");
  EXPECT_EQ(solution.receiver_time.week, 2190U);
  EXPECT_EQ(solution.receiver_time.milliseconds_of_week, 365174000U);
  EXPECT_EQ(solution.receiver_time.leap_seconds, 18U);
  EXPECT_EQ(solution.receiver_time.output_delay_ms, 12U);
  ASSERT_TRUE(solution.receiver_time.utc_unix_nanoseconds.has_value());
  EXPECT_EQ(*solution.receiver_time.utc_unix_nanoseconds, 1640841956000000000LL);
  EXPECT_EQ(solution.solution_status, "SOL_COMPUTED");
  EXPECT_EQ(solution.position_type, "NARROW_INT");
  EXPECT_DOUBLE_EQ(solution.baseline_m, 1.0);
  EXPECT_DOUBLE_EQ(solution.heading_deg, 90.0);
  EXPECT_DOUBLE_EQ(solution.pitch_deg, 10.0);
  EXPECT_DOUBLE_EQ(solution.heading_stddev_deg, 0.1);
  EXPECT_DOUBLE_EQ(solution.pitch_stddev_deg, 0.2);
  EXPECT_EQ(solution.satellites_tracked, 20);
  EXPECT_EQ(solution.satellites_used, 16);
  EXPECT_EQ(solution.raw_sentence, kValidHeading);
}

TEST(Um982Protocol, UnknownReceiverTimeIsPreservedButNotPromotedToUtc) {
  std::string payload =
    "UNIHEADINGA,97,GPS,UNKNOWN,2190,365174000,0,0,18,12;"
    "SOL_COMPUTED,NARROW_INT,1.0000,90.0000,10.0000,0.0000,"
    "0.1000,0.2000,\"0\",20,16,18,12,0,00,0,0";
  char crc[9]{};
  std::snprintf(crc, sizeof(crc), "%08x", ppbng_gnss::unicore_crc32(payload));
  const auto solution = ppbng_gnss::parse_uniheadinga("#" + payload + "*" + crc);
  EXPECT_EQ(solution.receiver_time.time_status, "UNKNOWN");
  EXPECT_FALSE(solution.receiver_time.utc_unix_nanoseconds.has_value());
}

TEST(Um982Protocol, RejectsOutOfRangeReceiverWeekTime) {
  std::string payload =
    "UNIHEADINGA,97,GPS,FINE,2190,604800000,0,0,18,12;"
    "SOL_COMPUTED,NARROW_INT,1.0000,90.0000,10.0000,0.0000,"
    "0.1000,0.2000,\"0\",20,16,18,12,0,00,0,0";
  char crc[9]{};
  std::snprintf(crc, sizeof(crc), "%08x", ppbng_gnss::unicore_crc32(payload));
  EXPECT_THROW(ppbng_gnss::parse_uniheadinga("#" + payload + "*" + crc),
    std::invalid_argument);
}

TEST(Um982Protocol, RejectsBadUniheadingCrc) {
  std::string sentence(kValidHeading);
  sentence.replace(sentence.size() - 8U, 8U, "00000000");
  EXPECT_THROW(ppbng_gnss::parse_uniheadinga(sentence), std::invalid_argument);
}

TEST(Um982Protocol, RejectsTruncatedUniheading) {
  EXPECT_THROW(
      ppbng_gnss::parse_uniheadinga(
          "#UNIHEADINGA,97,GPS,FINE;SOL_COMPUTED,NARROW_INT*12345678"),
      std::invalid_argument);
}

TEST(IncrementalLineDecoder, FramesSplitAndCoalescedCrLfInput) {
  ppbng_gnss::IncrementalLineDecoder decoder;
  EXPECT_TRUE(decoder.feed("$GNG").empty());
  const auto first = decoder.feed("GA,one\r\n#UNI");
  ASSERT_EQ(first.size(), 1U);
  EXPECT_EQ(first[0], "$GNGGA,one");
  EXPECT_EQ(decoder.buffered_bytes(), 4U);

  const auto remainder = decoder.feed("HEADINGA,two\nthird\n");
  EXPECT_EQ(remainder, (std::vector<std::string>{"#UNIHEADINGA,two", "third"}));
  EXPECT_EQ(decoder.buffered_bytes(), 0U);
}

TEST(IncrementalLineDecoder, RejectsUnboundedPartialLineAndRecovers) {
  ppbng_gnss::IncrementalLineDecoder decoder(8U);
  EXPECT_TRUE(decoder.feed("12345678").empty());
  EXPECT_THROW(decoder.feed("9"), std::length_error);
  EXPECT_EQ(decoder.buffered_bytes(), 0U);
  EXPECT_EQ(decoder.feed("ok\n"), (std::vector<std::string>{"ok"}));
}

}  // namespace
