#pragma once
#include "ppbng_core/gnss_association.hpp"
#include "ppbng_core/rsm_attitude_association.hpp"
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>

namespace ppbng_core {
enum class ContextAvailability : std::uint8_t { unavailable, available, degraded };
struct GnssHeadingSample {std::uint64_t sequence{};std::int64_t utc_nanoseconds{};std::uint64_t host_monotonic_ns{};double heading_deg{};double pitch_deg{};bool valid{};};
struct FrameContextQuery {std::int64_t utc_nanoseconds{};std::uint64_t host_monotonic_ns{};};
struct FrameContextResult {
 ContextAvailability gnss_status{ContextAvailability::unavailable};std::string gnss_detail;
 std::optional<AssociatedGnss> gnss;std::optional<double> heading_deg,pitch_deg;
 std::uint64_t heading_before_sequence{},heading_after_sequence{};
 ContextAvailability rsm_status{ContextAvailability::unavailable};std::string rsm_detail;
 std::optional<AssociatedRsmAttitude> rsm;
};
class FrameContextAssociator {
public:
 FrameContextAssociator(std::uint64_t gnss_max_span_ns,std::uint64_t gnss_max_age_ns,std::uint64_t rsm_max_age_ns,std::size_t capacity=256);
 bool add_gnss_position(const GnssObservation&sample);
 bool add_gnss_heading(const GnssHeadingSample&sample);
 bool add_rsm(const RsmAttitudeSample&sample);
 FrameContextResult associate(const FrameContextQuery&query)const;
 void reset();
private:
 std::uint64_t gnss_max_span_ns_,gnss_max_age_ns_,rsm_max_age_ns_;std::size_t capacity_;
 std::deque<GnssObservation>positions_;std::deque<GnssHeadingSample>headings_;RsmAttitudeAssociator rsm_;
};
}
