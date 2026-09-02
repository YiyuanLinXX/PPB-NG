#pragma once

#include "ppbng_core/pps_time_mapper.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace ppbng_core {

constexpr std::int64_t kUnknownUtcSecond = (std::numeric_limits<std::int64_t>::min)();

struct AuthorityPpsAnchor {
  std::uint32_t boot_id{};
  std::uint64_t pps_sequence{};
  std::uint64_t captured_tick{};
  std::uint64_t host_receive_monotonic_ns{};
  std::uint8_t lock{};  // PpsAnchor: UNSYNCED=0, LOCKED=1, HOLDOVER=2.
};

struct AuthorityGnssEvidence {
  std::uint64_t host_receive_monotonic_ns{};
  std::uint64_t connection_epoch{};
  std::int64_t utc_second{kUnknownUtcSecond};
  std::uint16_t receiver_output_delay_ms{};
  bool uniheadinga{};
  bool fine{};
};

struct AuthorityTrigger {
  std::string channel;
  std::uint64_t channel_sequence{};
  std::uint64_t pps_sequence{};
  std::uint64_t offset_ticks{};
  std::uint64_t ticks_per_second{};
  std::uint64_t arrival_monotonic_ns{};
  std::uint8_t controller_time_status{};  // TimeQuality: UNSYNCED=0, HOLDOVER=1, LOCKED=2.
};

struct CorrelatedTrigger {
  AuthorityTrigger trigger;
  std::int64_t utc_nanoseconds{};
  std::uint64_t uncertainty_nanoseconds{};
  TimeStatus status{TimeStatus::unsynced};
  std::string detail;
};

struct TimeAuthorityConfiguration {
  std::size_t maximum_pps_queue{16U};
  std::size_t maximum_gnss_queue{16U};
  std::size_t maximum_pending_triggers{1024U};
  std::uint64_t maximum_host_skew_ns{250'000'000ULL};
  std::int64_t expected_system_offset_ns{};
  std::uint64_t trigger_wait_timeout_ns{500'000'000ULL};
  std::uint64_t unverified_uncertainty_ns{250'000'000ULL};
  std::uint64_t confirmed_uncertainty_ns{1'000'000ULL};
  bool pps_gnss_evidence_confirmed{false};
  bool pps_required{true};
};

struct TimeAuthorityStatistics {
  std::uint64_t anchors_correlated{};
  std::uint64_t anchors_missed{};
  std::uint64_t ambiguous_candidates{};
  std::uint64_t reordered_inputs{};
  std::uint64_t trigger_timeouts{};
  std::uint64_t queue_overflows{};
  std::uint64_t invalid_inputs{};
};

class TimeAuthorityCorrelator {
 public:
  explicit TimeAuthorityCorrelator(TimeAuthorityConfiguration configuration = {});

  std::vector<CorrelatedTrigger> add_pps(const AuthorityPpsAnchor& anchor);
  std::vector<CorrelatedTrigger> add_gnss(const AuthorityGnssEvidence& evidence);
  std::vector<CorrelatedTrigger> add_trigger(const AuthorityTrigger& trigger);
  std::vector<CorrelatedTrigger> advance(std::uint64_t now_monotonic_ns);

  [[nodiscard]] const TimeAuthorityStatistics& statistics() const noexcept { return statistics_; }
  [[nodiscard]] std::size_t pending_trigger_count() const noexcept { return triggers_.size(); }

 private:
  struct MappedAnchor {
    AuthorityPpsAnchor anchor;
    std::int64_t utc_second{};
  };
  struct PendingPps { AuthorityPpsAnchor value; bool ambiguity_reported{}; };
  struct PendingGnss { AuthorityGnssEvidence value; };

  void correlate();
  std::vector<CorrelatedTrigger> release_ready();
  CorrelatedTrigger map(const AuthorityTrigger& trigger) const;
  CorrelatedTrigger unsynced(const AuthorityTrigger& trigger, const std::string& reason) const;
  std::optional<std::uint64_t> corrected_gnss_host(const AuthorityGnssEvidence& value) const;

  TimeAuthorityConfiguration configuration_;
  TimeAuthorityStatistics statistics_;
  std::deque<PendingPps> pps_;
  std::deque<PendingGnss> gnss_;
  std::deque<AuthorityTrigger> triggers_;
  std::deque<MappedAnchor> mapped_;
  std::optional<std::uint32_t> current_boot_id_;
  std::optional<std::uint64_t> last_pps_sequence_;
  std::optional<std::uint64_t> last_gnss_epoch_;
  std::optional<std::int64_t> last_gnss_utc_second_;
  std::optional<std::uint64_t> last_mapped_sequence_;
  std::optional<std::int64_t> last_mapped_utc_second_;
};

}  // namespace ppbng_core
