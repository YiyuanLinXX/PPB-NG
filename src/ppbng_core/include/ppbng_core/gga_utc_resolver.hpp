#pragma once
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string_view>
namespace ppbng_core {
struct GnssUtcAnchor {std::uint64_t sequence{},connection_epoch{},host_monotonic_ns{};std::int64_t utc_nanoseconds{};};
struct ResolvedGgaUtc {std::int64_t utc_nanoseconds{};std::uint64_t anchor_sequence{},host_delta_ns{},clock_residual_ns{};};
class GgaUtcResolver {
public:
 GgaUtcResolver(std::uint64_t maximum_anchor_age_ns,std::uint64_t maximum_clock_residual_ns);
 bool observe_anchor(const GnssUtcAnchor&anchor);
 std::optional<ResolvedGgaUtc> resolve(std::string_view hhmmss,std::uint64_t host_ns,std::uint64_t connection_epoch)const;
 void reset();
private:
 static constexpr std::size_t capacity_=64;
 std::uint64_t max_age_,max_residual_;
 std::deque<GnssUtcAnchor> anchors_;
};
}
