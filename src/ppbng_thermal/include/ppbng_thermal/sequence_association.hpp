#pragma once
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>
namespace ppbng_thermal {struct AssociationFrame{std::uint64_t frame_id{},sample_id{},segment_id{},seen_ns{};};struct AssociationTrigger{std::uint64_t sequence{},pps_sequence{},offset_ticks{},ticks_per_second{},uncertainty_ns{},seen_ns{};std::int64_t utc_ns{};std::uint8_t time_status{};};enum class SettlementKind{matched,consistent_unverified,unmatched};struct AssociationSettlement{SettlementKind kind{SettlementKind::unmatched};AssociationFrame frame{};std::optional<AssociationTrigger>trigger;std::string detail;};struct AssociationResult{bool accepted{true};bool degraded{false};std::string detail;std::vector<AssociationSettlement>settlements;};class SequenceAssociator{public:SequenceAssociator(std::size_t,std::uint64_t,bool evidence_confirmed=false);AssociationResult reset(std::uint64_t);AssociationResult frame(AssociationFrame);AssociationResult trigger(AssociationTrigger);AssociationResult expire(std::uint64_t);private:AssociationResult drain();void append(AssociationResult&,AssociationResult&&);std::size_t capacity_;std::uint64_t wait_ns_,segment_{};bool evidence_confirmed_{};std::optional<std::uint64_t>frame_anchor_,trigger_anchor_,last_frame_,last_trigger_;std::map<std::uint64_t,AssociationFrame>frames_;std::map<std::uint64_t,AssociationTrigger>triggers_;};}
