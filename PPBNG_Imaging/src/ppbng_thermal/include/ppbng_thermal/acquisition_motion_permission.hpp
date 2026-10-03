#pragma once
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace ppbng_thermal {
// A latched recording-health gate, independent of the recoverable NUC gate.
// Only a new nonempty session resets faults. Duplicate/old samples do not renew liveness.
class AcquisitionMotionPermission {
public:
  explicit AcquisitionMotionPermission(const std::vector<std::string>& devices) {
    for (const auto& id : devices) streams_.emplace(id, Stream{});
  }
  void manager(const std::string& session, bool recording, bool degraded, std::uint64_t now) {
    if (!session.empty() && session != session_) {
      session_ = session; fault_.clear(); permitted_once_ = false;
      for (auto& entry : streams_) entry.second = {};
    }
    const bool next_recording = recording && !session.empty();
    if (next_recording && !recording_) {
      for (auto& entry : streams_) entry.second = {};
    }
    recording_ = next_recording; manager_seen_ = true; manager_time_ = now;
    if (degraded && !session.empty()) fault(session, "acquisition manager reported warnings");
  }
  void sample(const std::string& session, const std::string& id,
              std::uint64_t sequence, std::uint64_t now) {
    if (!recording_ || session.empty() || session != session_) return;
    auto entry = streams_.find(id); if (entry == streams_.end()) return;
    auto& stream = entry->second;
    if (!stream.seen || sequence > stream.sequence) stream = {true, sequence, now};
  }
  void fault(const std::string& session, const std::string& reason) {
    if (!session.empty() && session == session_ && fault_.empty()) fault_ = reason;
  }
  bool permitted(std::uint64_t now, std::uint64_t timeout) {
    if (!fault_.empty() || !recording_) return false;
    std::string missing;
    if (!manager_seen_ || now < manager_time_ || now - manager_time_ > timeout)
      missing = "acquisition manager heartbeat stale";
    for (const auto& entry : streams_) {
      const auto& s = entry.second;
      if (!s.seen || now < s.time || now - s.time > timeout) {
        missing = "sample stream stale or missing: " + entry.first; break;
      }
    }
    if (!missing.empty()) {
      if (permitted_once_) fault_ = missing;
      return false;
    }
    permitted_once_ = true; return true;
  }
  const std::string& reason() const { return fault_; }
  const std::string& session() const { return session_; }
private:
  struct Stream { bool seen{false}; std::uint64_t sequence{0}, time{0}; };
  std::map<std::string,Stream> streams_;
  std::string session_, fault_;
  bool recording_{false}, manager_seen_{false}, permitted_once_{false};
  std::uint64_t manager_time_{0};
};
}
