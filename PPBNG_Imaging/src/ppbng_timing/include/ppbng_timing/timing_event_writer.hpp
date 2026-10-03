#pragma once

#include "ppbng_timing/protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace ppbng_timing {

enum class TimingLogCode { ok, unsafe_path, already_exists, open_failed,
                            invalid_record, write_failed, flush_failed };
struct TimingLogStatus {
  TimingLogCode code{TimingLogCode::ok};
  std::string detail;
  bool ok() const noexcept { return code == TimingLogCode::ok; }
};
struct TimingEventWriterOptions {
  std::uint64_t fail_after_total_bytes{(std::numeric_limits<std::uint64_t>::max)()};
  bool fail_flush{false};
};

class TimingEventWriter {
 public:
  static std::pair<TimingLogStatus, std::unique_ptr<TimingEventWriter>> create(
      const std::filesystem::path& session_root,
      const std::filesystem::path& relative_path = "segments/timing_events.bin",
      TimingEventWriterOptions options = {});
  ~TimingEventWriter();
  TimingEventWriter(const TimingEventWriter&) = delete;
  TimingEventWriter& operator=(const TimingEventWriter&) = delete;

  TimingLogStatus append_pps(const PpsAnchor& anchor,
                             std::uint64_t host_receive_monotonic_ns);
  TimingLogStatus append_trigger(const TriggerEvent& trigger,
                                 std::uint64_t host_receive_monotonic_ns);
  TimingLogStatus flush();
  std::uint64_t records_written() const noexcept { return records_written_; }

 private:
  struct Impl;
  TimingEventWriter(std::filesystem::path path, TimingEventWriterOptions options);
  TimingLogStatus open_exclusive();
  TimingLogStatus append_record(std::uint8_t type, const std::vector<std::uint8_t>& payload);
  TimingLogStatus write_all(const std::uint8_t* data, std::size_t size);

  std::filesystem::path path_;
  TimingEventWriterOptions options_;
  std::unique_ptr<Impl> impl_;
  std::uint64_t physical_bytes_{};
  std::uint64_t records_written_{};
};

struct TimingSessionBindResult {
  bool accepted{};
  bool duplicate{};
  std::string detail;
};

class TimingSessionBinding {
 public:
  explicit TimingSessionBinding(std::filesystem::path allowed_output_root)
      : allowed_output_root_(std::move(allowed_output_root)) {}
  TimingSessionBindResult prepare(const std::string& request_id,
      const std::string& session_id, const std::string& session_directory,
      bool task_active);
  bool bound() const noexcept { return bound_; }
  const std::string& session_id() const noexcept { return session_id_; }
  const std::filesystem::path& directory() const noexcept { return directory_; }
 private:
  struct Record { std::string session_id; std::string directory; TimingSessionBindResult result; };
  std::filesystem::path allowed_output_root_;
  bool bound_{};
  std::string session_id_;
  std::filesystem::path directory_;
  std::unordered_map<std::string, Record> history_;
};

}  // namespace ppbng_timing
