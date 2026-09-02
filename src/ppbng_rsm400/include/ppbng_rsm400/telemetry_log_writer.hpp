#pragma once

#include "ppbng_rsm400/mcp2_protocol.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <utility>

namespace ppbng_rsm400
{
struct TelemetryLogOptions
{
  std::filesystem::path segments_directory;
  std::string stem{"rsm400"};
  std::uint64_t flush_every_records{10U};
  std::uint64_t fail_after_records{(std::numeric_limits<std::uint64_t>::max)()};
};
struct TelemetryLogResult {bool success{}; std::string detail;};
class TelemetryLogWriter
{
public:
  static std::pair<TelemetryLogResult, std::unique_ptr<TelemetryLogWriter>> create(
    const TelemetryLogOptions & options);
  ~TelemetryLogWriter();
  TelemetryLogResult append(const Frame & frame, const Telemetry & telemetry,
    std::uint64_t host_system_ns, std::uint64_t host_monotonic_ns, std::uint32_t segment_id);
  TelemetryLogResult flush();
  TelemetryLogResult close();
private:
  explicit TelemetryLogWriter(TelemetryLogOptions options) : options_(std::move(options)) {}
  TelemetryLogResult open();
  TelemetryLogResult checkpoint();
  TelemetryLogOptions options_;
  std::ofstream stream_;
  std::uint64_t committed_{};
  bool closed_{};
};
}  // namespace ppbng_rsm400
