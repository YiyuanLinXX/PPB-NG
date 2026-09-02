#include "ppbng_rsm400/telemetry_log_writer.hpp"

#include <iomanip>
#include <sstream>
#include <system_error>

#include <ppbng_storage/dataset_session.hpp>

#ifdef _WIN32
#include <windows.h>
#endif

namespace ppbng_rsm400
{
namespace
{
bool reserve_exclusive(const std::filesystem::path & path, std::string & error)
{
#ifdef _WIN32
  const HANDLE handle = CreateFileW(path.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_NEW,
    FILE_ATTRIBUTE_NORMAL, nullptr);
  if (handle == INVALID_HANDLE_VALUE) {
    error = "exclusive create failed (Win32 " + std::to_string(GetLastError()) + ")";
    return false;
  }
  CloseHandle(handle);
  return true;
#else
  (void)path; error = "RSM log exclusive create is currently implemented for Windows only";
  return false;
#endif
}

std::string escape(std::string_view value)
{
  std::ostringstream out;
  for (const unsigned char byte : value) {
    if (byte == '"') out << "\\\"";
    else if (byte == '\\') out << "\\\\";
    else if (byte == '\r') out << "\\r";
    else if (byte == '\n') out << "\\n";
    else if (byte < 0x20U) out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                              << static_cast<unsigned>(byte) << std::dec;
    else out << static_cast<char>(byte);
  }
  return out.str();
}

template<typename T>
void optional_number(std::ostream & out, const std::optional<T> & value)
{
  if (value) out << *value; else out << "null";
}
}  // namespace

std::pair<TelemetryLogResult, std::unique_ptr<TelemetryLogWriter>>
TelemetryLogWriter::create(const TelemetryLogOptions & options)
{
  if (options.stem.empty() || options.flush_every_records == 0U ||
    !std::filesystem::is_directory(options.segments_directory))
  {
    return std::make_pair(TelemetryLogResult{false, "invalid RSM telemetry writer configuration"},
      std::unique_ptr<TelemetryLogWriter>{});
  }
  auto writer = std::unique_ptr<TelemetryLogWriter>(new TelemetryLogWriter(options));
  const auto result = writer->open();
  if (!result.success) return std::make_pair(result, std::unique_ptr<TelemetryLogWriter>{});
  return std::make_pair(TelemetryLogResult{true, "RSM telemetry log exclusively created"},
    std::move(writer));
}

TelemetryLogResult TelemetryLogWriter::open()
{
  const auto path = options_.segments_directory / (options_.stem + ".telemetry.jsonl");
  const auto checkpoint_path = options_.segments_directory / (options_.stem + ".checkpoint");
  std::string error;
  if (!reserve_exclusive(path, error)) return {false, error};
  if (!reserve_exclusive(checkpoint_path, error)) return {false, error};
  stream_.open(path, std::ios::app);
  return stream_ ? TelemetryLogResult{true, "RSM log opened"} :
    TelemetryLogResult{false, "reserved RSM log could not be opened"};
}

TelemetryLogResult TelemetryLogWriter::append(const Frame & frame, const Telemetry & telemetry,
  const std::uint64_t host_system_ns, const std::uint64_t host_monotonic_ns,
  const std::uint32_t segment_id)
{
  if (closed_ || !stream_) return {false, "RSM writer is closed"};
  if (committed_ >= options_.fail_after_records) return {false, "injected RSM write failure"};
  stream_ << std::setprecision(15) << "{\"sequence\":" << committed_
          << ",\"segment_id\":" << segment_id << ",\"host_system_ns\":" << host_system_ns
          << ",\"host_monotonic_ns\":" << host_monotonic_ns << ",\"roll_deg\":";
  optional_number(stream_, telemetry.roll_deg);
  stream_ << ",\"pitch_deg\":"; optional_number(stream_, telemetry.pitch_deg);
  stream_ << ",\"yaw_deg\":"; optional_number(stream_, telemetry.yaw_deg);
  stream_ << ",\"mount_timer_ms\":"; optional_number(stream_, telemetry.timer_ms);
  stream_ << ",\"raw_frame\":\"" << escape(frame.raw) << "\"}\n";
  if (!stream_) return {false, "RSM telemetry append failed"};
  ++committed_;
  return committed_ % options_.flush_every_records == 0U ? flush() :
    TelemetryLogResult{true, "RSM telemetry appended"};
}

TelemetryLogResult TelemetryLogWriter::checkpoint()
{
  const auto result = ppbng_storage::atomic_write_text(options_.segments_directory,
    options_.stem + ".checkpoint", "committed_records=" + std::to_string(committed_) + "\n",
    "rsm-" + std::to_string(committed_));
  return result.ok() ? TelemetryLogResult{true, "RSM checkpoint committed"} :
    TelemetryLogResult{false, "RSM checkpoint failed: " + result.detail};
}

TelemetryLogResult TelemetryLogWriter::flush()
{
  stream_.flush();
  if (!stream_) return {false, "RSM telemetry flush failed"};
  return checkpoint();
}

TelemetryLogResult TelemetryLogWriter::close()
{
  if (closed_) return {true, "RSM writer already closed"};
  const auto result = flush();
  stream_.close();
  closed_ = true;
  return result;
}

TelemetryLogWriter::~TelemetryLogWriter() {if (!closed_) (void)close();}
}  // namespace ppbng_rsm400
