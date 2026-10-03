#include "ppbng_gnss/gnss_log_writer.hpp"

#include "ppbng_storage/dataset_session.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>
#include <system_error>

#ifdef _WIN32
#include <windows.h>
#endif

namespace ppbng_gnss
{
namespace
{
bool safe_stem(const std::string & value)
{
  return !value.empty() && value != "." && value != ".." &&
    value.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") ==
    std::string::npos;
}

bool create_exclusive(const std::filesystem::path & path, std::string & error)
{
#ifdef _WIN32
  HANDLE handle = CreateFileW(path.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_NEW,
    FILE_ATTRIBUTE_NORMAL, nullptr);
  if (handle == INVALID_HANDLE_VALUE) {
    error = "exclusive create failed (Win32 error " + std::to_string(GetLastError()) + ")";
    return false;
  }
  CloseHandle(handle);
  return true;
#else
  if (std::filesystem::exists(path)) {error = "target already exists"; return false;}
  std::ofstream stream(path, std::ios::binary | std::ios::out);
  if (!stream) {error = "exclusive create failed"; return false;}
  return true;
#endif
}

std::string json_escape(const std::string & input)
{
  std::ostringstream output;
  for (const unsigned char ch : input) {
    switch (ch) {
      case '\\': output << "\\\\"; break;
      case '"': output << "\\\""; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        // The receive-only serial stream may be opened in the middle of a
        // sentence, so its first partial line can contain arbitrary bytes.
        // Escape high bytes as well as JSON control characters: this keeps
        // every JSONL record valid UTF-8 while preserving the original byte
        // value losslessly as a U+00XX code point.
        if (ch < 0x20U || ch >= 0x80U) {
          output << "\\u00" << std::hex << std::setw(2) << std::setfill('0') <<
            static_cast<unsigned int>(ch) << std::dec;
        } else {output << static_cast<char>(ch);}
    }
  }
  return output.str();
}

const char * kind_text(SentenceKind kind)
{
  switch (kind) {
    case SentenceKind::gga: return "gga_valid";
    case SentenceKind::uniheadinga: return "uniheadinga_valid";
    case SentenceKind::other: return "other";
    case SentenceKind::parse_error: return "parse_error";
  }
  return "unknown";
}

std::int64_t system_ns(const std::chrono::system_clock::time_point time)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
}
}  // namespace

GnssLogWriter::GnssLogWriter(GnssLogWriterOptions options) : options_(std::move(options)) {}
GnssLogWriter::~GnssLogWriter() {close();}

std::pair<GnssLogResult, std::unique_ptr<GnssLogWriter>> GnssLogWriter::create(
  const GnssLogWriterOptions & options)
{
  std::error_code ec;
  if (options.session_directory.empty() || !std::filesystem::is_directory(options.session_directory, ec)) {
    return std::make_pair(GnssLogResult{false,
      "session_directory must explicitly name an existing directory"},
      std::unique_ptr<GnssLogWriter>{});
  }
  if (!safe_stem(options.stream_stem) || options.flush_every_sentences == 0U) {
    return std::make_pair(GnssLogResult{false, "invalid GNSS writer configuration"},
      std::unique_ptr<GnssLogWriter>{});
  }
  auto writer = std::unique_ptr<GnssLogWriter>(new GnssLogWriter(options));
  const auto result = writer->open_files();
  if (!result.success) {return std::make_pair(result, std::unique_ptr<GnssLogWriter>{});}
  return std::make_pair(GnssLogResult{true, "append-only GNSS logs exclusively created"},
    std::move(writer));
}

GnssLogResult GnssLogWriter::open_files()
{
  const auto raw_path = options_.session_directory / (options_.stream_stem + ".sentences.jsonl");
  const auto gga_path = options_.session_directory / (options_.stream_stem + ".gga.jsonl");
  const auto heading_path = options_.session_directory / (options_.stream_stem + ".uniheadinga.jsonl");
  const auto checkpoint_path = options_.session_directory / (options_.stream_stem + ".checkpoint");
  std::string error;
  for (const auto & path : {raw_path, gga_path, heading_path, checkpoint_path}) {
    if (!create_exclusive(path, error)) {return {false, path.filename().string() + ": " + error};}
  }
  raw_.open(raw_path, std::ios::app);
  gga_.open(gga_path, std::ios::app);
  heading_.open(heading_path, std::ios::app);
  return raw_ && gga_ && heading_ ? GnssLogResult{true, "logs opened"} :
    GnssLogResult{false, "failed to open exclusively reserved GNSS logs"};
}

GnssLogResult GnssLogWriter::append(
  const ReceivedSentence & sentence, const std::uint64_t host_monotonic_ns,
  const std::uint64_t connection_epoch)
{
  if (closed_) {return {false, "GNSS writer is closed"};}
  if (committed_sentences_ >= options_.fail_after_sentences) {
    return {false, "injected GNSS log write failure"};
  }
  const auto host_system_ns = system_ns(sentence.host_receive_time);
  raw_ << "{\"sequence\":" << committed_sentences_ << ",\"host_system_ns\":" << host_system_ns
       << ",\"host_monotonic_ns\":" << host_monotonic_ns << ",\"connection_epoch\":"
       << connection_epoch << ",\"parse_status\":\"" << kind_text(sentence.kind)
       << "\",\"parse_error\":\"" << json_escape(sentence.parse_error) << "\",\"raw\":\""
       << json_escape(sentence.raw_line) << "\"}\n";
  if (sentence.gga) {
    const auto & fix = *sentence.gga;
    gga_ << std::setprecision(15) << "{\"sequence\":" << committed_sentences_
         << ",\"host_system_ns\":" << host_system_ns << ",\"host_monotonic_ns\":"
         << host_monotonic_ns << ",\"connection_epoch\":" << connection_epoch
         << ",\"utc_hhmmss\":\"" << json_escape(fix.utc_hhmmss) << "\",\"latitude_deg\":"
         << fix.latitude_deg << ",\"longitude_deg\":" << fix.longitude_deg
         << ",\"altitude_m\":" << fix.altitude_m << ",\"quality\":" << fix.quality
         << ",\"satellites_used\":" << fix.satellites_used << ",\"hdop\":" << fix.hdop
         << ",\"has_differential_age\":" << (fix.has_differential_age ? "true" : "false")
         << ",\"differential_age_sec\":" << fix.differential_age_sec << "}\n";
  }
  if (sentence.heading) {
    const auto & heading = *sentence.heading;
    heading_ << std::setprecision(15) << "{\"sequence\":" << committed_sentences_
             << ",\"host_system_ns\":" << host_system_ns << ",\"host_monotonic_ns\":"
             << host_monotonic_ns << ",\"connection_epoch\":" << connection_epoch
             << ",\"time_reference\":\"" << json_escape(heading.receiver_time.time_reference)
             << "\",\"time_status\":\"" << json_escape(heading.receiver_time.time_status)
             << "\",\"week\":" << heading.receiver_time.week
             << ",\"milliseconds_of_week\":" << heading.receiver_time.milliseconds_of_week
             << ",\"leap_seconds\":" << static_cast<unsigned>(heading.receiver_time.leap_seconds)
             << ",\"output_delay_ms\":" << heading.receiver_time.output_delay_ms
             << ",\"utc_valid\":" << (heading.receiver_time.utc_unix_nanoseconds ? "true" : "false")
             << ",\"utc_unix_ns\":"
             << heading.receiver_time.utc_unix_nanoseconds.value_or(0)
             << ",\"solution_status\":\"" << json_escape(heading.solution_status)
             << "\",\"position_type\":\"" << json_escape(heading.position_type)
             << "\",\"baseline_m\":" << heading.baseline_m << ",\"heading_deg\":"
             << heading.heading_deg << ",\"pitch_deg\":" << heading.pitch_deg
             << ",\"heading_stddev_deg\":" << heading.heading_stddev_deg
             << ",\"pitch_stddev_deg\":" << heading.pitch_stddev_deg
             << ",\"satellites_tracked\":" << heading.satellites_tracked
             << ",\"satellites_used\":" << heading.satellites_used << "}\n";
  }
  if (!raw_ || !gga_ || !heading_) {return {false, "GNSS append failed"};}
  ++committed_sentences_;
  if (committed_sentences_ % options_.flush_every_sentences == 0U) {return flush();}
  return {true, "sentence persisted"};
}

GnssLogResult GnssLogWriter::checkpoint()
{
  std::ostringstream state;
  state << "committed_sentences=" << committed_sentences_ << '\n';
  const auto result = ppbng_storage::atomic_write_text(options_.session_directory,
    options_.stream_stem + ".checkpoint", state.str(),
    "gnss-" + std::to_string(committed_sentences_));
  return result.ok() ? GnssLogResult{true, "checkpoint committed"} :
    GnssLogResult{false, "GNSS checkpoint failed: " + result.detail};
}

GnssLogResult GnssLogWriter::flush()
{
  raw_.flush(); gga_.flush(); heading_.flush();
  if (!raw_ || !gga_ || !heading_) {return {false, "GNSS flush failed"};}
  return checkpoint();
}

GnssLogResult GnssLogWriter::close()
{
  if (closed_) {return {true, "GNSS writer already closed"};}
  const auto result = flush();
  raw_.close(); gga_.close(); heading_.close();
  closed_ = true;
  return result;
}

}  // namespace ppbng_gnss
