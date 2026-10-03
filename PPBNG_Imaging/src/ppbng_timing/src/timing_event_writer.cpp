#include "ppbng_timing/timing_event_writer.hpp"

#include <array>
#include <cstring>
#include <system_error>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace ppbng_timing {
namespace {

constexpr std::int64_t kUnknownUtc = (std::numeric_limits<std::int64_t>::min)();

void u32(std::vector<std::uint8_t>& out, std::uint32_t value) {
  for (int i = 0; i < 4; ++i) out.push_back(static_cast<std::uint8_t>(value >> (i * 8)));
}
void u64(std::vector<std::uint8_t>& out, std::uint64_t value) {
  for (int i = 0; i < 8; ++i) out.push_back(static_cast<std::uint8_t>(value >> (i * 8)));
}

bool safe_target(const std::filesystem::path& root_input,
                 const std::filesystem::path& relative,
                 std::filesystem::path& target, std::string& detail) {
  std::error_code error;
  if (relative.empty() || relative.is_absolute()) {
    detail = "timing log path must be nonempty and relative"; return false;
  }
  const auto normalized = relative.lexically_normal();
  if (normalized.empty() || *normalized.begin() == "..") {
    detail = "timing log path escapes session root"; return false;
  }
  const auto root = std::filesystem::weakly_canonical(
      std::filesystem::absolute(root_input, error), error);
  if (error || !std::filesystem::is_directory(root, error)) {
    detail = "session root must be an existing directory"; return false;
  }
  target = root / normalized;
  const auto parent = std::filesystem::weakly_canonical(target.parent_path(), error);
  if (error || !std::filesystem::is_directory(parent, error)) {
    detail = "timing log parent must already exist"; return false;
  }
  const auto parent_relative = parent.lexically_relative(root);
  if (parent_relative.empty() || parent_relative.is_absolute() ||
      *parent_relative.begin() == "..") {
    detail = "timing log parent is outside session root"; return false;
  }
  target = parent / target.filename();
  return true;
}

}  // namespace

struct TimingEventWriter::Impl {
#ifdef _WIN32
  HANDLE file{INVALID_HANDLE_VALUE};
#else
  int file{-1};
#endif
};

TimingEventWriter::TimingEventWriter(std::filesystem::path path,
                                     TimingEventWriterOptions options)
    : path_(std::move(path)), options_(options), impl_(std::make_unique<Impl>()) {}

TimingEventWriter::~TimingEventWriter() {
#ifdef _WIN32
  if (impl_->file != INVALID_HANDLE_VALUE) CloseHandle(impl_->file);
#else
  if (impl_->file >= 0) ::close(impl_->file);
#endif
}

std::pair<TimingLogStatus, std::unique_ptr<TimingEventWriter>> TimingEventWriter::create(
    const std::filesystem::path& session_root, const std::filesystem::path& relative_path,
    TimingEventWriterOptions options) {
  std::filesystem::path target;
  std::string detail;
  if (!safe_target(session_root, relative_path, target, detail))
    return std::make_pair(TimingLogStatus{TimingLogCode::unsafe_path, detail}, nullptr);
  auto writer = std::unique_ptr<TimingEventWriter>(new TimingEventWriter(target, options));
  auto status = writer->open_exclusive();
  if (!status.ok()) return std::make_pair(status, nullptr);
  const std::array<std::uint8_t, 8> header{'P', 'T', 'L', 'G', 1U, 0U, 0U, 0U};
  status = writer->write_all(header.data(), header.size());
  if (!status.ok()) return std::make_pair(status, nullptr);
  return std::make_pair(TimingLogStatus{}, std::move(writer));
}

TimingLogStatus TimingEventWriter::open_exclusive() {
#ifdef _WIN32
  impl_->file = CreateFileW(path_.c_str(), GENERIC_WRITE, FILE_SHARE_READ, nullptr,
                            CREATE_NEW, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (impl_->file == INVALID_HANDLE_VALUE) {
    const auto error = GetLastError();
    return {error == ERROR_FILE_EXISTS || error == ERROR_ALREADY_EXISTS
                ? TimingLogCode::already_exists : TimingLogCode::open_failed,
            "exclusive timing log create failed with Win32 error " + std::to_string(error)};
  }
#else
  impl_->file = ::open(path_.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0644);
  if (impl_->file < 0) return {TimingLogCode::open_failed, "exclusive timing log create failed"};
#endif
  return {};
}

TimingLogStatus TimingEventWriter::write_all(const std::uint8_t* data, std::size_t size) {
  if (physical_bytes_ > options_.fail_after_total_bytes ||
      size > options_.fail_after_total_bytes - physical_bytes_)
    return {TimingLogCode::write_failed, "injected timing log write failure"};
#ifdef _WIN32
  DWORD written = 0U;
  if (size > (std::numeric_limits<DWORD>::max)() ||
      !WriteFile(impl_->file, data, static_cast<DWORD>(size), &written, nullptr) ||
      written != size)
    return {TimingLogCode::write_failed, "timing log WriteFile failed or was short"};
#else
  const auto written = ::write(impl_->file, data, size);
  if (written < 0 || static_cast<std::size_t>(written) != size)
    return {TimingLogCode::write_failed, "timing log write failed or was short"};
#endif
  physical_bytes_ += size;
  return {};
}

TimingLogStatus TimingEventWriter::append_record(
    std::uint8_t type, const std::vector<std::uint8_t>& payload) {
  std::vector<std::uint8_t> record;
  record.reserve(1U + 4U + payload.size() + 4U);
  record.push_back(type); u32(record, static_cast<std::uint32_t>(payload.size()));
  record.insert(record.end(), payload.begin(), payload.end());
  u32(record, crc32_ieee(record.data(), record.size()));
  auto status = write_all(record.data(), record.size());
  if (status.ok()) ++records_written_;
  return status;
}

TimingLogStatus TimingEventWriter::append_pps(
    const PpsAnchor& anchor, std::uint64_t host_receive_monotonic_ns) {
  const bool utc_valid = anchor.utc_second != kUnknownUtc;
  if (anchor.boot_id == 0U || host_receive_monotonic_ns == 0U)
    return {TimingLogCode::invalid_record, "invalid PPS anchor identity/time"};
  std::vector<std::uint8_t> payload;
  payload.reserve(38U);
  u32(payload, anchor.boot_id); u64(payload, anchor.pps_sequence);
  u64(payload, anchor.captured_tick); u64(payload, host_receive_monotonic_ns);
  payload.push_back(static_cast<std::uint8_t>(anchor.lock));
  payload.push_back(utc_valid ? 1U : 0U);
  u64(payload, static_cast<std::uint64_t>(anchor.utc_second));
  return append_record(1U, payload);
}

TimingLogStatus TimingEventWriter::append_trigger(
    const TriggerEvent& trigger, std::uint64_t host_receive_monotonic_ns) {
  if (trigger.boot_id == 0U || trigger.ticks_per_second == 0U ||
      host_receive_monotonic_ns == 0U)
    return {TimingLogCode::invalid_record, "invalid trigger event identity/time"};
  std::vector<std::uint8_t> payload;
  payload.reserve(50U);
  u32(payload, trigger.boot_id); u64(payload, trigger.event_sequence);
  payload.push_back(static_cast<std::uint8_t>(trigger.channel));
  u64(payload, trigger.channel_sequence); u64(payload, trigger.pps_sequence);
  u64(payload, trigger.offset_ticks); u32(payload, trigger.ticks_per_second);
  payload.push_back(static_cast<std::uint8_t>(trigger.lock));
  u64(payload, host_receive_monotonic_ns);
  return append_record(2U, payload);
}

TimingLogStatus TimingEventWriter::flush() {
  if (options_.fail_flush)
    return {TimingLogCode::flush_failed, "injected timing log flush failure"};
#ifdef _WIN32
  if (!FlushFileBuffers(impl_->file))
    return {TimingLogCode::flush_failed, "timing log FlushFileBuffers failed"};
#else
  if (::fsync(impl_->file) != 0) return {TimingLogCode::flush_failed, "timing log fsync failed"};
#endif
  return {};
}

TimingSessionBindResult TimingSessionBinding::prepare(
    const std::string& request_id, const std::string& session_id,
    const std::string& session_directory, bool task_active) {
  if (const auto old = history_.find(request_id); old != history_.end()) {
    const bool same = old->second.session_id == session_id &&
                      old->second.directory == session_directory;
    return same ? TimingSessionBindResult{old->second.result.accepted, true,
                                           old->second.result.detail}
                : TimingSessionBindResult{false, false, "request_id payload mismatch"};
  }
  TimingSessionBindResult result;
  if (request_id.empty() || session_id.empty() || session_directory.empty())
    result = {false, false, "request_id, session_id and session_directory are required"};
  else if (task_active)
    result = {false, false, "cannot prepare while timing task is active"};
  else {
    std::error_code error;
    const auto allowed = std::filesystem::weakly_canonical(
        std::filesystem::absolute(allowed_output_root_, error), error);
    if (error || !std::filesystem::is_directory(allowed, error))
      result = {false, false, "allowed_output_root is missing or invalid"};
    else {
      const auto session = std::filesystem::weakly_canonical(
          std::filesystem::absolute(session_directory, error), error);
      const auto relative = session.lexically_relative(allowed);
      if (error || !std::filesystem::is_directory(session, error) ||
          !std::filesystem::is_directory(session / "segments", error) ||
          relative.empty() || relative.is_absolute() || *relative.begin() == "..")
        result = {false, false,
                  "session must be an existing strict descendant with segments directory"};
      else if (bound_ && (session_id_ != session_id || directory_ != session))
        result = {false, false, "timing node is already bound to a different dataset session"};
      else {
        const bool was_bound = bound_;
        bound_ = true; session_id_ = session_id; directory_ = session;
        result = {true, was_bound, "existing dataset session bound; no file opened"};
      }
    }
  }
  history_.emplace(request_id, Record{session_id, session_directory, result});
  return result;
}

}  // namespace ppbng_timing
