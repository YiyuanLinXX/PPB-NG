#include "ppbng_gnss/um982_receiver.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string_view>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace ppbng_gnss {
namespace {

ByteReadResult invalid_configuration(const std::string& detail) {
  return {ByteReadCode::io_error, {}, detail};
}

bool valid_configuration(const SerialReceiveConfiguration& configuration) {
  return !configuration.com_path.empty() && configuration.baud_rate != 0U &&
         configuration.read_chunk_bytes != 0U && configuration.max_line_bytes != 0U;
}

ReceivedSentence parse_line(std::string line,
                            std::chrono::system_clock::time_point timestamp) {
  ReceivedSentence result;
  result.raw_line = std::move(line);
  result.host_receive_time = timestamp;
  try {
    if (result.raw_line.rfind("$GNGGA,", 0U) == 0U ||
        result.raw_line.rfind("$GPGGA,", 0U) == 0U) {
      result.gga = parse_gga(result.raw_line);
      result.kind = SentenceKind::gga;
    } else if (result.raw_line.rfind("#UNIHEADINGA,", 0U) == 0U) {
      result.heading = parse_uniheadinga(result.raw_line);
      result.kind = SentenceKind::uniheadinga;
    } else {
      result.kind = SentenceKind::other;
    }
  } catch (const std::exception& error) {
    result.kind = SentenceKind::parse_error;
    result.parse_error = error.what();
  }
  return result;
}

#ifdef _WIN32
std::wstring widen_ascii(const std::string& value) {
  return std::wstring(value.begin(), value.end());
}

std::string windows_error(const char* operation, DWORD error) {
  return std::string(operation) + " failed with Win32 error " +
         std::to_string(static_cast<unsigned long>(error));
}

std::wstring canonical_com_path(const std::string& path) {
  constexpr std::string_view prefix{"\\\\.\\"};
  const std::string_view candidate = path.rfind(prefix, 0U) == 0U
                                         ? std::string_view(path).substr(prefix.size())
                                         : std::string_view(path);
  if (candidate.size() <= 3U || candidate.substr(0U, 3U) != "COM" ||
      !std::all_of(candidate.begin() + 3, candidate.end(),
                   [](char value) { return value >= '0' && value <= '9'; })) {
    return {};
  }
  return widen_ascii(std::string(prefix) + std::string(candidate));
}
#endif

}  // namespace

bool ReceiveStopToken::stop_requested() const noexcept {
  return flag_ && flag_->load();
}

ReceiveStopSource::ReceiveStopSource()
    : flag_(std::make_shared<std::atomic_bool>(false)) {}

void ReceiveStopSource::request_stop() noexcept { flag_->store(true); }

struct WindowsReceiveOnlySerial::Impl {
#ifdef _WIN32
  HANDLE port{INVALID_HANDLE_VALUE};
#endif
};

WindowsReceiveOnlySerial::WindowsReceiveOnlySerial()
    : impl_(std::make_unique<Impl>()) {}

WindowsReceiveOnlySerial::~WindowsReceiveOnlySerial() { close(); }

ByteReadResult WindowsReceiveOnlySerial::open(
    const SerialReceiveConfiguration& configuration,
    std::chrono::milliseconds timeout, const ReceiveStopToken& stop) {
  if (timeout.count() <= 0) {
    return {ByteReadCode::timeout, {}, "finite positive timeout required"};
  }
  if (stop.stop_requested()) {
    return {ByteReadCode::cancelled, {}, "stop requested"};
  }
  if (!valid_configuration(configuration)) {
    return invalid_configuration("explicit COM path, baud rate and buffer limits are required");
  }
#ifdef _WIN32
  const auto path = canonical_com_path(configuration.com_path);
  if (path.empty()) {
    return invalid_configuration("COM path must be COMn or \\\\.\\COMn");
  }
  close();
  // Deliberately GENERIC_READ only. No write handle is ever requested.
  impl_->port = CreateFileW(path.c_str(), GENERIC_READ, 0, nullptr, OPEN_EXISTING,
                            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, nullptr);
  if (impl_->port == INVALID_HANDLE_VALUE) {
    return {ByteReadCode::disconnected, {},
            windows_error("CreateFileW", GetLastError())};
  }
  DCB dcb{};
  dcb.DCBlength = sizeof(dcb);
  if (!GetCommState(impl_->port, &dcb)) {
    const auto error = GetLastError();
    close();
    return {ByteReadCode::io_error, {}, windows_error("GetCommState", error)};
  }
  dcb.BaudRate = configuration.baud_rate;
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fBinary = TRUE;
  dcb.fOutxCtsFlow = FALSE;
  dcb.fOutxDsrFlow = FALSE;
  dcb.fDtrControl = DTR_CONTROL_DISABLE;
  dcb.fDsrSensitivity = FALSE;
  dcb.fOutX = FALSE;
  dcb.fInX = FALSE;
  dcb.fRtsControl = RTS_CONTROL_DISABLE;
  if (!SetCommState(impl_->port, &dcb)) {
    const auto error = GetLastError();
    close();
    return {ByteReadCode::io_error, {}, windows_error("SetCommState", error)};
  }
  return {ByteReadCode::data, {}, {}};
#else
  (void)configuration;
  return invalid_configuration("Windows receive-only serial backend unavailable");
#endif
}

ByteReadResult WindowsReceiveOnlySerial::read(
    std::size_t maximum_bytes, std::chrono::milliseconds timeout,
    const ReceiveStopToken& stop) {
  if (timeout.count() <= 0) {
    return {ByteReadCode::timeout, {}, "finite positive timeout required"};
  }
  if (stop.stop_requested()) {
    return {ByteReadCode::cancelled, {}, "stop requested"};
  }
  if (maximum_bytes == 0U ||
      maximum_bytes > static_cast<std::size_t>((std::numeric_limits<DWORD>::max)())) {
    return invalid_configuration("invalid serial read size");
  }
#ifdef _WIN32
  if (!is_open()) {
    return {ByteReadCode::disconnected, {}, "COM port is not open"};
  }
  ByteReadResult result;
  result.bytes.resize(maximum_bytes);
  OVERLAPPED overlapped{};
  overlapped.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
  if (overlapped.hEvent == nullptr) {
    return {ByteReadCode::io_error, {}, windows_error("CreateEventW", GetLastError())};
  }
  DWORD received = 0U;
  const BOOL started = ReadFile(impl_->port, result.bytes.data(),
                                static_cast<DWORD>(maximum_bytes), &received, &overlapped);
  DWORD error = started ? ERROR_SUCCESS : GetLastError();
  if (!started && error != ERROR_IO_PENDING) {
    CloseHandle(overlapped.hEvent);
    return {error == ERROR_DEVICE_NOT_CONNECTED ? ByteReadCode::disconnected
                                                 : ByteReadCode::io_error,
            {}, windows_error("ReadFile", error)};
  }
  if (!started) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (true) {
      if (stop.stop_requested()) {
        CancelIoEx(impl_->port, &overlapped);
        WaitForSingleObject(overlapped.hEvent, INFINITE);
        CloseHandle(overlapped.hEvent);
        return {ByteReadCode::cancelled, {}, "stop requested"};
      }
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        CancelIoEx(impl_->port, &overlapped);
        WaitForSingleObject(overlapped.hEvent, INFINITE);
        CloseHandle(overlapped.hEvent);
        return {ByteReadCode::timeout, {}, "serial read timed out"};
      }
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
      const DWORD slice = static_cast<DWORD>(std::max<std::int64_t>(
          1, std::min<std::int64_t>(10, remaining.count())));
      if (WaitForSingleObject(overlapped.hEvent, slice) == WAIT_OBJECT_0) {
        break;
      }
    }
    if (!GetOverlappedResult(impl_->port, &overlapped, &received, FALSE)) {
      error = GetLastError();
      CloseHandle(overlapped.hEvent);
      return {error == ERROR_DEVICE_NOT_CONNECTED || error == ERROR_OPERATION_ABORTED
                  ? ByteReadCode::disconnected
                  : ByteReadCode::io_error,
              {}, windows_error("GetOverlappedResult", error)};
    }
  }
  CloseHandle(overlapped.hEvent);
  result.bytes.resize(received);
  result.code = received == 0U ? ByteReadCode::timeout : ByteReadCode::data;
  return result;
#else
  (void)maximum_bytes;
  return invalid_configuration("Windows receive-only serial backend unavailable");
#endif
}

void WindowsReceiveOnlySerial::close() noexcept {
#ifdef _WIN32
  if (impl_->port != INVALID_HANDLE_VALUE) {
    CancelIoEx(impl_->port, nullptr);
    CloseHandle(impl_->port);
    impl_->port = INVALID_HANDLE_VALUE;
  }
#endif
}

bool WindowsReceiveOnlySerial::is_open() const noexcept {
#ifdef _WIN32
  return impl_->port != INVALID_HANDLE_VALUE;
#else
  return false;
#endif
}

Um982Receiver::Um982Receiver(std::unique_ptr<IReceiveOnlyByteSource> source)
    : source_(std::move(source)) {
  if (!source_) {
    throw std::invalid_argument("receive-only byte source is required");
  }
}

Um982Receiver::~Um982Receiver() { stop(); }

ByteReadResult Um982Receiver::connect(
    const SerialReceiveConfiguration& configuration,
    std::chrono::milliseconds timeout, const ReceiveStopToken& stop_token) {
  if (!valid_configuration(configuration)) {
    return invalid_configuration("explicit COM path, baud rate and buffer limits are required");
  }
  source_->close();
  auto result = source_->open(configuration, timeout, stop_token);
  if (result.code == ByteReadCode::data && source_->is_open()) {
    configuration_ = configuration;
    decoder_ = std::make_unique<IncrementalLineDecoder>(configuration.max_line_bytes);
    // Epoch zero means "no live receiver connection" to downstream time and
    // frame-context consumers.  Number the initial successful connection as
    // epoch one, then increment again after every successful recovery so
    // observations can never be associated across a serial reconnect.
    ++connection_epoch_;
    state_ = ReceiverState::connected;
  } else {
    state_ = result.code == ByteReadCode::disconnected ? ReceiverState::disconnected
                                                       : ReceiverState::faulted;
  }
  return result;
}

ReceiverPollResult Um982Receiver::poll(std::chrono::milliseconds timeout,
                                       const ReceiveStopToken& stop_token) {
  if (state_ != ReceiverState::connected || !configuration_ || !decoder_) {
    return {ByteReadCode::disconnected, {}, "receiver is not connected"};
  }
  auto bytes = source_->read(configuration_->read_chunk_bytes, timeout, stop_token);
  ReceiverPollResult result{bytes.code, {}, bytes.detail};
  if (bytes.code != ByteReadCode::data) {
    if (bytes.code == ByteReadCode::disconnected || bytes.code == ByteReadCode::io_error) {
      source_->close();
      decoder_->reset();
      state_ = bytes.code == ByteReadCode::disconnected ? ReceiverState::disconnected
                                                        : ReceiverState::faulted;
    }
    return result;
  }
  const auto timestamp = std::chrono::system_clock::now();
  try {
    const std::string_view chunk(reinterpret_cast<const char*>(bytes.bytes.data()),
                                 bytes.bytes.size());
    for (auto& line : decoder_->feed(chunk)) {
      result.sentences.push_back(parse_line(std::move(line), timestamp));
    }
  } catch (const std::exception& error) {
    result.code = ByteReadCode::io_error;
    result.detail = error.what();
    state_ = ReceiverState::faulted;
  }
  return result;
}

ByteReadResult Um982Receiver::recover(std::chrono::milliseconds timeout,
                                      const ReceiveStopToken& stop_token) {
  if (!configuration_) {
    return invalid_configuration("no previous serial configuration to recover");
  }
  const auto saved = *configuration_;
  source_->close();
  decoder_->reset();
  auto result = source_->open(saved, timeout, stop_token);
  if (result.code == ByteReadCode::data && source_->is_open()) {
    ++connection_epoch_;
    state_ = ReceiverState::connected;
  } else {
    state_ = result.code == ByteReadCode::disconnected ? ReceiverState::disconnected
                                                       : ReceiverState::faulted;
  }
  return result;
}

void Um982Receiver::stop() noexcept {
  source_->close();
  if (decoder_) {
    decoder_->reset();
  }
  state_ = ReceiverState::stopped;
}

}  // namespace ppbng_gnss
