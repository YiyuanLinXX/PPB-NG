#include "ppbng_timing/host_client.hpp"

#include <algorithm>
#include <limits>
#include <string_view>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace ppbng_timing {
namespace {

#ifdef _WIN32
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
  const std::string canonical = std::string(prefix) + std::string(candidate);
  return std::wstring(canonical.begin(), canonical.end());
}

std::string win32_error(const char* operation, DWORD code) {
  return std::string(operation) + " failed with Win32 error " +
         std::to_string(static_cast<unsigned long>(code));
}

TransportCode classify_error(DWORD code) {
  return code == ERROR_DEVICE_NOT_CONNECTED || code == ERROR_INVALID_HANDLE ||
                 code == ERROR_GEN_FAILURE
             ? TransportCode::disconnected
             : TransportCode::io_error;
}

TransportResult overlapped_io(HANDLE port, bool writing,
                              const std::vector<std::uint8_t>& input,
                              std::size_t read_size, std::chrono::milliseconds timeout,
                              const HostStopToken& stop) {
  if (timeout.count() <= 0) {
    return {TransportCode::timeout, {}, "finite positive timeout required"};
  }
  if (stop.stop_requested()) {
    return {TransportCode::cancelled, {}, "stop requested"};
  }
  const std::size_t size = writing ? input.size() : read_size;
  if (size == 0U || size > static_cast<std::size_t>((std::numeric_limits<DWORD>::max)())) {
    return {TransportCode::io_error, {}, "invalid controller I/O size"};
  }
  std::vector<std::uint8_t> output(writing ? 0U : size);
  OVERLAPPED operation{};
  operation.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
  if (!operation.hEvent) {
    return {TransportCode::io_error, {}, win32_error("CreateEventW", GetLastError())};
  }
  DWORD transferred = 0U;
  const BOOL started = writing
                           ? WriteFile(port, input.data(), static_cast<DWORD>(size),
                                       &transferred, &operation)
                           : ReadFile(port, output.data(), static_cast<DWORD>(size),
                                      &transferred, &operation);
  DWORD error = started ? ERROR_SUCCESS : GetLastError();
  if (!started && error != ERROR_IO_PENDING) {
    CloseHandle(operation.hEvent);
    return {classify_error(error), {},
            win32_error(writing ? "WriteFile" : "ReadFile", error)};
  }
  if (!started) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (true) {
      if (stop.stop_requested()) {
        CancelIoEx(port, &operation);
        WaitForSingleObject(operation.hEvent, INFINITE);
        CloseHandle(operation.hEvent);
        return {TransportCode::cancelled, {}, "stop requested"};
      }
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        CancelIoEx(port, &operation);
        WaitForSingleObject(operation.hEvent, INFINITE);
        CloseHandle(operation.hEvent);
        return {TransportCode::timeout, {}, "controller I/O timed out"};
      }
      const auto remaining =
          std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
      const DWORD slice = static_cast<DWORD>(std::max<std::int64_t>(
          1, std::min<std::int64_t>(10, remaining.count())));
      const DWORD wait = WaitForSingleObject(operation.hEvent, slice);
      if (wait == WAIT_OBJECT_0) {
        break;
      }
      if (wait == WAIT_FAILED) {
        error = GetLastError();
        CancelIoEx(port, &operation);
        WaitForSingleObject(operation.hEvent, INFINITE);
        CloseHandle(operation.hEvent);
        return {classify_error(error), {}, win32_error("WaitForSingleObject", error)};
      }
    }
    if (!GetOverlappedResult(port, &operation, &transferred, FALSE)) {
      error = GetLastError();
      CloseHandle(operation.hEvent);
      return {classify_error(error), {}, win32_error("GetOverlappedResult", error)};
    }
  }
  CloseHandle(operation.hEvent);
  if (writing) {
    if (transferred != size) {
      return {TransportCode::io_error, {}, "short controller write"};
    }
    return {};
  }
  output.resize(transferred);
  return {TransportCode::ok, std::move(output), {}};
}
#endif

}  // namespace

struct WindowsControllerSerialTransport::Impl {
#ifdef _WIN32
  HANDLE port{INVALID_HANDLE_VALUE};
#endif
};

WindowsControllerSerialTransport::WindowsControllerSerialTransport()
    : impl_(std::make_unique<Impl>()) {}

WindowsControllerSerialTransport::~WindowsControllerSerialTransport() { close(); }

TransportResult WindowsControllerSerialTransport::open(
    const ControllerPortIdentity& identity, std::chrono::milliseconds timeout,
    const HostStopToken& stop) {
  if (timeout.count() <= 0) {
    return {TransportCode::timeout, {}, "finite positive timeout required"};
  }
  if (stop.stop_requested()) {
    return {TransportCode::cancelled, {}, "stop requested"};
  }
  if (identity.baud_rate == 0U) {
    return {TransportCode::io_error, {}, "controller baud rate is required"};
  }
#ifdef _WIN32
  const auto path = canonical_com_path(identity.com_path);
  if (path.empty()) {
    return {TransportCode::io_error, {}, "COM path must be COMn or \\\\.\\COMn"};
  }
  close();
  impl_->port = CreateFileW(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                            OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
                            nullptr);
  if (impl_->port == INVALID_HANDLE_VALUE) {
    const auto error = GetLastError();
    return {classify_error(error), {}, win32_error("CreateFileW", error)};
  }
  DCB dcb{};
  dcb.DCBlength = sizeof(dcb);
  if (!GetCommState(impl_->port, &dcb)) {
    const auto error = GetLastError();
    close();
    return {TransportCode::io_error, {}, win32_error("GetCommState", error)};
  }
  dcb.BaudRate = identity.baud_rate;
  dcb.ByteSize = 8U;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fBinary = TRUE;
  dcb.fParity = FALSE;
  dcb.fOutxCtsFlow = FALSE;
  dcb.fOutxDsrFlow = FALSE;
  dcb.fDtrControl = DTR_CONTROL_DISABLE;
  dcb.fDsrSensitivity = FALSE;
  dcb.fOutX = FALSE;
  dcb.fInX = FALSE;
  dcb.fRtsControl = RTS_CONTROL_DISABLE;
  dcb.fAbortOnError = FALSE;
  if (!SetCommState(impl_->port, &dcb)) {
    const auto error = GetLastError();
    close();
    return {TransportCode::io_error, {}, win32_error("SetCommState", error)};
  }
  return {};
#else
  (void)identity;
  return {TransportCode::io_error, {}, "Windows serial transport unavailable"};
#endif
}

TransportResult WindowsControllerSerialTransport::write(
    const std::vector<std::uint8_t>& bytes, std::chrono::milliseconds timeout,
    const HostStopToken& stop) {
#ifdef _WIN32
  if (!is_open()) return {TransportCode::disconnected, {}, "controller port is closed"};
  return overlapped_io(impl_->port, true, bytes, 0U, timeout, stop);
#else
  (void)bytes; (void)timeout; (void)stop;
  return {TransportCode::io_error, {}, "Windows serial transport unavailable"};
#endif
}

TransportResult WindowsControllerSerialTransport::read(
    std::size_t maximum_bytes, std::chrono::milliseconds timeout,
    const HostStopToken& stop) {
#ifdef _WIN32
  if (!is_open()) return {TransportCode::disconnected, {}, "controller port is closed"};
  return overlapped_io(impl_->port, false, {}, maximum_bytes, timeout, stop);
#else
  (void)maximum_bytes; (void)timeout; (void)stop;
  return {TransportCode::io_error, {}, "Windows serial transport unavailable"};
#endif
}

void WindowsControllerSerialTransport::close() noexcept {
#ifdef _WIN32
  if (impl_->port != INVALID_HANDLE_VALUE) {
    CancelIoEx(impl_->port, nullptr);
    CloseHandle(impl_->port);
    impl_->port = INVALID_HANDLE_VALUE;
  }
#endif
}

bool WindowsControllerSerialTransport::is_open() const noexcept {
#ifdef _WIN32
  return impl_->port != INVALID_HANDLE_VALUE;
#else
  return false;
#endif
}

}  // namespace ppbng_timing
