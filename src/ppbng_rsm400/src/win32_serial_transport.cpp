#include "ppbng_rsm400/win32_serial_transport.hpp"

#include <algorithm>
#include <cctype>
#include <limits>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace ppbng_rsm400 {

class Win32SerialTransport::Impl {
 public:
#ifdef _WIN32
  HANDLE handle{INVALID_HANDLE_VALUE};
#endif
};

namespace {
bool valid_com_name(const std::string& port) {
  if (port.size() < 4U || port.size() > 8U) { return false; }
  if (std::toupper(static_cast<unsigned char>(port[0])) != 'C' ||
      std::toupper(static_cast<unsigned char>(port[1])) != 'O' ||
      std::toupper(static_cast<unsigned char>(port[2])) != 'M') { return false; }
  return std::all_of(port.begin() + 3, port.end(),
                     [](char c) { return c >= '0' && c <= '9'; });
}

#ifdef _WIN32
std::string windows_error(const char* operation) {
  return std::string(operation) + " failed with Win32 error " + std::to_string(GetLastError());
}

bool apply_timeouts(HANDLE handle, std::chrono::milliseconds timeout, bool reading) {
  if (timeout.count() <= 0) { return false; }
  COMMTIMEOUTS settings{};
  const auto bounded = static_cast<DWORD>(std::min<std::int64_t>(
      timeout.count(), static_cast<std::int64_t>((std::numeric_limits<DWORD>::max)())));
  if (reading) {
    settings.ReadIntervalTimeout = MAXDWORD;
    settings.ReadTotalTimeoutConstant = std::max<DWORD>(1U, bounded);
  } else {
    settings.WriteTotalTimeoutConstant = std::max<DWORD>(1U, bounded);
  }
  return SetCommTimeouts(handle, &settings) != FALSE;
}
#endif
}  // namespace

Win32SerialTransport::Win32SerialTransport() : impl_(std::make_unique<Impl>()) {}
Win32SerialTransport::~Win32SerialTransport() { close(); }

SerialOpenResult Win32SerialTransport::open_port(const std::string& com_port) {
  if (!valid_com_name(com_port)) {
    return {IoCode::error, "port must be an explicit COM<number> name"};
  }
  close();
#ifdef _WIN32
  std::wstring path = L"\\\\.\\";
  path.append(com_port.begin(), com_port.end());
  impl_->handle = CreateFileW(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                              OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (impl_->handle == INVALID_HANDLE_VALUE) { return {IoCode::error, windows_error("CreateFileW")}; }
  DCB dcb{};
  dcb.DCBlength = sizeof(dcb);
  if (!GetCommState(impl_->handle, &dcb)) {
    const auto detail = windows_error("GetCommState"); close(); return {IoCode::error, detail};
  }
  // ICD 4.3: fixed 115200 baud, 8 data bits, no parity, one stop bit, no flow control.
  dcb.BaudRate = CBR_115200; dcb.ByteSize = 8; dcb.Parity = NOPARITY; dcb.StopBits = ONESTOPBIT;
  dcb.fBinary = TRUE; dcb.fParity = FALSE; dcb.fOutxCtsFlow = FALSE; dcb.fOutxDsrFlow = FALSE;
  dcb.fDtrControl = DTR_CONTROL_DISABLE; dcb.fDsrSensitivity = FALSE;
  dcb.fOutX = FALSE; dcb.fInX = FALSE; dcb.fRtsControl = RTS_CONTROL_DISABLE;
  if (!SetCommState(impl_->handle, &dcb)) {
    const auto detail = windows_error("SetCommState"); close(); return {IoCode::error, detail};
  }
  return {};
#else
  (void)com_port;
  return {IoCode::error, "Win32 serial transport is unavailable on this platform"};
#endif
}

void Win32SerialTransport::close() noexcept {
#ifdef _WIN32
  if (impl_->handle != INVALID_HANDLE_VALUE) { CloseHandle(impl_->handle); impl_->handle = INVALID_HANDLE_VALUE; }
#endif
}

bool Win32SerialTransport::is_open() const noexcept {
#ifdef _WIN32
  return impl_->handle != INVALID_HANDLE_VALUE;
#else
  return false;
#endif
}

IoResult Win32SerialTransport::write_some(std::string_view bytes,
                                          std::chrono::milliseconds timeout) {
#ifdef _WIN32
  if (!is_open()) { return {IoCode::closed, 0U, "COM port is closed"}; }
  if (bytes.empty()) { return {IoCode::error, 0U, "empty write is invalid"}; }
  if (!apply_timeouts(impl_->handle, timeout, false)) {
    return {timeout.count() <= 0 ? IoCode::timeout : IoCode::error, 0U,
            timeout.count() <= 0 ? "positive timeout required" : windows_error("SetCommTimeouts")};
  }
  const auto count = static_cast<DWORD>(std::min<std::size_t>(bytes.size(), MAXDWORD));
  DWORD written = 0;
  if (!WriteFile(impl_->handle, bytes.data(), count, &written, nullptr)) {
    return {IoCode::error, 0U, windows_error("WriteFile")};
  }
  return written == 0U ? IoResult{IoCode::timeout, 0U, "serial write timed out"}
                       : IoResult{IoCode::ok, written, {}};
#else
  (void)bytes; (void)timeout;
  return {IoCode::closed, 0U, "Win32 serial transport is unavailable"};
#endif
}

IoResult Win32SerialTransport::read_some(char* destination, std::size_t capacity,
                                         std::chrono::milliseconds timeout) {
#ifdef _WIN32
  if (!is_open()) { return {IoCode::closed, 0U, "COM port is closed"}; }
  if (destination == nullptr || capacity == 0U) { return {IoCode::error, 0U, "invalid read buffer"}; }
  if (!apply_timeouts(impl_->handle, timeout, true)) {
    return {timeout.count() <= 0 ? IoCode::timeout : IoCode::error, 0U,
            timeout.count() <= 0 ? "positive timeout required" : windows_error("SetCommTimeouts")};
  }
  const auto count = static_cast<DWORD>(std::min<std::size_t>(capacity, MAXDWORD));
  DWORD received = 0;
  if (!ReadFile(impl_->handle, destination, count, &received, nullptr)) {
    return {IoCode::error, 0U, windows_error("ReadFile")};
  }
  return received == 0U ? IoResult{IoCode::timeout, 0U, "serial read timed out"}
                        : IoResult{IoCode::ok, received, {}};
#else
  (void)destination; (void)capacity; (void)timeout;
  return {IoCode::closed, 0U, "Win32 serial transport is unavailable"};
#endif
}

}  // namespace ppbng_rsm400
