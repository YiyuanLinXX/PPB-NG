#pragma once

#include "ppbng_rsm400/command_transport.hpp"

#include <memory>
#include <string>

namespace ppbng_rsm400 {

struct SerialOpenResult {
  IoCode code{IoCode::ok};
  std::string detail;
  bool ok() const noexcept { return code == IoCode::ok; }
};

// Construction is inert. open_port() is the only operation that opens a COM
// handle and must be invoked only after the project's explicit hardware approval.
class Win32SerialTransport final : public IByteTransport {
 public:
  Win32SerialTransport();
  ~Win32SerialTransport() override;
  Win32SerialTransport(const Win32SerialTransport&) = delete;
  Win32SerialTransport& operator=(const Win32SerialTransport&) = delete;

  SerialOpenResult open_port(const std::string& com_port);
  void close() noexcept;
  bool is_open() const noexcept;
  IoResult write_some(std::string_view bytes, std::chrono::milliseconds timeout) override;
  IoResult read_some(char* destination, std::size_t capacity,
                     std::chrono::milliseconds timeout) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ppbng_rsm400
