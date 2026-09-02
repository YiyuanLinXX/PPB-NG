#pragma once

#include "ppbng_gnss/um982_protocol.hpp"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace ppbng_gnss {

struct SerialReceiveConfiguration {
  std::string com_path;
  std::uint32_t baud_rate{};
  std::size_t read_chunk_bytes{4096U};
  std::size_t max_line_bytes{65536U};
};

class ReceiveStopToken {
 public:
  bool stop_requested() const noexcept;

 private:
  friend class ReceiveStopSource;
  explicit ReceiveStopToken(std::shared_ptr<std::atomic_bool> flag)
      : flag_(std::move(flag)) {}
  std::shared_ptr<std::atomic_bool> flag_;
};

class ReceiveStopSource {
 public:
  ReceiveStopSource();
  ReceiveStopToken token() const { return ReceiveStopToken(flag_); }
  void request_stop() noexcept;

 private:
  std::shared_ptr<std::atomic_bool> flag_;
};

enum class ByteReadCode { data, timeout, cancelled, disconnected, io_error };

struct ByteReadResult {
  ByteReadCode code{ByteReadCode::timeout};
  std::vector<std::byte> bytes;
  std::string detail;
};

// Receive-only byte source contract. Implementations must never transmit bytes.
class IReceiveOnlyByteSource {
 public:
  virtual ~IReceiveOnlyByteSource() = default;
  virtual ByteReadResult open(const SerialReceiveConfiguration& configuration,
                              std::chrono::milliseconds timeout,
                              const ReceiveStopToken& stop) = 0;
  virtual ByteReadResult read(std::size_t maximum_bytes,
                              std::chrono::milliseconds timeout,
                              const ReceiveStopToken& stop) = 0;
  virtual void close() noexcept = 0;
  virtual bool is_open() const noexcept = 0;
};

// Construction performs no discovery and never opens a COM port. open() uses
// Win32 GENERIC_READ only; this class intentionally has no write API.
class WindowsReceiveOnlySerial final : public IReceiveOnlyByteSource {
 public:
  WindowsReceiveOnlySerial();
  ~WindowsReceiveOnlySerial() override;
  WindowsReceiveOnlySerial(const WindowsReceiveOnlySerial&) = delete;
  WindowsReceiveOnlySerial& operator=(const WindowsReceiveOnlySerial&) = delete;

  ByteReadResult open(const SerialReceiveConfiguration& configuration,
                      std::chrono::milliseconds timeout,
                      const ReceiveStopToken& stop) override;
  ByteReadResult read(std::size_t maximum_bytes,
                      std::chrono::milliseconds timeout,
                      const ReceiveStopToken& stop) override;
  void close() noexcept override;
  bool is_open() const noexcept override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

enum class ReceiverState { idle, connected, disconnected, faulted, stopped };
enum class SentenceKind { gga, uniheadinga, other, parse_error };

struct ReceivedSentence {
  std::string raw_line;
  std::chrono::system_clock::time_point host_receive_time;
  SentenceKind kind{SentenceKind::other};
  std::optional<GgaFix> gga;
  std::optional<HeadingSolution> heading;
  std::string parse_error;
};

struct ReceiverPollResult {
  ByteReadCode code{ByteReadCode::timeout};
  std::vector<ReceivedSentence> sentences;
  std::string detail;
};

class Um982Receiver {
 public:
  explicit Um982Receiver(std::unique_ptr<IReceiveOnlyByteSource> source);
  ~Um982Receiver();
  Um982Receiver(const Um982Receiver&) = delete;
  Um982Receiver& operator=(const Um982Receiver&) = delete;

  // The only operations which may open the configured port are connect() and
  // recover(). Neither the receiver nor the transport has any transmit API.
  ByteReadResult connect(const SerialReceiveConfiguration& configuration,
                         std::chrono::milliseconds timeout,
                         const ReceiveStopToken& stop);
  ReceiverPollResult poll(std::chrono::milliseconds timeout,
                          const ReceiveStopToken& stop);
  ByteReadResult recover(std::chrono::milliseconds timeout,
                         const ReceiveStopToken& stop);
  void stop() noexcept;

  ReceiverState state() const noexcept { return state_; }
  std::uint64_t connection_epoch() const noexcept { return connection_epoch_; }
  const std::optional<SerialReceiveConfiguration>& configuration() const noexcept {
    return configuration_;
  }

 private:
  std::unique_ptr<IReceiveOnlyByteSource> source_;
  std::optional<SerialReceiveConfiguration> configuration_;
  std::unique_ptr<IncrementalLineDecoder> decoder_;
  ReceiverState state_{ReceiverState::idle};
  std::uint64_t connection_epoch_{0U};
};

}  // namespace ppbng_gnss
