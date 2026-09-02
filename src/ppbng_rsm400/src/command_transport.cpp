#include "ppbng_rsm400/command_transport.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <stdexcept>
#include <utility>

namespace ppbng_rsm400 {
namespace {

using Clock = std::chrono::steady_clock;

struct PreparedCommand {
  TransactionCode code{TransactionCode::ok};
  std::string detail;
  std::vector<Message> messages;
};

PreparedCommand feature_failure(FeatureAvailability availability, const char* feature) {
  if (availability == FeatureAvailability::unknown) {
    return {TransactionCode::feature_unknown,
            std::string(feature) + " availability has not been confirmed", {}};
  }
  if (availability == FeatureAvailability::unavailable) {
    return {TransactionCode::feature_unavailable,
            std::string(feature) + " is not unlocked on this Mount", {}};
  }
  return {};
}

PreparedCommand prepare(const ControlRequest& request, const ConfirmedFeatures& features) {
  switch (request.kind) {
    case ControlKind::activate_horizon_stabilization:
      // ICD 5.4.1: ST 1 activates horizon axes only and leaves drift unchanged.
      return {TransactionCode::ok, {}, {{"ST", std::string("1"), "ST 1"}}};
    case ControlKind::trigger_fast_level:
      // ICD 5.4.1: FL has no argument and is valid during STAB mode.
      return {TransactionCode::ok, {}, {{"FL", std::nullopt, "FL"}}};
    case ControlKind::set_leveling_target: {
      auto gate = feature_failure(features.of002_leveling_offset, "OF002");
      if (gate.code != TransactionCode::ok) { return gate; }
      if (request.roll_centidegrees < -3000 || request.roll_centidegrees > 3000 ||
          request.pitch_centidegrees < -3000 || request.pitch_centidegrees > 3000) {
        return {TransactionCode::invalid_argument,
                "OFR/OFP must be within -3000..3000 centidegrees", {}};
      }
      return {TransactionCode::ok, {},
              {{"OFR", std::to_string(request.roll_centidegrees),
                "OFR " + std::to_string(request.roll_centidegrees)},
               {"OFP", std::to_string(request.pitch_centidegrees),
                "OFP " + std::to_string(request.pitch_centidegrees)}}};
    }
    case ControlKind::reset_errors: {
      auto gate = feature_failure(features.of005_status_analysis, "OF005");
      if (gate.code != TransactionCode::ok) { return gate; }
      // ICD 5.4.3: RER resets errors except built-in-test errors.
      return {TransactionCode::ok, {}, {{"RER", std::nullopt, "RER"}}};
    }
  }
  return {TransactionCode::invalid_argument, "unknown control kind", {}};
}

std::string serialize(const std::vector<Message>& messages) {
  std::string tail = "H /";  // ASCII '@' base + ACKN bit: acknowledgement required.
  for (const auto& message : messages) {
    if (message.command.empty() || message.command.size() > 3U ||
        !std::all_of(message.command.begin(), message.command.end(),
                     [](char c) { return c >= 'A' && c <= 'Z'; })) {
      throw std::invalid_argument("invalid MCP command token");
    }
    tail += message.command;
    if (message.argument.has_value()) {
      if (message.argument->empty() || message.argument->find_first_of("/\r\n") != std::string::npos) {
        throw std::invalid_argument("invalid MCP command argument");
      }
      tail += " " + *message.argument;
    }
    tail += '/';
  }
  tail += "\r\n";
  const auto checksum = calculate_checksum(tail);
  auto digits = std::to_string(checksum);
  digits.insert(digits.begin(), 3U - digits.size(), '0');
  const auto frame = "VM" + digits + tail;
  if (frame.size() > kMcp2MaximumFrameBytes || messages.size() > kMcp2MaximumMessages) {
    throw std::invalid_argument("MCP command frame exceeds documented limit");
  }
  return frame;
}

std::chrono::milliseconds remaining(Clock::time_point deadline) {
  const auto now = Clock::now();
  if (now >= deadline) { return std::chrono::milliseconds(0); }
  const auto value = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
  return std::max(value, std::chrono::milliseconds(1));
}

TransactionCode map_io(IoCode code) {
  return code == IoCode::timeout ? TransactionCode::timeout : TransactionCode::transport_error;
}

}  // namespace

CommandClient::CommandClient(IByteTransport& transport, CommandClientOptions options)
    : transport_(transport), options_(std::move(options)) {}

TransactionResult CommandClient::execute(const ControlRequest& request,
                                         std::chrono::milliseconds timeout) {
  std::lock_guard<std::mutex> lock(mutex_);  // MCP 2.0 has no on-wire sequence field.
  TransactionResult result;
  result.local_sequence = ++last_sequence_;
  if (timeout.count() <= 0) {
    result.code = TransactionCode::invalid_argument;
    result.detail = "positive transaction timeout required";
    return result;
  }
  if (!options_.allow_control) {
    result.code = TransactionCode::control_disabled;
    result.detail = "observe-only client: allow_control=true was not explicitly set";
    return result;
  }
  const auto prepared = prepare(request, options_.features);
  if (prepared.code != TransactionCode::ok) {
    result.code = prepared.code;
    result.detail = prepared.detail;
    return result;
  }

  std::string frame;
  try { frame = serialize(prepared.messages); }
  catch (const std::exception& error) {
    result.code = TransactionCode::invalid_argument;
    result.detail = error.what();
    return result;
  }

  const auto deadline = Clock::now() + timeout;
  for (;;) {
    std::size_t sent = 0U;
    while (sent < frame.size()) {
      const auto budget = remaining(deadline);
      if (budget.count() == 0) {
        result.code = TransactionCode::timeout;
        result.detail = "timeout during MCP frame write";
        return result;
      }
      const auto io = transport_.write_some(std::string_view(frame).substr(sent), budget);
      if (io.code != IoCode::ok || io.bytes == 0U || io.bytes > frame.size() - sent) {
        result.code = io.code == IoCode::ok ? TransactionCode::transport_error : map_io(io.code);
        result.detail = io.detail.empty() ? "invalid/failed partial write" : io.detail;
        return result;
      }
      sent += io.bytes;
    }

    for (;;) {
      const auto budget = remaining(deadline);
      if (budget.count() == 0) {
        result.code = TransactionCode::timeout;
        result.detail = "timeout waiting for MCP acknowledgement";
        return result;
      }
      std::array<char, 256U> bytes{};
      const auto io = transport_.read_some(bytes.data(), bytes.size(), budget);
      if (io.code != IoCode::ok) {
        result.code = map_io(io.code);
        result.detail = io.detail.empty() ? "MCP acknowledgement read failed" : io.detail;
        return result;
      }
      if (io.bytes == 0U || io.bytes > bytes.size()) {
        result.code = TransactionCode::transport_error;
        result.detail = "transport returned an invalid empty/oversize read";
        return result;
      }
      const auto candidates = decoder_.feed(std::string_view(bytes.data(), io.bytes));
      for (const auto& candidate : candidates) {
        Frame response;
        try { response = parse_frame(candidate); }
        catch (const std::exception& error) {
          result.code = TransactionCode::protocol_error;
          result.detail = error.what();
          return result;
        }
        const auto& status = response.connection_status;
        if (!status.acknowledgement) {
          if (status.retransmission_requested) {
            if (result.retransmissions >= options_.maximum_retransmissions) {
              result.code = TransactionCode::retransmission_limit;
              result.detail = "Mount retransmission request limit reached";
              return result;
            }
            ++result.retransmissions;
            decoder_.reset();
            goto retransmit;
          }
          result.unsolicited_frames.push_back(std::move(response));
          continue;
        }
        if (status.command_error) {
          result.code = TransactionCode::command_rejected;
          result.detail = "Mount set CMD ERROR in acknowledgement";
          return result;
        }
        if (status.checksum_error || status.retransmission_requested) {
          if (status.retransmission_requested &&
              result.retransmissions < options_.maximum_retransmissions) {
            ++result.retransmissions;
            decoder_.reset();
            goto retransmit;
          }
          result.code = status.retransmission_requested ? TransactionCode::retransmission_limit
                                                        : TransactionCode::protocol_error;
          result.detail = "Mount acknowledgement reports checksum/retransmission error";
          return result;
        }
        // Documented command acknowledgements are empty; data-bearing ACK cannot
        // be correlated because MCP 2.0 has no command echo or sequence field.
        if (!response.messages.empty()) {
          result.code = TransactionCode::acknowledgement_mismatch;
          result.detail = "data-bearing acknowledgement cannot be correlated to control transaction";
          return result;
        }
        result.code = TransactionCode::ok;
        return result;
      }
    }
retransmit:
    continue;
  }
}

std::uint64_t CommandClient::last_local_sequence() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_sequence_;
}

}  // namespace ppbng_rsm400
