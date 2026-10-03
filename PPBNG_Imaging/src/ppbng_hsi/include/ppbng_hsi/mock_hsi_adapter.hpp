#pragma once

#include "ppbng_hsi/hsi_adapter.hpp"

#include <cstdint>
#include <optional>

namespace ppbng_hsi
{

enum class FailurePoint : std::uint8_t
{
  connect = 0,
  configure = 1,
  close_shutter = 2,
  begin_dark = 3,
  open_shutter = 4,
  start_streaming = 5,
  acquire_line = 6,
  recover = 7,
};

class MockHsiAdapter final : public IHsiAdapter
{
public:
  explicit MockHsiAdapter(CameraKind kind);

  [[nodiscard]] CameraKind kind() const noexcept override;
  [[nodiscard]] HsiState state() const noexcept override;
  [[nodiscard]] const HsiConfig & config() const noexcept override;
  [[nodiscard]] std::uint32_t segment_id() const noexcept override;

  OperationResult connect() override;
  OperationResult configure(const HsiConfig & config) override;
  OperationResult close_shutter() override;
  OperationResult begin_dark_capture(std::size_t line_count) override;
  OperationResult open_shutter() override;
  OperationResult start_streaming() override;
  OperationResult stop_streaming() override;
  OperationResult recover() override;
  LineResult on_trigger(const TriggerEvent & trigger) override;
  LineResult poll_internal() override;

  void fail_next(FailurePoint point) noexcept;
  void disconnect_on_next_line() noexcept;

private:
  [[nodiscard]] bool consume_failure(FailurePoint point) noexcept;
  [[nodiscard]] OperationResult fail(FailurePoint point, const char * message);
  [[nodiscard]] LineResult make_line(const TriggerEvent & trigger, CaptureKind capture_kind);

  CameraKind kind_;
  HsiState state_{HsiState::disconnected};
  HsiConfig config_;
  bool configured_{false};
  bool shutter_open_before_recovery_{false};
  bool has_started_segment_{false};
  std::uint32_t segment_id_{0};
  std::uint64_t camera_line_sequence_{0};
  std::uint64_t segment_sample_line_index_{0};
  std::uint64_t segment_dark_line_index_{0};
  std::optional<std::uint64_t> last_trigger_sequence_;
  std::uint64_t internal_sequence_{0U};
  std::size_t dark_lines_remaining_{0};
  std::optional<FailurePoint> next_failure_;
  bool disconnect_on_next_line_{false};
};

}  // namespace ppbng_hsi
