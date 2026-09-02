#pragma once

#include "ppbng_hsi/hsi_adapter.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace ppbng_hsi
{

enum class SpecSensorTransport : std::uint8_t {pleora_gige = 0, ni_camera_link = 1};

struct SpecSensorBackendOptions
{
  CameraKind kind{CameraKind::fx10e};
  SpecSensorTransport transport{SpecSensorTransport::pleora_gige};
  int device_index{-1};
  std::wstring license_path;
  std::string expected_profile_name;
  std::string expected_sensor_serial;
  std::wstring calibration_pack_path;
  std::wstring grabber_channel;
  std::wstring ni_grabber_channel;
  std::wstring ni_camera_file_path;
  std::wstring ni_camera_serial_port;
  std::uint32_t pleora_packet_size{0U};
  std::uint32_t initialization_timeout_ms{5000U};
  std::size_t callback_queue_capacity{128U};
  std::size_t maximum_frame_bytes{8U * 1024U * 1024U};
};

struct SpecSensorReadback
{
  bool valid{false};
  double frame_rate_hz{0.0};
  double exposure_us{0.0};
  std::string trigger_mode;
  std::uint32_t width{0U};
  std::uint32_t height{0U};
  std::uint32_t byte_depth{0U};
  std::uint64_t frame_bytes{0U};
  std::string sensor_serial;
  std::string profile_name;
  bool calibration_pack_loaded{false};
  std::uint32_t pleora_packet_size{0U};
};

struct SpecSensorFrame
{
  std::int64_t sdk_frame_number{0};
  std::uint64_t host_receive_monotonic_ns{0};
  std::vector<std::uint8_t> bytes;
};

struct SpecSensorQueueStats
{
  std::size_t capacity{0U};
  std::size_t depth{0U};
  std::uint64_t overflow_count{0U};
  std::uint64_t invalid_frame_count{0U};
};

// Preallocates every slot. The producer callback performs only validation, memcpy and enqueue.
class BoundedSpecSensorFrameQueue
{
public:
  BoundedSpecSensorFrameQueue(std::size_t capacity, std::size_t maximum_frame_bytes);
  ~BoundedSpecSensorFrameQueue();
  BoundedSpecSensorFrameQueue(const BoundedSpecSensorFrameQueue &) = delete;
  BoundedSpecSensorFrameQueue & operator=(const BoundedSpecSensorFrameQueue &) = delete;

  bool try_push(const std::uint8_t * data, std::size_t size, std::int64_t frame_number,
    std::uint64_t host_receive_monotonic_ns) noexcept;
  bool try_pop(SpecSensorFrame & output);
  void clear() noexcept;
  [[nodiscard]] std::size_t size() const noexcept;
  [[nodiscard]] std::size_t capacity() const noexcept;
  [[nodiscard]] std::uint64_t overflow_count() const noexcept;
  [[nodiscard]] std::uint64_t invalid_frame_count() const noexcept;
  [[nodiscard]] SpecSensorQueueStats stats() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

[[nodiscard]] OperationResult validate_specsensor_options(
  const SpecSensorBackendOptions & options);
[[nodiscard]] bool specsensor_backend_compiled() noexcept;

class SpecSensorHsiAdapter final : public IHsiAdapter
{
public:
  explicit SpecSensorHsiAdapter(SpecSensorBackendOptions options);
  ~SpecSensorHsiAdapter() override;
  SpecSensorHsiAdapter(const SpecSensorHsiAdapter &) = delete;
  SpecSensorHsiAdapter & operator=(const SpecSensorHsiAdapter &) = delete;

  [[nodiscard]] CameraKind kind() const noexcept override;
  [[nodiscard]] HsiState state() const noexcept override;
  [[nodiscard]] const HsiConfig & config() const noexcept override;
  [[nodiscard]] std::uint32_t segment_id() const noexcept override;
  [[nodiscard]] SpecSensorReadback readback() const;
  [[nodiscard]] SpecSensorQueueStats queue_stats() const noexcept;

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

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ppbng_hsi
