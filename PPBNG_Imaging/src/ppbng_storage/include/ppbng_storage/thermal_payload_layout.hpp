#pragma once

#include <cstddef>

namespace ppbng_storage
{

struct ByteRange
{
  std::size_t offset{0};
  std::size_t size{0};
};

struct ThermalPayloadLayout
{
  std::size_t width_pixels{0};
  std::size_t image_height_rows{0};
  std::size_t bytes_per_pixel{0};
  std::size_t row_stride_bytes{0};
  std::size_t header_rows{0};

  static constexpr ThermalPayloadLayout a6701_mono16() noexcept
  {
    return ThermalPayloadLayout{640, 512, 2, 1280, 1};
  }
};

enum class ThermalPayloadError
{
  none,
  zero_width,
  zero_image_height,
  zero_bytes_per_pixel,
  zero_header_rows,
  arithmetic_overflow,
  stride_too_small,
  payload_size_mismatch,
};

struct ThermalPayloadComponents
{
  // These ranges refer to the caller-owned payload. No bytes are copied or discarded.
  ByteRange full_payload;
  ByteRange header;
  ByteRange image_storage;
  std::size_t active_row_bytes{0};
  std::size_t row_stride_bytes{0};
  std::size_t row_padding_bytes{0};
  std::size_t image_height_rows{0};
};

struct ThermalPayloadValidation
{
  ThermalPayloadError error{ThermalPayloadError::none};
  std::size_t expected_payload_size{0};
  ThermalPayloadComponents components{};

  constexpr bool ok() const noexcept
  {
    return error == ThermalPayloadError::none;
  }
};

// Validates an exact payload size and returns byte offsets into the original buffer.
// The header is always the first header_rows rows, followed by image storage rows.
ThermalPayloadValidation validate_and_split_thermal_payload(
  const ThermalPayloadLayout & layout, std::size_t payload_size) noexcept;

}  // namespace ppbng_storage
