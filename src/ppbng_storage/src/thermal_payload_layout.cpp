#include "ppbng_storage/thermal_payload_layout.hpp"

#include <limits>

namespace ppbng_storage
{
namespace
{

bool checked_add(const std::size_t lhs, const std::size_t rhs, std::size_t & result) noexcept
{
  if (rhs > std::numeric_limits<std::size_t>::max() - lhs) {
    return false;
  }
  result = lhs + rhs;
  return true;
}

bool checked_multiply(
  const std::size_t lhs, const std::size_t rhs, std::size_t & result) noexcept
{
  if (lhs != 0 && rhs > std::numeric_limits<std::size_t>::max() / lhs) {
    return false;
  }
  result = lhs * rhs;
  return true;
}

ThermalPayloadValidation failure(const ThermalPayloadError error) noexcept
{
  ThermalPayloadValidation result;
  result.error = error;
  return result;
}

}  // namespace

ThermalPayloadValidation validate_and_split_thermal_payload(
  const ThermalPayloadLayout & layout, const std::size_t payload_size) noexcept
{
  if (layout.width_pixels == 0) {
    return failure(ThermalPayloadError::zero_width);
  }
  if (layout.image_height_rows == 0) {
    return failure(ThermalPayloadError::zero_image_height);
  }
  if (layout.bytes_per_pixel == 0) {
    return failure(ThermalPayloadError::zero_bytes_per_pixel);
  }
  if (layout.header_rows == 0) {
    return failure(ThermalPayloadError::zero_header_rows);
  }

  std::size_t active_row_bytes = 0;
  if (!checked_multiply(layout.width_pixels, layout.bytes_per_pixel, active_row_bytes)) {
    return failure(ThermalPayloadError::arithmetic_overflow);
  }
  if (layout.row_stride_bytes < active_row_bytes) {
    return failure(ThermalPayloadError::stride_too_small);
  }

  std::size_t total_rows = 0;
  if (!checked_add(layout.header_rows, layout.image_height_rows, total_rows)) {
    return failure(ThermalPayloadError::arithmetic_overflow);
  }

  std::size_t expected_payload_size = 0;
  if (!checked_multiply(total_rows, layout.row_stride_bytes, expected_payload_size)) {
    return failure(ThermalPayloadError::arithmetic_overflow);
  }

  if (payload_size != expected_payload_size) {
    ThermalPayloadValidation result = failure(ThermalPayloadError::payload_size_mismatch);
    result.expected_payload_size = expected_payload_size;
    return result;
  }

  std::size_t header_size = 0;
  std::size_t image_storage_size = 0;
  // Both products are bounded by expected_payload_size, but retain checked arithmetic here so
  // this function stays safe if its validation order changes later.
  if (!checked_multiply(layout.header_rows, layout.row_stride_bytes, header_size) ||
    !checked_multiply(layout.image_height_rows, layout.row_stride_bytes, image_storage_size))
  {
    return failure(ThermalPayloadError::arithmetic_overflow);
  }

  ThermalPayloadValidation result;
  result.expected_payload_size = expected_payload_size;
  result.components.full_payload = ByteRange{0, payload_size};
  result.components.header = ByteRange{0, header_size};
  result.components.image_storage = ByteRange{header_size, image_storage_size};
  result.components.active_row_bytes = active_row_bytes;
  result.components.row_stride_bytes = layout.row_stride_bytes;
  result.components.row_padding_bytes = layout.row_stride_bytes - active_row_bytes;
  result.components.image_height_rows = layout.image_height_rows;
  return result;
}

}  // namespace ppbng_storage
