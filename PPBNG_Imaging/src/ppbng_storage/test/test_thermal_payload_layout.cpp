#include "ppbng_storage/thermal_payload_layout.hpp"

#include <cstddef>
#include <limits>

#include <gtest/gtest.h>

using ppbng_storage::ThermalPayloadError;
using ppbng_storage::ThermalPayloadLayout;
using ppbng_storage::validate_and_split_thermal_payload;

TEST(ThermalPayloadLayout, SplitsA6701HeaderFirstWithoutDroppingPayload)
{
  constexpr std::size_t payload_size = 656'640;
  const auto result = validate_and_split_thermal_payload(
    ThermalPayloadLayout::a6701_mono16(), payload_size);

  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.expected_payload_size, payload_size);
  EXPECT_EQ(result.components.full_payload.offset, 0U);
  EXPECT_EQ(result.components.full_payload.size, payload_size);
  EXPECT_EQ(result.components.header.offset, 0U);
  EXPECT_EQ(result.components.header.size, 1'280U);
  EXPECT_EQ(result.components.image_storage.offset, 1'280U);
  EXPECT_EQ(result.components.image_storage.size, 655'360U);
  EXPECT_EQ(
    result.components.header.size + result.components.image_storage.size,
    result.components.full_payload.size);
  EXPECT_EQ(result.components.active_row_bytes, 1'280U);
  EXPECT_EQ(result.components.row_padding_bytes, 0U);
}

TEST(ThermalPayloadLayout, RejectsPayloadLengthMismatchAndReportsExpectedLength)
{
  const auto short_result = validate_and_split_thermal_payload(
    ThermalPayloadLayout::a6701_mono16(), 656'639);
  const auto long_result = validate_and_split_thermal_payload(
    ThermalPayloadLayout::a6701_mono16(), 656'641);

  EXPECT_EQ(short_result.error, ThermalPayloadError::payload_size_mismatch);
  EXPECT_EQ(short_result.expected_payload_size, 656'640U);
  EXPECT_EQ(long_result.error, ThermalPayloadError::payload_size_mismatch);
  EXPECT_EQ(long_result.expected_payload_size, 656'640U);
}

TEST(ThermalPayloadLayout, AccountsForRowPaddingInHeaderAndImageStorage)
{
  const ThermalPayloadLayout layout{
    4,  // pixels
    2,  // image rows
    2,  // bytes per pixel
    12, // stride: 8 active bytes plus 4 padding bytes
    1}; // header rows

  const auto result = validate_and_split_thermal_payload(layout, 36);

  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.components.header.offset, 0U);
  EXPECT_EQ(result.components.header.size, 12U);
  EXPECT_EQ(result.components.image_storage.offset, 12U);
  EXPECT_EQ(result.components.image_storage.size, 24U);
  EXPECT_EQ(result.components.active_row_bytes, 8U);
  EXPECT_EQ(result.components.row_stride_bytes, 12U);
  EXPECT_EQ(result.components.row_padding_bytes, 4U);
}

TEST(ThermalPayloadLayout, RejectsStrideSmallerThanActivePixels)
{
  const ThermalPayloadLayout layout{640, 512, 2, 1'279, 1};
  const auto result = validate_and_split_thermal_payload(layout, 0);
  EXPECT_EQ(result.error, ThermalPayloadError::stride_too_small);
}

TEST(ThermalPayloadLayout, RejectsIllegalZeroDimensions)
{
  EXPECT_EQ(
    validate_and_split_thermal_payload({0, 512, 2, 1'280, 1}, 0).error,
    ThermalPayloadError::zero_width);
  EXPECT_EQ(
    validate_and_split_thermal_payload({640, 0, 2, 1'280, 1}, 0).error,
    ThermalPayloadError::zero_image_height);
  EXPECT_EQ(
    validate_and_split_thermal_payload({640, 512, 0, 1'280, 1}, 0).error,
    ThermalPayloadError::zero_bytes_per_pixel);
  EXPECT_EQ(
    validate_and_split_thermal_payload({640, 512, 2, 1'280, 0}, 0).error,
    ThermalPayloadError::zero_header_rows);
}

TEST(ThermalPayloadLayout, RejectsArithmeticOverflow)
{
  constexpr auto maximum = std::numeric_limits<std::size_t>::max();

  EXPECT_EQ(
    validate_and_split_thermal_payload({maximum, 1, 2, maximum, 1}, 0).error,
    ThermalPayloadError::arithmetic_overflow);
  EXPECT_EQ(
    validate_and_split_thermal_payload({1, maximum, 1, 1, 1}, 0).error,
    ThermalPayloadError::arithmetic_overflow);
  EXPECT_EQ(
    validate_and_split_thermal_payload({1, maximum / 2, 1, 3, 1}, 0).error,
    ThermalPayloadError::arithmetic_overflow);
}

