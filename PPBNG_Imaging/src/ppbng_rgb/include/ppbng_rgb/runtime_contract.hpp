#pragma once
#include "ppbng_rgb/rgb_adapter.hpp"
#include <cstddef>
#include <cstdint>
#include <string>
namespace ppbng_rgb {
struct RuntimeFrameDescriptor {
  std::uint32_t transport_width{}; std::uint32_t transport_height{};
  std::uint32_t row_stride_bytes{}; std::uint64_t payload_size_bytes{};
  std::string pixel_format; bool raw_payload_preserved{}; bool complete{};
  std::uint64_t sdk_frame_counter{}; std::uint64_t camera_timestamp_ns{};
  std::uint64_t host_receive_monotonic_ns{};
  bool chunk_data_valid{};
  bool chunk_frame_id_valid{}; std::uint64_t chunk_frame_id{};
  bool chunk_timestamp_valid{}; std::uint64_t chunk_timestamp{};
  bool exposure_time_valid{}; double exposure_time_us{};
  bool gain_valid{}; double gain_db{};
  bool black_level_valid{}; double black_level{};
  bool white_balance_red_valid{}; double white_balance_red{};
  bool white_balance_blue_valid{}; double white_balance_blue{};
  std::string exposure_auto,gain_auto,balance_white_auto;
  std::string validation_detail;
};
RuntimeFrameDescriptor describe_runtime_frame(const FrameInfo&,const RgbReadback&);
Status validate_inert_arm_preflight(bool hardware_authorized,bool prepared,
  const std::string&device_id,const std::string&trigger_channel,
  const RgbConfiguration&config,std::size_t association_capacity,std::uint64_t association_wait_ns);
} // namespace ppbng_rgb
