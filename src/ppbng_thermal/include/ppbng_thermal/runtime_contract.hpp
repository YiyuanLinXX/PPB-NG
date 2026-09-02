#pragma once
#include "ppbng_thermal/thermal_adapter.hpp"
#include <cstddef>
#include <cstdint>
#include <string>
namespace ppbng_thermal {
struct RuntimeFrameDescriptor {
  std::uint32_t transport_width{},transport_height{},row_stride_bytes{};
  std::uint64_t payload_size_bytes{};
  std::uint32_t image_y{},image_width{},image_height{};
  std::uint64_t image_data_offset_bytes{},image_data_length_bytes{};
  std::uint32_t auxiliary_row_count{};
  std::uint64_t auxiliary_data_offset_bytes{},auxiliary_data_length_bytes{};
  std::string auxiliary_type;bool raw_payload_preserved{},complete{};
  std::uint64_t sdk_frame_counter{},camera_timestamp_ns{},host_receive_monotonic_ns{};
  std::string validation_detail;
};
RuntimeFrameDescriptor describe_runtime_frame(const FrameInfo&,const ThermalReadback&);
Status validate_inert_arm_preflight(bool hardware_authorized,bool prepared,
  const std::string&device_id,const std::string&expected_model,const std::string&trigger_channel,
  const ThermalConfiguration&config,std::size_t association_capacity,std::uint64_t association_wait_ns);
} // namespace ppbng_thermal
