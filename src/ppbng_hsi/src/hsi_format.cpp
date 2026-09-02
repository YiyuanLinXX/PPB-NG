#include "ppbng_hsi/hsi_format.hpp"

#include <cmath>
#include <limits>
#include <sstream>

namespace ppbng_hsi
{

OperationResult HsiFormat::validate_config(const HsiConfig & config)
{
  if (config.device_id.empty()) {
    return {false, "device_id must not be empty"};
  }
  if (config.trigger_mode != "Internal" && config.trigger_mode != "External") {
    return {false, "trigger_mode must be exactly Internal or External"};
  }
  if (config.trigger_mode == "External" && config.trigger_channel.empty()) {
    return {false, "trigger_channel must not be empty in External mode"};
  }
  if (config.spatial_samples == 0 || config.spectral_bands == 0) {
    return {false, "spatial_samples and spectral_bands must be non-zero"};
  }
  if (!std::isfinite(config.line_rate_hz) || config.line_rate_hz <= 0.0) {
    return {false, "line_rate_hz must be finite and positive"};
  }
  if (!std::isfinite(config.exposure_us) || config.exposure_us <= 0.0) {
    return {false, "exposure_us must be finite and positive"};
  }
  const auto bytes = payload_bytes_per_line(config);
  if (bytes == 0) {
    return {false, "line payload size overflows uint64"};
  }
  return {true, "configuration valid"};
}

std::uint64_t HsiFormat::payload_bytes_per_line(const HsiConfig & config)
{
  constexpr std::uint64_t bytes_per_sample = sizeof(std::uint16_t);
  const auto samples = static_cast<std::uint64_t>(config.spatial_samples);
  const auto bands = static_cast<std::uint64_t>(config.spectral_bands);
  if (samples == 0 || bands == 0) {
    return 0;
  }
  if (samples > std::numeric_limits<std::uint64_t>::max() / bands ||
    samples * bands > std::numeric_limits<std::uint64_t>::max() / bytes_per_sample)
  {
    return 0;
  }
  return samples * bands * bytes_per_sample;
}

EnviLayout HsiFormat::make_envi_layout(
  const HsiConfig & config,
  const std::uint32_t segment_id,
  const std::uint64_t line_count,
  const std::string & stem)
{
  EnviLayout layout;
  layout.samples = config.spatial_samples;
  layout.lines = line_count;
  layout.bands = config.spectral_bands;
  layout.bytes_per_line = payload_bytes_per_line(config);
  const std::string segment = "_segment_" + std::to_string(segment_id);
  layout.raw_filename = stem + segment + ".raw";
  layout.header_filename = stem + segment + ".hdr";
  layout.timestamp_filename = stem + segment + ".timestamps.bin";
  layout.index_filename = stem + segment + ".index.bin";
  return layout;
}

std::string HsiFormat::render_envi_header(const EnviLayout & layout)
{
  std::ostringstream output;
  output << "ENVI\n"
         << "samples = " << layout.samples << '\n'
         << "lines = " << layout.lines << '\n'
         << "bands = " << layout.bands << '\n'
         << "header offset = " << layout.header_offset << '\n'
         << "file type = ENVI Standard\n"
         << "data type = " << layout.data_type << '\n'
         << "interleave = " << layout.interleave << '\n'
         << "byte order = " << static_cast<unsigned int>(layout.byte_order) << '\n';
  return output.str();
}

}  // namespace ppbng_hsi
