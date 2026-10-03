#pragma once

#include "ppbng_hsi/hsi_types.hpp"

#include <cstdint>
#include <string>

namespace ppbng_hsi
{

// Stateless helpers shared by both camera adapters.
class HsiFormat
{
public:
  [[nodiscard]] static OperationResult validate_config(const HsiConfig & config);
  [[nodiscard]] static OperationResult resolve_exposure(
    double requested_us, double minimum_us, double maximum_us,
    const std::string & policy, double & effective_us);
  [[nodiscard]] static std::uint64_t payload_bytes_per_line(const HsiConfig & config);
  [[nodiscard]] static EnviLayout make_envi_layout(
    const HsiConfig & config,
    std::uint32_t segment_id,
    std::uint64_t line_count,
    const std::string & stem);
  [[nodiscard]] static std::string render_envi_header(const EnviLayout & layout);
};

}  // namespace ppbng_hsi
