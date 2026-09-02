#pragma once

#include "ppbng_hsi/hsi_types.hpp"

#include <optional>
#include <string>

#include <ppbng_interfaces/msg/sample_stamp.hpp>

namespace ppbng_hsi
{

// Returns no value for dark references. Non-MATCHED scene lines remain publishable for
// audit, but their canonical time status is forcibly UNSYNCED while raw trigger UTC,
// uncertainty and the original trigger status remain explicit in TimeQuality.detail.
std::optional<ppbng_interfaces::msg::SampleStamp> make_sample_stamp(
  const LineRecord & line, const std::string & session_id);

}  // namespace ppbng_hsi
