#pragma once

#include <string>
#include <vector>

namespace ppbng_runtime
{
// A delimiter-safe UTF-8 payload for the local text control bridge. The operator
// must receive warnings even when the workflow remains in RECORDING (state 6).
inline std::string operator_warning_status(const std::vector<std::string> & warnings)
{
  std::string text;
  for (const auto & warning : warnings) {
    if (!text.empty()) text += '\n';
    text += warning;
  }
  constexpr char digits[] = "0123456789abcdef";
  std::string encoded;
  for (const unsigned char value : text) {
    encoded += digits[value >> 4U];
    encoded += digits[value & 15U];
  }
  return ";operator_warning_count=" + std::to_string(warnings.size()) +
         ";operator_warnings_hex=" + encoded + ";";
}
}  // namespace ppbng_runtime
