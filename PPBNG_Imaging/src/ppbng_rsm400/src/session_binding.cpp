#include "ppbng_rsm400/session_binding.hpp"

#include <system_error>

namespace ppbng_rsm400
{
SessionBindingResult SessionBinding::prepare(const std::string & request_id,
  const std::string & session_id, const std::string & session_directory,
  const bool started, const bool armed)
{
  if (started) return {false, false, "prepare is forbidden while started"};
  const auto prior = history_.find(request_id);
  if (prior != history_.end()) {
    if (prior->second.session_id == session_id && prior->second.directory == session_directory) {
      return {true, true, "duplicate prepare accepted idempotently"};
    }
    return {false, false, "request_id was reused with different payload"};
  }
  if (armed) return {false, false, "prepare is forbidden while armed"};
  if (request_id.empty() || session_id.empty() || session_directory.empty()) {
    return {false, false, "request_id, session_id, and session_directory are required"};
  }
  std::error_code ec;
  if (root_.empty() || !std::filesystem::is_directory(root_, ec)) {
    return {false, false, "allowed_output_root is missing or invalid"};
  }
  const auto root = std::filesystem::weakly_canonical(root_, ec);
  if (ec) return {false, false, "allowed_output_root canonicalization failed"};
  const std::filesystem::path requested(session_directory);
  if (!std::filesystem::is_directory(requested, ec)) {
    return {false, false, "session_directory must already exist"};
  }
  const auto session = std::filesystem::weakly_canonical(requested, ec);
  const auto relative = session.lexically_relative(root);
  if (ec || relative.empty() || relative == "." || relative.is_absolute() ||
    *relative.begin() == "..")
  {
    return {false, false, "session_directory must be a strict child of allowed_output_root"};
  }
  const auto candidate = session / "segments";
  if (!std::filesystem::is_directory(candidate, ec)) {
    return {false, false, "existing segments directory is required"};
  }
  const auto segments = std::filesystem::weakly_canonical(candidate, ec);
  const auto segment_relative = segments.lexically_relative(session);
  if (ec || segment_relative.empty() || segment_relative == "." ||
    segment_relative.is_absolute() || *segment_relative.begin() == "..")
  {
    return {false, false, "segments directory escapes the canonical session"};
  }
  history_.emplace(request_id, Payload{session_id, session_directory});
  session_id_ = session_id;
  segments_ = segments;
  bound_ = true;
  return {true, false, "existing session bound without creating or overwriting it"};
}
}  // namespace ppbng_rsm400
