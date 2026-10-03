#include "ppbng_gnss/session_binding.hpp"

#include <system_error>

namespace ppbng_gnss
{
SessionBindingResult SessionBinding::prepare(const std::string & request_id,
  const std::string & session_id, const std::string & session_directory,
  const bool started, const bool may_rebind)
{
  if (started) {return {false, false, "prepare is forbidden while started"};}
  const auto previous = history_.find(request_id);
  if (previous != history_.end()) {
    if (session_id == previous->second.session_id && session_directory == previous->second.directory) {
      return {true, true, "duplicate prepare request accepted idempotently"};
    }
    return {false, false, "request_id was reused with different payload"};
  }
  if (bound_ && !may_rebind) {return {false, false, "cannot rebind an armed device"};}
  if (request_id.empty() || session_id.empty() || session_directory.empty()) {
    return {false, false, "request_id, session_id, and session_directory are required"};
  }
  std::error_code ec;
  if (allowed_output_root_.empty() || !std::filesystem::is_directory(allowed_output_root_, ec)) {
    return {false, false, "configured allowed_output_root is missing or invalid"};
  }
  const auto root = std::filesystem::weakly_canonical(allowed_output_root_, ec);
  if (ec) {return {false, false, "allowed_output_root canonicalization failed"};}
  const std::filesystem::path path(session_directory);
  if (!std::filesystem::is_directory(path, ec)) {
    return {false, false, "session_directory must already exist and be a directory"};
  }
  const auto canonical = std::filesystem::weakly_canonical(path, ec);
  if (ec) {return {false, false, "session_directory canonicalization failed"};}
  const auto relative = canonical.lexically_relative(root);
  if (relative.empty() || relative == "." || relative.is_absolute() ||
    (!relative.empty() && *relative.begin() == ".."))
  {
    return {false, false, "session_directory must be a strict descendant of allowed_output_root"};
  }
  if (!std::filesystem::is_directory(canonical / "segments", ec)) {
    return {false, false, "prepared dataset session must contain its existing segments directory"};
  }
  const auto segments = std::filesystem::weakly_canonical(canonical / "segments", ec);
  const auto segments_relative = segments.lexically_relative(canonical);
  if (ec || segments_relative.empty() || segments_relative == "." || segments_relative.is_absolute() ||
    (!segments_relative.empty() && *segments_relative.begin() == ".."))
  {
    return {false, false, "segments directory must remain beneath the canonical session directory"};
  }
  bound_ = true;
  session_id_ = session_id;
  history_.emplace(request_id, Payload{session_id, session_directory});
  canonical_directory_ = segments;
  return {true, false, "existing session directory bound without creating or overwriting it"};
}
}  // namespace ppbng_gnss
