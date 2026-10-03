#pragma once

#include <filesystem>
#include <string>
#include <unordered_map>
#include <utility>

namespace ppbng_rsm400
{
struct SessionBindingResult {bool accepted{}; bool duplicate{}; std::string detail;};
class SessionBinding
{
public:
  void set_allowed_output_root(std::filesystem::path root) {root_ = std::move(root);}
  SessionBindingResult prepare(const std::string & request_id, const std::string & session_id,
    const std::string & session_directory, bool started, bool armed);
  bool bound() const noexcept {return bound_;}
  const std::filesystem::path & segments_directory() const noexcept {return segments_;}
  const std::string & session_id() const noexcept {return session_id_;}
private:
  struct Payload {std::string session_id; std::string directory;};
  std::filesystem::path root_, segments_;
  std::string session_id_;
  std::unordered_map<std::string, Payload> history_;
  bool bound_{};
};
}  // namespace ppbng_rsm400
