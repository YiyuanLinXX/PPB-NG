#pragma once

#include <filesystem>
#include <string>
#include <unordered_map>
#include <utility>

namespace ppbng_gnss
{
struct SessionBindingResult {bool accepted{false}; bool duplicate{false}; std::string detail;};
class SessionBinding
{
public:
  void set_allowed_output_root(std::filesystem::path root) {allowed_output_root_ = std::move(root);}
  SessionBindingResult prepare(const std::string & request_id, const std::string & session_id,
    const std::string & session_directory, bool started, bool may_rebind);
  bool bound() const noexcept {return bound_;}
  const std::filesystem::path & directory() const noexcept {return canonical_directory_;}
  const std::string & session_id() const noexcept {return session_id_;}
private:
  struct Payload {std::string session_id; std::string directory;};
  bool bound_{false};
  std::string session_id_;
  std::filesystem::path allowed_output_root_;
  std::unordered_map<std::string, Payload> history_;
  std::filesystem::path canonical_directory_;
};
}  // namespace ppbng_gnss
