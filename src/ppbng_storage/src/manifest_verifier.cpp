#include "ppbng_storage/manifest_verifier.hpp"

#include "ppbng_storage/json_verifier.hpp"

#include <charconv>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <string_view>

namespace ppbng_storage
{
namespace
{

constexpr std::uint64_t kMaximumManifestBytes = 16U * 1024U * 1024U;

struct Span {std::size_t begin{0U}; std::size_t end{0U};};

class Inspector
{
public:
  explicit Inspector(const std::string_view text) : text_(text) {}

  bool object(const Span span, std::map<std::string, Span> & members, std::string & error) const
  {
    std::size_t position = span.begin;
    skip(position);
    if (position >= span.end || text_[position++] != '{') {error = "expected object"; return false;}
    skip(position);
    if (position < span.end && text_[position] == '}') {return true;}
    for (;;) {
      std::string key;
      if (!decoded_string(position, key, error)) {return false;}
      skip(position);
      if (position >= span.end || text_[position++] != ':') {error = "expected colon"; return false;}
      skip(position);
      Span value;
      if (!value_span(position, value, error)) {return false;}
      if (!members.emplace(std::move(key), value).second) {
        error = "duplicate object key";
        return false;
      }
      skip(position);
      if (position < span.end && text_[position] == '}') {return true;}
      if (position >= span.end || text_[position++] != ',') {error = "expected comma"; return false;}
      skip(position);
    }
  }

  bool array(const Span span, std::vector<Span> & values, std::string & error) const
  {
    std::size_t position = span.begin;
    skip(position);
    if (position >= span.end || text_[position++] != '[') {error = "expected array"; return false;}
    skip(position);
    if (position < span.end && text_[position] == ']') {return true;}
    for (;;) {
      Span value;
      if (!value_span(position, value, error)) {return false;}
      values.push_back(value);
      skip(position);
      if (position < span.end && text_[position] == ']') {return true;}
      if (position >= span.end || text_[position++] != ',') {error = "expected comma"; return false;}
      skip(position);
    }
  }

  bool string_value(const Span span, std::string & value, std::string & error) const
  {
    std::size_t position = span.begin;
    skip(position);
    return decoded_string(position, value, error);
  }

  bool bool_value(const Span span, bool & value) const
  {
    const auto token = trimmed(span);
    if (token == "true") {value = true; return true;}
    if (token == "false") {value = false; return true;}
    return false;
  }

  bool uint_value(const Span span, std::uint64_t & value) const
  {
    const auto token = trimmed(span);
    if (token.empty()) {return false;}
    const auto conversion = std::from_chars(token.data(), token.data() + token.size(), value);
    return conversion.ec == std::errc{} && conversion.ptr == token.data() + token.size();
  }

  Span document() const {return {0U, text_.size()};}

private:
  void skip(std::size_t & position) const
  {
    while (position < text_.size() &&
      (text_[position] == ' ' || text_[position] == '\t' ||
      text_[position] == '\r' || text_[position] == '\n')) {++position;}
  }

  std::string_view trimmed(const Span span) const
  {
    std::size_t begin = span.begin;
    std::size_t end = span.end;
    skip(begin);
    while (end > begin && (text_[end - 1U] == ' ' || text_[end - 1U] == '\t' ||
      text_[end - 1U] == '\r' || text_[end - 1U] == '\n')) {--end;}
    return text_.substr(begin, end - begin);
  }

  bool decoded_string(std::size_t & position, std::string & output, std::string & error) const
  {
    if (position >= text_.size() || text_[position++] != '"') {
      error = "expected string";
      return false;
    }
    while (position < text_.size()) {
      const char value = text_[position++];
      if (value == '"') {return true;}
      if (value != '\\') {output.push_back(value); continue;}
      if (position >= text_.size()) {error = "truncated escape"; return false;}
      const char escaped = text_[position++];
      switch (escaped) {
        case '"': output.push_back('"'); break;
        case '\\': output.push_back('\\'); break;
        case '/': output.push_back('/'); break;
        case 'b': output.push_back('\b'); break;
        case 'f': output.push_back('\f'); break;
        case 'n': output.push_back('\n'); break;
        case 'r': output.push_back('\r'); break;
        case 't': output.push_back('\t'); break;
        case 'u':
          // Semantic field names and role values are ASCII in schema 3. Preserve
          // a marker so escaped content cannot accidentally equal a required key.
          if (position + 4U > text_.size()) {error = "truncated Unicode escape"; return false;}
          output.append("\\u");
          output.append(text_.substr(position, 4U));
          position += 4U;
          break;
        default: error = "invalid escape"; return false;
      }
    }
    error = "unterminated string";
    return false;
  }

  bool value_span(std::size_t & position, Span & span, std::string & error) const
  {
    skip(position);
    if (position >= text_.size()) {error = "missing value"; return false;}
    span.begin = position;
    if (text_[position] == '"') {
      std::string ignored;
      if (!decoded_string(position, ignored, error)) {return false;}
      span.end = position;
      return true;
    }
    if (text_[position] == '{' || text_[position] == '[') {
      const char opening = text_[position];
      const char closing = opening == '{' ? '}' : ']';
      std::size_t depth = 0U;
      bool in_string = false;
      bool escaped = false;
      while (position < text_.size()) {
        const char value = text_[position++];
        if (in_string) {
          if (escaped) {escaped = false;}
          else if (value == '\\') {escaped = true;}
          else if (value == '"') {in_string = false;}
          continue;
        }
        if (value == '"') {in_string = true; continue;}
        if (value == opening) {++depth;}
        else if (value == closing && --depth == 0U) {span.end = position; return true;}
      }
      error = "unterminated composite value";
      return false;
    }
    while (position < text_.size() && text_[position] != ',' &&
      text_[position] != '}' && text_[position] != ']') {++position;}
    span.end = position;
    return true;
  }

  std::string_view text_;
};

bool member(
  const std::map<std::string, Span> & members, const std::string & key,
  Span & value, std::string & error)
{
  const auto found = members.find(key);
  if (found == members.end()) {error = "missing manifest field: " + key; return false;}
  value = found->second;
  return true;
}

ManifestVerificationResult failure(const std::string & message)
{
  return {false, {}, false, false, 0U, {}, message};
}

}  // namespace

ManifestVerificationResult verify_session_manifest_file(
  const std::filesystem::path & path,
  const std::vector<std::string> & expected_roles) noexcept
{
  try {
    const auto syntax = verify_json_object_file(path, kMaximumManifestBytes);
    if (!syntax.success) {return failure("manifest JSON invalid: " + syntax.message);}
    std::error_code io_error;
    const auto size = std::filesystem::file_size(path, io_error);
    if (io_error || size > std::numeric_limits<std::size_t>::max()) {
      return failure("cannot size manifest");
    }
    std::ifstream input(path, std::ios::binary);
    std::string text(static_cast<std::size_t>(size), '\0');
    input.read(text.data(), static_cast<std::streamsize>(text.size()));
    if (input.gcount() != static_cast<std::streamsize>(text.size())) {
      return failure("short read while inspecting manifest");
    }
    Inspector inspector(text);
    std::map<std::string, Span> root;
    std::string error;
    if (!inspector.object(inspector.document(), root, error)) {return failure(error);}

    Span field;
    std::uint64_t schema = 0U;
    if (!member(root, "schema_version", field, error) ||
      !inspector.uint_value(field, schema) || schema != 3U)
    {
      return failure(error.empty() ? "schema_version must equal 3" : error);
    }
    std::string session_id, dataset_name, created_utc, finalized_utc, state;
    for (auto item : {
        std::pair<const char *, std::string *>("session_id", &session_id),
        {"user_dataset_name", &dataset_name}, {"created_utc", &created_utc},
        {"finalized_utc", &finalized_utc}, {"state", &state}})
    {
      if (!member(root, item.first, field, error) ||
        !inspector.string_value(field, *item.second, error)) {return failure(error);}
    }
    if (session_id.empty() || dataset_name.empty() || created_utc.empty()) {
      return failure("session_id, user_dataset_name, and created_utc must be non-empty");
    }
    if (state != "finalized" && state != "faulted") {
      return failure("offline verification requires a finalized or faulted manifest");
    }
    if (finalized_utc.empty()) {return failure("terminal manifest requires finalized_utc");}

    bool simulation = false;
    bool hardware = false;
    if (!member(root, "simulation", field, error) ||
      !inspector.bool_value(field, simulation)) {return failure("simulation must be boolean");}
    if (!member(root, "hardware_enabled", field, error) ||
      !inspector.bool_value(field, hardware)) {return failure("hardware_enabled must be boolean");}
    if (simulation == hardware) {
      return failure("exactly one of simulation and hardware_enabled must be true");
    }
    if (hardware) {
      std::string machine_id, snapshot;
      if (!member(root, "machine_id", field, error) ||
        !inspector.string_value(field, machine_id, error) || machine_id.empty())
      {return failure("hardware manifest requires machine_id");}
      if (!member(root, "configuration_snapshot", field, error) ||
        !inspector.string_value(field, snapshot, error) || snapshot.empty())
      {return failure("hardware manifest requires configuration_snapshot");}
    }

    if (!member(root, "devices", field, error)) {return failure(error);}
    std::vector<Span> devices;
    if (!inspector.array(field, devices, error)) {return failure("devices must be an array");}
    std::set<std::string> roles;
    for (const auto device_span : devices) {
      std::map<std::string, Span> device;
      if (!inspector.object(device_span, device, error)) {return failure("device must be an object");}
      Span role_span;
      std::string role;
      if (!member(device, "role", role_span, error) ||
        !inspector.string_value(role_span, role, error) || role.empty())
      {return failure("device role must be a non-empty string");}
      if (!roles.insert(role).second) {return failure("duplicate device role: " + role);}
    }
    for (const auto & role : expected_roles) {
      if (roles.count(role) == 0U) {return failure("missing expected device role: " + role);}
    }

    if (!member(root, "storage", field, error)) {return failure(error);}
    std::map<std::string, Span> storage;
    if (!inspector.object(field, storage, error)) {return failure("storage must be an object");}
    bool final_available_valid = false;
    bool actual_bytes_valid = false;
    Span storage_field;
    if (!member(storage, "final_available_valid", storage_field, error) ||
      !inspector.bool_value(storage_field, final_available_valid) || !final_available_valid)
    {return failure("terminal manifest requires final storage-capacity evidence");}
    if (!member(storage, "actual_dataset_bytes_valid", storage_field, error) ||
      !inspector.bool_value(storage_field, actual_bytes_valid) || !actual_bytes_valid)
    {return failure("terminal manifest requires actual dataset byte evidence");}

    return {true, state, simulation, hardware,
      static_cast<std::uint64_t>(devices.size()), session_id,
      "schema-3 manifest semantics verified"};
  } catch (const std::exception & error) {return failure(error.what());}
  catch (...) {return failure("unknown manifest verification failure");}
}

}  // namespace ppbng_storage
