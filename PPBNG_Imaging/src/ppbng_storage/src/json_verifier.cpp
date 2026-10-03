#include "ppbng_storage/json_verifier.hpp"

#include <cctype>
#include <fstream>
#include <iterator>
#include <limits>

namespace ppbng_storage
{
namespace
{

constexpr std::size_t kMaximumDepth = 128U;

class Parser
{
public:
  explicit Parser(const std::string_view text) : text_(text) {}

  JsonValidationResult parse() noexcept
  {
    skip_space();
    JsonRootType type{};
    if (!value(0U, type)) {return failure();}
    skip_space();
    if (position_ != text_.size()) {
      fail("trailing characters after JSON value");
      return failure();
    }
    return {true, type, 0U, "valid JSON"};
  }

private:
  JsonValidationResult failure() const
  {
    return {false, JsonRootType::null_value, error_offset_, error_};
  }

  void fail(const char * message)
  {
    if (error_.empty()) {
      error_offset_ = position_;
      error_ = message;
    }
  }

  void skip_space()
  {
    while (position_ < text_.size() &&
      (text_[position_] == ' ' || text_[position_] == '\t' ||
      text_[position_] == '\r' || text_[position_] == '\n'))
    {
      ++position_;
    }
  }

  bool consume(const char value)
  {
    if (position_ < text_.size() && text_[position_] == value) {
      ++position_;
      return true;
    }
    return false;
  }

  bool literal(const std::string_view expected)
  {
    if (text_.substr(position_, expected.size()) != expected) {
      fail("invalid JSON literal");
      return false;
    }
    position_ += expected.size();
    return true;
  }

  bool value(const std::size_t depth, JsonRootType & type)
  {
    if (depth > kMaximumDepth) {fail("JSON nesting depth exceeds limit"); return false;}
    if (position_ >= text_.size()) {fail("expected JSON value"); return false;}
    switch (text_[position_]) {
      case '{': type = JsonRootType::object; return object(depth + 1U);
      case '[': type = JsonRootType::array; return array(depth + 1U);
      case '"': type = JsonRootType::string; return string();
      case 't': type = JsonRootType::boolean; return literal("true");
      case 'f': type = JsonRootType::boolean; return literal("false");
      case 'n': type = JsonRootType::null_value; return literal("null");
      default:
        if (text_[position_] == '-' ||
          (text_[position_] >= '0' && text_[position_] <= '9'))
        {
          type = JsonRootType::number;
          return number();
        }
        fail("unexpected character while parsing JSON value");
        return false;
    }
  }

  bool object(const std::size_t depth)
  {
    ++position_;
    skip_space();
    if (consume('}')) {return true;}
    for (;;) {
      if (!string()) {return false;}
      skip_space();
      if (!consume(':')) {fail("expected colon after object key"); return false;}
      skip_space();
      JsonRootType ignored{};
      if (!value(depth, ignored)) {return false;}
      skip_space();
      if (consume('}')) {return true;}
      if (!consume(',')) {fail("expected comma or closing brace"); return false;}
      skip_space();
    }
  }

  bool array(const std::size_t depth)
  {
    ++position_;
    skip_space();
    if (consume(']')) {return true;}
    for (;;) {
      JsonRootType ignored{};
      if (!value(depth, ignored)) {return false;}
      skip_space();
      if (consume(']')) {return true;}
      if (!consume(',')) {fail("expected comma or closing bracket"); return false;}
      skip_space();
    }
  }

  bool unicode_escape()
  {
    for (int digit = 0; digit < 4; ++digit) {
      if (position_ >= text_.size() || !std::isxdigit(
          static_cast<unsigned char>(text_[position_])))
      {
        fail("invalid Unicode escape");
        return false;
      }
      ++position_;
    }
    return true;
  }

  bool utf8_sequence()
  {
    const auto lead = static_cast<unsigned char>(text_[position_]);
    std::size_t count = 0U;
    unsigned char second_min = 0x80U;
    unsigned char second_max = 0xbfU;
    if (lead >= 0xc2U && lead <= 0xdfU) {
      count = 2U;
    } else if (lead >= 0xe0U && lead <= 0xefU) {
      count = 3U;
      if (lead == 0xe0U) {second_min = 0xa0U;}
      if (lead == 0xedU) {second_max = 0x9fU;}
    } else if (lead >= 0xf0U && lead <= 0xf4U) {
      count = 4U;
      if (lead == 0xf0U) {second_min = 0x90U;}
      if (lead == 0xf4U) {second_max = 0x8fU;}
    } else {
      fail("invalid UTF-8 lead byte in JSON string");
      return false;
    }
    if (position_ + count > text_.size()) {
      fail("truncated UTF-8 sequence in JSON string");
      return false;
    }
    const auto second = static_cast<unsigned char>(text_[position_ + 1U]);
    if (second < second_min || second > second_max) {
      fail("invalid UTF-8 sequence in JSON string");
      return false;
    }
    for (std::size_t index = 2U; index < count; ++index) {
      const auto byte = static_cast<unsigned char>(text_[position_ + index]);
      if (byte < 0x80U || byte > 0xbfU) {
        fail("invalid UTF-8 continuation byte in JSON string");
        return false;
      }
    }
    position_ += count;
    return true;
  }

  bool string()
  {
    if (!consume('"')) {fail("expected JSON string"); return false;}
    while (position_ < text_.size()) {
      const auto value = static_cast<unsigned char>(text_[position_]);
      if (value == '"') {++position_; return true;}
      if (value < 0x20U) {fail("unescaped control byte in JSON string"); return false;}
      if (value == '\\') {
        ++position_;
        if (position_ >= text_.size()) {fail("truncated JSON escape"); return false;}
        const auto escaped = text_[position_++];
        if (escaped == 'u') {
          if (!unicode_escape()) {return false;}
        } else if (escaped != '"' && escaped != '\\' && escaped != '/' &&
          escaped != 'b' && escaped != 'f' && escaped != 'n' &&
          escaped != 'r' && escaped != 't')
        {
          fail("invalid JSON escape");
          return false;
        }
      } else if (value >= 0x80U) {
        if (!utf8_sequence()) {return false;}
      } else {
        ++position_;
      }
    }
    fail("unterminated JSON string");
    return false;
  }

  bool number()
  {
    consume('-');
    if (position_ >= text_.size()) {fail("truncated JSON number"); return false;}
    if (consume('0')) {
      if (position_ < text_.size() && text_[position_] >= '0' && text_[position_] <= '9') {
        fail("leading zero in JSON number");
        return false;
      }
    } else {
      if (text_[position_] < '1' || text_[position_] > '9') {
        fail("invalid JSON integer");
        return false;
      }
      while (position_ < text_.size() && text_[position_] >= '0' && text_[position_] <= '9') {
        ++position_;
      }
    }
    if (consume('.')) {
      if (position_ >= text_.size() || text_[position_] < '0' || text_[position_] > '9') {
        fail("fraction requires a digit");
        return false;
      }
      while (position_ < text_.size() && text_[position_] >= '0' && text_[position_] <= '9') {
        ++position_;
      }
    }
    if (position_ < text_.size() && (text_[position_] == 'e' || text_[position_] == 'E')) {
      ++position_;
      if (position_ < text_.size() && (text_[position_] == '+' || text_[position_] == '-')) {
        ++position_;
      }
      if (position_ >= text_.size() || text_[position_] < '0' || text_[position_] > '9') {
        fail("exponent requires a digit");
        return false;
      }
      while (position_ < text_.size() && text_[position_] >= '0' && text_[position_] <= '9') {
        ++position_;
      }
    }
    return true;
  }

  std::string_view text_;
  std::size_t position_{0U};
  std::size_t error_offset_{0U};
  std::string error_;
};

JsonFileVerificationResult file_failure(const std::string & message)
{
  return {false, 0U, 0U, 0U, 0U, message};
}

}  // namespace

JsonValidationResult validate_json(const std::string_view text) noexcept
{
  try {return Parser(text).parse();}
  catch (const std::exception & error) {
    return {false, JsonRootType::null_value, 0U, error.what()};
  } catch (...) {
    return {false, JsonRootType::null_value, 0U, "unknown JSON validation failure"};
  }
}

FlatJsonObjectResult parse_flat_json_object(const std::string_view text) noexcept
{
  try {
    const auto syntax = validate_json(text);
    if (!syntax.success || syntax.root_type != JsonRootType::object) {
      return {false, {}, syntax.success ? "JSON root is not an object" : syntax.message};
    }
    std::size_t position = 0U;
    const auto skip = [&]() {
        while (position < text.size() &&
          (text[position] == ' ' || text[position] == '\t' ||
          text[position] == '\r' || text[position] == '\n')) {++position;}
      };
    const auto decode_string = [&](std::string & output) {
        if (position >= text.size() || text[position++] != '"') {return false;}
        while (position < text.size()) {
          const char value = text[position++];
          if (value == '"') {return true;}
          if (value != '\\') {output.push_back(value); continue;}
          if (position >= text.size()) {return false;}
          const char escaped = text[position++];
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
              output.append("\\u");
              output.append(text.substr(position, 4U));
              position += 4U;
              break;
            default: return false;
          }
        }
        return false;
      };
    skip();
    ++position;  // opening brace, guaranteed by validate_json
    skip();
    FlatJsonObjectResult result;
    if (position < text.size() && text[position] == '}') {
      result.success = true;
      result.message = "flat JSON object parsed";
      return result;
    }
    for (;;) {
      std::string key;
      if (!decode_string(key)) {return {false, {}, "cannot decode object key"};}
      skip();
      ++position;  // colon, guaranteed by validate_json
      skip();
      JsonScalar scalar;
      if (text[position] == '{' || text[position] == '[') {
        return {false, {}, "nested JSON value is not allowed in a flat record"};
      }
      if (text[position] == '"') {
        scalar.type = JsonScalarType::string;
        if (!decode_string(scalar.text)) {return {false, {}, "cannot decode string value"};}
      } else {
        const auto begin = position;
        while (position < text.size() && text[position] != ',' && text[position] != '}') {
          ++position;
        }
        auto end = position;
        while (end > begin && (text[end - 1U] == ' ' || text[end - 1U] == '\t' ||
          text[end - 1U] == '\r' || text[end - 1U] == '\n')) {--end;}
        scalar.text = std::string(text.substr(begin, end - begin));
        if (scalar.text == "true" || scalar.text == "false") {
          scalar.type = JsonScalarType::boolean;
        } else if (scalar.text == "null") {
          scalar.type = JsonScalarType::null_value;
        } else {
          scalar.type = JsonScalarType::number;
        }
      }
      if (!result.members.emplace(std::move(key), std::move(scalar)).second) {
        return {false, {}, "duplicate key in flat JSON object"};
      }
      skip();
      if (position < text.size() && text[position] == '}') {
        result.success = true;
        result.message = "flat JSON object parsed";
        return result;
      }
      ++position;  // comma, guaranteed by validate_json
      skip();
    }
  } catch (const std::exception & error) {return {false, {}, error.what()};}
  catch (...) {return {false, {}, "unknown flat JSON parse failure"};}
}

JsonFileVerificationResult verify_json_object_file(
  const std::filesystem::path & path, const std::uint64_t maximum_bytes) noexcept
{
  try {
    std::error_code error;
    const auto size = std::filesystem::file_size(path, error);
    if (error) {return file_failure("cannot determine JSON file size: " + error.message());}
    if (size > maximum_bytes || size > std::numeric_limits<std::size_t>::max()) {
      return file_failure("JSON document exceeds configured size limit");
    }
    std::ifstream input(path, std::ios::binary);
    if (!input) {return file_failure("cannot open JSON file");}
    std::string text(static_cast<std::size_t>(size), '\0');
    input.read(text.data(), static_cast<std::streamsize>(text.size()));
    if (input.gcount() != static_cast<std::streamsize>(text.size())) {
      return file_failure("short read while loading JSON file");
    }
    const auto validation = validate_json(text);
    if (!validation.success) {
      return {false, 0U, size, 1U, validation.error_offset + 1U, validation.message};
    }
    if (validation.root_type != JsonRootType::object) {
      return {false, 0U, size, 1U, 1U, "JSON document root must be an object"};
    }
    return {true, 1U, size, 0U, 0U, "JSON object verified"};
  } catch (const std::exception & error) {return file_failure(error.what());}
  catch (...) {return file_failure("unknown JSON file verification failure");}
}

JsonFileVerificationResult verify_json_lines_file(
  const std::filesystem::path & path, const std::size_t maximum_line_bytes) noexcept
{
  try {
    std::ifstream input(path, std::ios::binary);
    if (!input) {return file_failure("cannot open JSON-lines file");}
    JsonFileVerificationResult result;
    std::string line;
    std::uint64_t line_number = 0U;
    while (std::getline(input, line)) {
      ++line_number;
      if (input.eof()) {
        return {false, line_number - 1U, result.byte_count, line_number, 1U,
          "final JSON-lines record is not newline-terminated"};
      }
      result.byte_count += static_cast<std::uint64_t>(line.size()) + 1U;
      if (!line.empty() && line.back() == '\r') {line.pop_back();}
      if (line.empty()) {
        return {false, line_number - 1U, result.byte_count, line_number, 1U,
          "empty JSON-lines record"};
      }
      if (line.size() > maximum_line_bytes) {
        return {false, line_number - 1U, result.byte_count, line_number, 1U,
          "JSON-lines record exceeds configured size limit"};
      }
      const auto validation = validate_json(line);
      if (!validation.success) {
        return {false, line_number - 1U, result.byte_count, line_number,
          validation.error_offset + 1U, validation.message};
      }
      if (validation.root_type != JsonRootType::object) {
        return {false, line_number - 1U, result.byte_count, line_number, 1U,
          "JSON-lines record root must be an object"};
      }
      const auto flat = parse_flat_json_object(line);
      if (!flat.success) {
        return {false, line_number - 1U, result.byte_count, line_number, 1U, flat.message};
      }
      ++result.record_count;
    }
    if (!input.eof()) {return file_failure("read failure in JSON-lines file");}
    result.success = true;
    result.message = "JSON-lines records verified";
    return result;
  } catch (const std::exception & error) {return file_failure(error.what());}
  catch (...) {return file_failure("unknown JSON-lines verification failure");}
}

}  // namespace ppbng_storage
