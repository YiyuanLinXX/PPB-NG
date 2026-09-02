#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <string>
#include <string_view>

namespace ppbng_storage
{

enum class JsonRootType {object, array, string, number, boolean, null_value};

struct JsonValidationResult
{
  bool success{false};
  JsonRootType root_type{JsonRootType::null_value};
  std::size_t error_offset{0};
  std::string message;
};

struct JsonFileVerificationResult
{
  bool success{false};
  std::uint64_t record_count{0};
  std::uint64_t byte_count{0};
  std::uint64_t error_line{0};
  std::size_t error_column{0};
  std::string message;
};

JsonValidationResult validate_json(std::string_view text) noexcept;

enum class JsonScalarType {string, number, boolean, null_value};
struct JsonScalar {JsonScalarType type{JsonScalarType::null_value}; std::string text;};
struct FlatJsonObjectResult
{
  bool success{false};
  std::map<std::string, JsonScalar> members;
  std::string message;
};

// Parses an already self-contained flat JSON object into scalar members. Nested
// objects/arrays and duplicate keys are rejected. Strings are JSON-unescaped.
FlatJsonObjectResult parse_flat_json_object(std::string_view text) noexcept;

// A manifest is bounded because it is read as one document. The function opens
// the path read-only and requires a JSON object.
JsonFileVerificationResult verify_json_object_file(
  const std::filesystem::path & path,
  std::uint64_t maximum_bytes = 16U * 1024U * 1024U) noexcept;

// JSONL/NDJSON is validated incrementally. Each non-empty line must contain one
// JSON object and every record must end in a newline. Memory is bounded by
// maximum_line_bytes rather than total file size.
JsonFileVerificationResult verify_json_lines_file(
  const std::filesystem::path & path,
  std::size_t maximum_line_bytes = 1024U * 1024U) noexcept;

}  // namespace ppbng_storage
