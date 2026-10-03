#include "ppbng_storage/json_verifier.hpp"

#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

namespace
{

class TempDirectory
{
public:
  TempDirectory()
  {
    path = std::filesystem::temp_directory_path() /
      ("ppbng_json_verify_" + std::to_string(++sequence));
    std::filesystem::create_directory(path);
  }
  ~TempDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }
  std::filesystem::path path;
  static inline std::uint64_t sequence{0U};
};

void write_binary(const std::filesystem::path & path, const std::string & value)
{
  std::ofstream output(path, std::ios::binary);
  output.write(value.data(), static_cast<std::streamsize>(value.size()));
}

}  // namespace

TEST(JsonVerifier, AcceptsManifestShapeEscapesUtf8AndNumbers)
{
  const auto result = ppbng_storage::validate_json(
    " {\"schema_version\":3,\"name\":\"\xE4\xB8\xAD\xE6\x96\x87\\n\","
    "\"values\":[null,true,false,-12.5e+2]} \r\n");
  EXPECT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.root_type, ppbng_storage::JsonRootType::object);
}

TEST(JsonVerifier, RejectsMalformedNumbersStringsUtf8AndTrailingData)
{
  EXPECT_FALSE(ppbng_storage::validate_json("{\"x\":01}").success);
  EXPECT_FALSE(ppbng_storage::validate_json("{\"x\":\"bad\\q\"}").success);
  EXPECT_FALSE(ppbng_storage::validate_json(std::string("{\"x\":\"") +
    static_cast<char>(0xc0) + static_cast<char>(0x80) + "\"}").success);
  EXPECT_FALSE(ppbng_storage::validate_json("{} garbage").success);
  EXPECT_FALSE(ppbng_storage::validate_json("[1,]").success);
}

TEST(JsonVerifier, EnforcesNestingLimit)
{
  std::string nested(130U, '[');
  nested += "0";
  nested.append(130U, ']');
  const auto result = ppbng_storage::validate_json(nested);
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("depth"), std::string::npos);
}

TEST(JsonVerifier, VerifiesBoundedObjectFile)
{
  TempDirectory temp;
  const auto path = temp.path / "manifest.json";
  write_binary(path, "{\n  \"schema_version\": 3\n}\n");
  const auto valid = ppbng_storage::verify_json_object_file(path);
  EXPECT_TRUE(valid.success) << valid.message;
  EXPECT_EQ(valid.record_count, 1U);

  const auto too_small = ppbng_storage::verify_json_object_file(path, 4U);
  EXPECT_FALSE(too_small.success);
  EXPECT_NE(too_small.message.find("size limit"), std::string::npos);

  write_binary(path, "[]");
  EXPECT_FALSE(ppbng_storage::verify_json_object_file(path).success);
}

TEST(JsonVerifier, StreamsJsonLinesAndRequiresObjectPerTerminatedLine)
{
  TempDirectory temp;
  const auto path = temp.path / "events.jsonl";
  write_binary(path, "{\"id\":1}\n{\"id\":2,\"ok\":true}\r\n");
  const auto valid = ppbng_storage::verify_json_lines_file(path);
  EXPECT_TRUE(valid.success) << valid.message;
  EXPECT_EQ(valid.record_count, 2U);

  write_binary(path, "{\"id\":1}\n{\"id\":2}");
  const auto unterminated = ppbng_storage::verify_json_lines_file(path);
  EXPECT_FALSE(unterminated.success);
  EXPECT_EQ(unterminated.error_line, 2U);

  write_binary(path, "{\"id\":1}\n[]\n");
  const auto wrong_root = ppbng_storage::verify_json_lines_file(path);
  EXPECT_FALSE(wrong_root.success);
  EXPECT_EQ(wrong_root.error_line, 2U);
}

TEST(JsonVerifier, RejectsEmptyMalformedAndOversizedJsonLines)
{
  TempDirectory temp;
  const auto path = temp.path / "events.ndjson";
  write_binary(path, "{}\n\n");
  EXPECT_FALSE(ppbng_storage::verify_json_lines_file(path).success);

  write_binary(path, "{\"broken\":}\n");
  const auto malformed = ppbng_storage::verify_json_lines_file(path);
  EXPECT_FALSE(malformed.success);
  EXPECT_EQ(malformed.error_line, 1U);
  EXPECT_GT(malformed.error_column, 0U);

  write_binary(path, "{\"long\":12345}\n");
  EXPECT_FALSE(ppbng_storage::verify_json_lines_file(path, 8U).success);
}

TEST(JsonVerifier, ParsesFlatScalarsAndRejectsDuplicateOrNestedMembers)
{
  const auto parsed = ppbng_storage::parse_flat_json_object(
    "{\"state\":\"MATCHED\",\"sample\":7,\"valid\":true,\"optional\":null}");
  ASSERT_TRUE(parsed.success) << parsed.message;
  EXPECT_EQ(parsed.members.at("state").text, "MATCHED");
  EXPECT_EQ(parsed.members.at("sample").type, ppbng_storage::JsonScalarType::number);
  EXPECT_EQ(parsed.members.at("valid").type, ppbng_storage::JsonScalarType::boolean);
  EXPECT_EQ(parsed.members.at("optional").type, ppbng_storage::JsonScalarType::null_value);

  EXPECT_FALSE(ppbng_storage::parse_flat_json_object("{\"x\":1,\"x\":2}").success);
  EXPECT_FALSE(ppbng_storage::parse_flat_json_object("{\"nested\":{}}").success);
}
