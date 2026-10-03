#include "ppbng_rsm400/session_binding.hpp"
#include "ppbng_rsm400/telemetry_log_writer.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>

namespace
{
class TempTree
{
public:
  TempTree()
  {
    root = std::filesystem::temp_directory_path() /
      ("ppbng-rsm-storage-" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    session = root / "session";
    std::filesystem::create_directories(session / "segments");
  }
  ~TempTree() {std::error_code ec; std::filesystem::remove_all(root, ec);}
  std::filesystem::path root, session;
};
}

TEST(RsmSessionBinding, IsStrictIdempotentAndRejectsRoot)
{
  TempTree tree;
  ppbng_rsm400::SessionBinding binding;
  binding.set_allowed_output_root(tree.root);
  ASSERT_TRUE(binding.prepare("r1", "s1", tree.session.string(), false, false).accepted);
  const auto duplicate = binding.prepare("r1", "s1", tree.session.string(), false, false);
  EXPECT_TRUE(duplicate.accepted);
  EXPECT_TRUE(duplicate.duplicate);
  EXPECT_FALSE(binding.prepare("r1", "changed", tree.session.string(), false, false).accepted);
  EXPECT_FALSE(binding.prepare("r2", "s2", tree.root.string(), false, false).accepted);
  EXPECT_FALSE(binding.prepare("r3", "s3", tree.session.string(), true, false).accepted);
}

TEST(RsmTelemetryLogWriter, ExclusivelyPersistsRawAndDecodedTelemetry)
{
  TempTree tree;
  ppbng_rsm400::TelemetryLogOptions options;
  options.segments_directory = tree.session / "segments";
  options.flush_every_records = 1U;
  auto [created, writer] = ppbng_rsm400::TelemetryLogWriter::create(options);
  ASSERT_TRUE(created.success) << created.detail;
  ppbng_rsm400::Frame frame;
  frame.raw = "VM1,2*00\r\n";
  ppbng_rsm400::Telemetry telemetry;
  telemetry.roll_deg = 1.25;
  telemetry.pitch_deg = -2.5;
  ASSERT_TRUE(writer->append(frame, telemetry, 100U, 200U, 3U).success);
  ASSERT_TRUE(writer->close().success);
  std::ifstream input(options.segments_directory / "rsm400.telemetry.jsonl");
  const std::string text((std::istreambuf_iterator<char>(input)), {});
  EXPECT_NE(text.find("\"segment_id\":3"), std::string::npos);
  EXPECT_NE(text.find("\"roll_deg\":1.25"), std::string::npos);
  EXPECT_NE(text.find("VM1,2*00\\r\\n"), std::string::npos);
  auto [duplicate, second] = ppbng_rsm400::TelemetryLogWriter::create(options);
  EXPECT_FALSE(duplicate.success);
  EXPECT_FALSE(second);
}

TEST(RsmTelemetryLogWriter, ReportsInjectedFailureBeforeSecondRecord)
{
  TempTree tree;
  ppbng_rsm400::TelemetryLogOptions options;
  options.segments_directory = tree.session / "segments";
  options.flush_every_records = 10U;
  options.fail_after_records = 1U;
  auto [created, writer] = ppbng_rsm400::TelemetryLogWriter::create(options);
  ASSERT_TRUE(created.success);
  ppbng_rsm400::Frame frame; frame.raw = "raw";
  ppbng_rsm400::Telemetry telemetry;
  EXPECT_TRUE(writer->append(frame, telemetry, 1U, 2U, 1U).success);
  EXPECT_FALSE(writer->append(frame, telemetry, 3U, 4U, 1U).success);
}
