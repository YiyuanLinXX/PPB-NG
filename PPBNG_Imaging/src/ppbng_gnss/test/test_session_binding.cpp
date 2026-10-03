#include "ppbng_gnss/session_binding.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>

TEST(GnssSessionBinding, RejectsArbitraryPathsAndKeepsAllRequestIdsIdempotent)
{
  const auto root = std::filesystem::temp_directory_path() /
    ("ppbng-gnss-binding-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  const auto first = root / "one";
  const auto second = root / "two";
  std::filesystem::create_directories(first / "segments");
  std::filesystem::create_directories(second / "segments");
  ppbng_gnss::SessionBinding binding;
  binding.set_allowed_output_root(root);
  ASSERT_TRUE(binding.prepare("a", "session-a", first.string(), false, true).accepted);
  ASSERT_TRUE(binding.prepare("b", "session-b", second.string(), false, true).accepted);
  const auto old_replay = binding.prepare("a", "session-a", first.string(), false, true);
  EXPECT_TRUE(old_replay.accepted); EXPECT_TRUE(old_replay.duplicate);
  EXPECT_FALSE(binding.prepare("a", "other", first.string(), false, true).accepted);
  EXPECT_FALSE(binding.prepare("c", "root", root.string(), false, true).accepted);
  EXPECT_FALSE(binding.prepare("d", "outside", std::filesystem::temp_directory_path().string(), false, true).accepted);
  EXPECT_FALSE(binding.prepare("e", "started", second.string(), true, false).accepted);
  std::error_code ec; std::filesystem::remove_all(root, ec);
}
