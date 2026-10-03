#pragma once

#include "ppbng_gnss/um982_receiver.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>

namespace ppbng_gnss
{

struct GnssLogWriterOptions
{
  std::filesystem::path session_directory;
  std::string stream_stem{"um982"};
  std::uint64_t flush_every_sentences{10U};
  std::uint64_t fail_after_sentences{(std::numeric_limits<std::uint64_t>::max)()};
};

struct GnssLogResult
{
  bool success{false};
  std::string detail;
};

class GnssLogWriter
{
public:
  static std::pair<GnssLogResult, std::unique_ptr<GnssLogWriter>> create(
    const GnssLogWriterOptions & options);
  ~GnssLogWriter();
  GnssLogWriter(const GnssLogWriter &) = delete;
  GnssLogWriter & operator=(const GnssLogWriter &) = delete;

  GnssLogResult append(
    const ReceivedSentence & sentence, std::uint64_t host_monotonic_ns,
    std::uint64_t connection_epoch);
  GnssLogResult flush();
  GnssLogResult close();
  std::uint64_t committed_sentences() const noexcept {return committed_sentences_;}

private:
  explicit GnssLogWriter(GnssLogWriterOptions options);
  GnssLogResult open_files();
  GnssLogResult checkpoint();

  GnssLogWriterOptions options_;
  std::ofstream raw_;
  std::ofstream gga_;
  std::ofstream heading_;
  std::uint64_t committed_sentences_{0U};
  bool closed_{false};
};

}  // namespace ppbng_gnss
