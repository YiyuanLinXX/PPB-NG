#include "ppbng_hsi/envi_part_verifier.hpp"
#include "ppbng_hsi/dataset_context_verifier.hpp"
#include "ppbng_storage/association_verifier.hpp"
#include "ppbng_storage/json_verifier.hpp"
#include "ppbng_storage/manifest_verifier.hpp"
#include "ppbng_storage/segment_verifier.hpp"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace
{
constexpr const char * kIndexSuffix = ".index.csv";
constexpr const char * kSegmentSuffix = ".ppbseg";

bool is_part_index(const std::filesystem::path & path)
{
  const auto name = path.filename().string();
  const std::string suffix{kIndexSuffix};
  return name.size() > suffix.size() &&
    name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0 &&
    name.find("_segment_") != std::string::npos &&
    name.find("_part_") != std::string::npos;
}

bool is_json_lines(const std::filesystem::path & path)
{
  const auto name = path.filename().string();
  for (const std::string suffix : {".jsonl", ".ndjson"}) {
    if (name.size() > suffix.size() &&
      name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0)
    {
      return true;
    }
  }
  return false;
}

bool collect_json_lines(
  const std::filesystem::path & directory,
  std::vector<std::filesystem::path> & paths,
  std::error_code & error)
{
  if (!std::filesystem::is_directory(directory, error) || error) {return false;}
  for (std::filesystem::directory_iterator it(directory, error), end;
    it != end && !error; it.increment(error))
  {
    const auto status = it->symlink_status(error);
    if (error) {return false;}
    if (std::filesystem::is_regular_file(status) && is_json_lines(it->path())) {
      paths.push_back(it->path());
    }
  }
  return !error;
}
}  // namespace

int run(
  const std::filesystem::path & session,
  const std::string & hsi_only_stream = {}, const bool quick = false)
{
  std::error_code error;
  if (!std::filesystem::is_directory(session, error) || error) {
    std::cerr << "refused: session directory is missing or unreadable\n";
    return 2;
  }

  const auto segments_directory = session / "segments";
  ppbng_hsi::EnviVerificationOptions verification_options;
  verification_options.full_payload_crc = !quick;
  verification_options.progress = [quick](
    const std::string & part, const std::uint64_t processed, const std::uint64_t total)
    {
      const auto percent = total == 0U ? 100.0 :
        100.0 * static_cast<double>(processed) / static_cast<double>(total);
      std::cout << "progress mode=" << (quick ? "quick" : "full") <<
        " part=" << part << " processed_mib=" << processed / (1024U * 1024U) <<
        " total_mib=" << total / (1024U * 1024U) << " percent=" << percent <<
        '\n' << std::flush;
    };
  std::vector<std::filesystem::path> indices;
  for (const auto & directory : {session, segments_directory}) {
    error.clear();
    if (!std::filesystem::is_directory(directory, error) || error) {
      if (directory == session) {
        std::cerr << "verification failed: cannot enumerate session directory\n";
        return 2;
      }
      continue;
    }
    for (std::filesystem::directory_iterator it(directory, error), end;
      it != end && !error; it.increment(error))
    {
      const auto status = it->symlink_status(error);
      if (error) {break;}
      if (std::filesystem::is_regular_file(status) && is_part_index(it->path())) {
        indices.push_back(it->path());
      }
    }
    if (error) {break;}
  }
  if (error) {
    std::cerr << "verification failed: cannot enumerate session directory\n";
    return 2;
  }
  std::sort(indices.begin(), indices.end());

  std::vector<std::filesystem::path> segments;
  error.clear();
  if (std::filesystem::is_directory(segments_directory, error) && !error) {
    for (std::filesystem::directory_iterator it(segments_directory, error), end;
      it != end && !error; it.increment(error))
    {
      const auto status = it->symlink_status(error);
      if (error) {break;}
      const auto name = it->path().filename().string();
      const std::string suffix{kSegmentSuffix};
      if (std::filesystem::is_regular_file(status) && name.size() > suffix.size() &&
        name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0)
      {
        segments.push_back(std::filesystem::path("segments") / it->path().filename());
      }
    }
  } else if (error) {
    std::cerr << "verification failed: cannot inspect segments directory\n";
    return 2;
  }
  if (error) {
    std::cerr << "verification failed: cannot enumerate segments directory\n";
    return 2;
  }
  std::sort(segments.begin(), segments.end());
  if (indices.empty() && segments.empty()) {
    std::cerr << "verification failed: no HSI ENVI indices or ppbseg files found\n";
    return 1;
  }

  if (!hsi_only_stream.empty()) {
    if (hsi_only_stream != "fx10e" && hsi_only_stream != "swir") {
      std::cerr << "refused: --hsi-only stream must be fx10e or swir\n";
      return 2;
    }
    // A combined dual-camera session legitimately contains the peer stream.
    // Verify the requested stream without rejecting or rereading the peer's
    // much larger payload; companion completeness is still checked globally.
    const auto hsi_dataset = ppbng_hsi::verify_envi_dataset(
      segments_directory, {hsi_only_stream}, false, verification_options);
    if (!hsi_dataset.success) {
      std::cerr << "FAIL HSI dataset: " << hsi_dataset.message << '\n';
      return 1;
    }
    std::uint64_t lines = 0U;
    std::uint64_t bytes = 0U;
    std::uint64_t parts = 0U;
    std::uint64_t crc_lines = 0U;
    std::uint64_t crc_unavailable = 0U;
    bool requested_stream_found = false;
    for (const auto & stream : hsi_dataset.streams) {
      if (stream.stream_name != hsi_only_stream) {continue;}
      requested_stream_found = true;
      lines += stream.line_count;
      bytes += stream.payload_bytes;
      parts += stream.part_count;
      crc_lines += stream.crc_lines_checked;
      crc_unavailable += stream.crc_lines_unavailable;
      std::cout << "OK HSI stream=" << stream.stream_name <<
        " segments=" << stream.segment_count << " parts=" << stream.part_count <<
        " lines=" << stream.line_count << " bytes=" << stream.payload_bytes <<
        " crc_lines_checked=" << stream.crc_lines_checked << '\n';
    }
    if (!requested_stream_found) {
      std::cerr << "FAIL HSI dataset: missing HSI stream: " << hsi_only_stream << '\n';
      return 1;
    }
    std::cout << "summary hsi_parts=" << parts << " hsi_lines=" << lines <<
      " hsi_bytes=" << bytes << " crc_lines_checked=" << crc_lines <<
      " verification_mode=" << (quick ? "quick" : "full") <<
      " crc_lines_unavailable=" << crc_unavailable <<
      " payload_crc_complete=" << (!quick && crc_unavailable == 0U ? "true" : "false") << " valid=true\n";
    return 0;
  }

  std::uint64_t total_lines = 0U;
  std::uint64_t total_bytes = 0U;
  bool all_valid = true;
  std::uint64_t crc_unavailable = 0U;

  std::string verified_session_id;
  const auto manifest_path = session / "manifest.json";
  error.clear();
  const auto manifest_status = std::filesystem::symlink_status(manifest_path, error);
  if (error || !std::filesystem::is_regular_file(manifest_status)) {
    all_valid = false;
    std::cerr << "FAIL manifest.json: missing, unreadable, or not a regular file\n";
  } else {
    const auto manifest = ppbng_storage::verify_session_manifest_file(manifest_path);
    if (manifest.success) {
      verified_session_id = manifest.session_id;
      std::cout << "OK manifest.json state=" << manifest.state <<
        " devices=" << manifest.device_count << " mode=" <<
        (manifest.hardware_enabled ? "hardware" : "simulation") << '\n';
    } else {
      all_valid = false;
      std::cerr << "FAIL manifest.json: " << manifest.message << '\n';
    }
  }

  std::vector<std::filesystem::path> json_lines;
  error.clear();
  if (!collect_json_lines(session, json_lines, error)) {
    std::cerr << "verification failed: cannot enumerate session JSON-lines files\n";
    return 2;
  }
  error.clear();
  if (!collect_json_lines(segments_directory, json_lines, error)) {
    std::cerr << "verification failed: cannot enumerate segment JSON-lines files\n";
    return 2;
  }
  std::sort(json_lines.begin(), json_lines.end());
  std::uint64_t json_records = 0U;
  for (const auto & path : json_lines) {
    const auto result = ppbng_storage::verify_json_lines_file(path);
    const auto relative = path.lexically_relative(session).generic_string();
    if (result.success) {
      json_records += result.record_count;
      std::cout << "OK " << relative << " json_records=" << result.record_count <<
        " bytes=" << result.byte_count << '\n';
    } else {
      all_valid = false;
      std::cerr << "FAIL " << relative << " line=" << result.error_line <<
        " column=" << result.error_column << ": " << result.message << '\n';
    }
  }

  const auto hsi_dataset = ppbng_hsi::verify_envi_dataset(
    segments_directory, {"fx10e", "swir"}, true, verification_options);
  if (!hsi_dataset.success) {
    all_valid = false;
    std::cerr << "FAIL HSI dataset: " << hsi_dataset.message << '\n';
  } else {
    for (const auto & stream : hsi_dataset.streams) {
      total_lines += stream.line_count;
      total_bytes += stream.payload_bytes;
      crc_unavailable += stream.crc_lines_unavailable;
      std::cout << "OK HSI stream=" << stream.stream_name <<
        " segments=" << stream.segment_count << " parts=" << stream.part_count <<
        " lines=" << stream.line_count << " bytes=" << stream.payload_bytes <<
        " crc_lines_checked=" << stream.crc_lines_checked << '\n';
    }
  }
  std::uint64_t total_records = 0U;
  std::uint64_t segment_payload_bytes = 0U;
  const auto segment_set = ppbng_storage::verify_segment_set(session);
  if (!segment_set.success) {
    all_valid = false;
    std::cerr << "FAIL ppbseg set: " << segment_set.message << '\n';
  } else {
    for (const auto & stream : segment_set.streams) {
      total_records += stream.record_count;
      segment_payload_bytes += stream.payload_bytes;
      std::cout << "OK ppbseg stream=" << stream.stream_name <<
        " segments=" << stream.segment_count << " records=" << stream.record_count <<
        " last_sample=" << stream.last_sample_id << " payload_bytes=" <<
        stream.payload_bytes << '\n';
      const auto association = ppbng_storage::verify_camera_association(session, stream);
      if (association.success) {
        std::cout << "OK association stream=" << stream.stream_name <<
          " pending=" << association.pending_records << " terminal=" <<
          association.terminal_records << " matched=" << association.matched_records <<
          " degraded_events=" << association.degraded_records << '\n';
      } else {
        all_valid = false;
        std::cerr << "FAIL association stream=" << stream.stream_name << ": " <<
          association.message << '\n';
      }
    }
  }
  if (hsi_dataset.success && segment_set.success && !verified_session_id.empty()) {
    const auto contexts = ppbng_hsi::verify_dataset_frame_context(
      session, verified_session_id, segment_set);
    if (contexts.success) {
      std::cout << "OK frame_context records=" << contexts.context_records <<
        " gnss_unavailable=" << contexts.unavailable_gnss_records <<
        " rsm_unavailable=" << contexts.unavailable_rsm_records << '\n';
    } else {
      all_valid = false;
      std::cerr << "FAIL frame_context: " << contexts.message << '\n';
    }
  }
  std::cout << "summary hsi_parts=" << indices.size() << " hsi_lines=" << total_lines <<
    " hsi_bytes=" << total_bytes << " ppbseg_files=" << segments.size() <<
    " ppbseg_records=" << total_records << " ppbseg_payload_bytes=" <<
    segment_payload_bytes << " json_lines_files=" << json_lines.size() <<
    " json_records=" << json_records << " valid=" <<
    (all_valid ? "true" : "false") << " verification_mode=" <<
    (quick ? "quick" : "full") << " payload_crc_complete=" <<
    (!quick && all_valid && crc_unavailable == 0U ? "true" : "false") <<
    " crc_lines_unavailable=" << crc_unavailable << '\n';
  return all_valid ? 0 : 1;
}

#ifdef _WIN32
int wmain(int argc, wchar_t ** argv)
{
  if (argc < 2) {
    std::cerr << "usage: ppbng_verify_dataset <session-directory> [--hsi-only fx10e|swir] [--quick]\n";
    return 2;
  }
  std::string stream;
  bool quick = false;
  for (int index = 2; index < argc; ++index) {
    const std::wstring option(argv[index]);
    if (option == L"--quick" && !quick) {quick = true;continue;}
    if (option == L"--hsi-only" && stream.empty() && index + 1 < argc) {
      const std::wstring value(argv[++index]);
      stream = value == L"fx10e" ? "fx10e" : value == L"swir" ? "swir" : "invalid";
      continue;
    }
    std::cerr << "refused: unknown, duplicate, or incomplete option\n";
    return 2;
  }
  return run(std::filesystem::path(argv[1]), stream, quick);
}
#else
int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "usage: ppbng_verify_dataset <session-directory> [--hsi-only fx10e|swir] [--quick]\n";
    return 2;
  }
  std::string stream;
  bool quick = false;
  for (int index = 2; index < argc; ++index) {
    const std::string option(argv[index]);
    if (option == "--quick" && !quick) {quick = true;continue;}
    if (option == "--hsi-only" && stream.empty() && index + 1 < argc) {
      stream = argv[++index];
      continue;
    }
    std::cerr << "refused: unknown, duplicate, or incomplete option\n";
    return 2;
  }
  return run(std::filesystem::u8path(argv[1]), stream, quick);
}
#endif
