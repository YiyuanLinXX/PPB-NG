#include "ppbng_storage/dataset_session.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cwctype>
#include <fstream>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <system_error>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace ppbng_storage
{
namespace
{

constexpr std::size_t kMaxUserComponentBytes = 120;

bool is_ascii_space(const char value) noexcept
{
  return value == ' ' || value == '\t' || value == '\r' || value == '\n';
}

std::size_t utf8_sequence_size(const std::string_view input, const std::size_t offset) noexcept
{
  const auto lead = static_cast<unsigned char>(input[offset]);
  std::size_t size = 0;
  if (lead >= 0xc2U && lead <= 0xdfU) {
    size = 2;
  } else if (lead >= 0xe0U && lead <= 0xefU) {
    size = 3;
  } else if (lead >= 0xf0U && lead <= 0xf4U) {
    size = 4;
  } else {
    return 0;
  }
  if (offset + size > input.size()) {
    return 0;
  }
  for (std::size_t index = 1; index < size; ++index) {
    const auto continuation = static_cast<unsigned char>(input[offset + index]);
    if ((continuation & 0xc0U) != 0x80U) {
      return 0;
    }
  }
  return size;
}

std::string ascii_upper(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char character) {
    return static_cast<char>(std::toupper(character));
  });
  return value;
}

bool is_windows_reserved_name(const std::string & component)
{
  const auto dot = component.find('.');
  const auto stem = ascii_upper(component.substr(0, dot));
  if (stem == "CON" || stem == "PRN" || stem == "AUX" || stem == "NUL") {
    return true;
  }
  if (stem.size() == 4 && stem[3] >= '1' && stem[3] <= '9') {
    return stem.substr(0, 3) == "COM" || stem.substr(0, 3) == "LPT";
  }
  return false;
}

bool is_hex(const char value) noexcept
{
  return (value >= '0' && value <= '9') || (value >= 'a' && value <= 'f');
}

bool is_valid_utc(const std::string & value) noexcept
{
  if (value.size() != 16 || value[8] != 'T' || value[15] != 'Z') {
    return false;
  }
  for (std::size_t index = 0; index < value.size(); ++index) {
    if (index == 8 || index == 15) {
      continue;
    }
    if (value[index] < '0' || value[index] > '9') {
      return false;
    }
  }
  return true;
}

bool is_valid_uuid(const std::string & value) noexcept
{
  if (value.size() != 36) {
    return false;
  }
  for (std::size_t index = 0; index < value.size(); ++index) {
    const bool hyphen = index == 8 || index == 13 || index == 18 || index == 23;
    if ((hyphen && value[index] != '-') || (!hyphen && !is_hex(value[index]))) {
      return false;
    }
  }
  return true;
}

std::filesystem::path normalized_absolute(
  const std::filesystem::path & path, std::error_code & error)
{
  auto absolute = std::filesystem::absolute(path, error);
  if (error) {
    return {};
  }
  return std::filesystem::weakly_canonical(absolute, error);
}

std::wstring comparable_component(const std::filesystem::path & component)
{
  auto value = component.native();
#ifdef _WIN32
  std::transform(value.begin(), value.end(), value.begin(), [](const wchar_t character) {
    return static_cast<wchar_t>(std::towlower(character));
  });
#endif
  return value;
}

bool is_beneath_or_equal(
  const std::filesystem::path & candidate, const std::filesystem::path & root)
{
  auto candidate_it = candidate.begin();
  auto root_it = root.begin();
  for (; root_it != root.end(); ++root_it, ++candidate_it) {
    if (candidate_it == candidate.end() ||
      comparable_component(*candidate_it) != comparable_component(*root_it))
    {
      return false;
    }
  }
  return true;
}

bool is_safe_relative_path(const std::filesystem::path & path)
{
  if (path.empty() || path.is_absolute() || path.has_root_name() || path.has_root_directory()) {
    return false;
  }
  for (const auto & component : path) {
    if (component == "." || component == ".." || component.empty()) {
      return false;
    }
  }
  return true;
}

std::string path_detail(const std::error_code & error)
{
  return error ? error.message() : std::string{};
}

#ifdef _WIN32
AtomicWriteResult write_and_replace_windows(
  const std::filesystem::path & temporary_path,
  const std::filesystem::path & target_path,
  const std::string_view contents, bool durable)
{
  HANDLE file = CreateFileW(
    temporary_path.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_NEW,
    FILE_ATTRIBUTE_NORMAL | (durable ? FILE_FLAG_WRITE_THROUGH : 0), nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    const auto error = GetLastError();
    return {
      error == ERROR_FILE_EXISTS ? AtomicWriteError::temporary_file_exists :
      AtomicWriteError::io_error,
      "CreateFileW failed: " + std::to_string(error)};
  }

  bool write_ok = true;
  std::size_t offset = 0;
  while (offset < contents.size()) {
    const auto remaining = contents.size() - offset;
    const auto request = static_cast<DWORD>(std::min<std::size_t>(
      remaining, std::numeric_limits<DWORD>::max()));
    DWORD written = 0;
    if (!WriteFile(file, contents.data() + offset, request, &written, nullptr) || written != request) {
      write_ok = false;
      break;
    }
    offset += written;
  }
  if (write_ok && durable) {
    write_ok = FlushFileBuffers(file) != 0;
  }
  const bool close_ok = CloseHandle(file) != 0;
  if (!write_ok || !close_ok) {
    const auto error = GetLastError();
    DeleteFileW(temporary_path.c_str());
    return {AtomicWriteError::io_error, "write/flush failed: " + std::to_string(error)};
  }

  if (!MoveFileExW(
      temporary_path.c_str(), target_path.c_str(),
      MOVEFILE_REPLACE_EXISTING | (durable ? MOVEFILE_WRITE_THROUGH : 0)))
  {
    const auto error = GetLastError();
    DeleteFileW(temporary_path.c_str());
    return {AtomicWriteError::io_error, "MoveFileExW failed: " + std::to_string(error)};
  }
  return {};
}
#endif

}  // namespace

SessionIdentity make_session_identity_now()
{
  const auto now = std::chrono::system_clock::now();
  const auto now_time = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
#ifdef _WIN32
  gmtime_s(&utc, &now_time);
#else
  gmtime_r(&now_time, &utc);
#endif

  std::ostringstream timestamp;
  timestamp << std::put_time(&utc, "%Y%m%dT%H%M%SZ");

  std::array<unsigned char, 16> bytes{};
  std::random_device random;
  for (auto & byte : bytes) {
    byte = static_cast<unsigned char>(random());
  }
  bytes[6] = static_cast<unsigned char>((bytes[6] & 0x0fU) | 0x40U);
  bytes[8] = static_cast<unsigned char>((bytes[8] & 0x3fU) | 0x80U);

  std::ostringstream id;
  id << std::hex << std::setfill('0');
  for (std::size_t index = 0; index < bytes.size(); ++index) {
    if (index == 4 || index == 6 || index == 8 || index == 10) {
      id << '-';
    }
    id << std::setw(2) << static_cast<unsigned int>(bytes[index]);
  }
  return {timestamp.str(), id.str()};
}

std::string sanitize_windows_component(const std::string_view input)
{
  std::string result;
  result.reserve(std::min(input.size(), kMaxUserComponentBytes));
  for (std::size_t index = 0; index < input.size();) {
    if (result.size() >= kMaxUserComponentBytes) {
      break;
    }
    const auto value = static_cast<unsigned char>(input[index]);
    if (value >= 0x80U) {
      const auto sequence_size = utf8_sequence_size(input, index);
      if (sequence_size == 0) {
        result.push_back('_');
        ++index;
        continue;
      }
      if (result.size() + sequence_size > kMaxUserComponentBytes) {
        break;
      }
      result.append(input.substr(index, sequence_size));
      index += sequence_size;
      continue;
    }
    const bool forbidden = value < 32 || value == '<' || value == '>' || value == ':' ||
      value == '"' || value == '/' || value == '\\' || value == '|' || value == '?' ||
      value == '*';
    result.push_back(forbidden ? '_' : static_cast<char>(value));
    ++index;
  }

  while (!result.empty() && (result.back() == ' ' || result.back() == '.')) {
    result.pop_back();
  }
  if (result.empty() || std::all_of(result.begin(), result.end(), is_ascii_space)) {
    result = "dataset";
  }
  if (is_windows_reserved_name(result)) {
    result.insert(result.begin(), '_');
  }
  return result;
}

bool is_valid_session_identity(const SessionIdentity & identity) noexcept
{
  return is_valid_utc(identity.utc_timestamp) && is_valid_uuid(identity.unique_id);
}

DatasetSessionResult create_dataset_session(
  const std::filesystem::path & output_root,
  const std::string_view user_name,
  const SessionIdentity & identity) noexcept
{
  try {
    if (user_name.empty() || std::all_of(user_name.begin(), user_name.end(), is_ascii_space)) {
      return {DatasetSessionError::empty_user_name, {}, "dataset name is empty"};
    }
    if (!is_valid_session_identity(identity)) {
      return {DatasetSessionError::invalid_identity, {}, "invalid UTC timestamp or UUID"};
    }

    std::error_code error;
    const auto root = normalized_absolute(output_root, error);
    if (error || !std::filesystem::is_directory(root, error) || error) {
      return {DatasetSessionError::output_root_not_directory, {}, path_detail(error)};
    }

    const auto directory_name = sanitize_windows_component(user_name) + "__" +
      identity.utc_timestamp + "__" + identity.unique_id;
    const auto session = (root / std::filesystem::u8path(directory_name)).lexically_normal();
    if (!is_beneath_or_equal(session, root) || session.parent_path() != root) {
      return {DatasetSessionError::unsafe_path, {}, "session path escaped output root"};
    }

    if (!std::filesystem::create_directory(session, error)) {
      return {
        error ? DatasetSessionError::io_error : DatasetSessionError::already_exists, {},
        path_detail(error)};
    }

    const auto segments = session / "segments";
    if (!std::filesystem::create_directory(segments, error)) {
      // This exact directory was created by this invocation and contains no user data.
      std::error_code cleanup_error;
      std::filesystem::remove(session, cleanup_error);
      return {DatasetSessionError::io_error, {}, path_detail(error)};
    }

    DatasetSessionPaths paths;
    paths.output_root = root;
    paths.session_directory = session;
    paths.manifest_path = session / "manifest.json";
    paths.segments_directory = segments;
    paths.segment_index_path = session / "segments.tsv";
    paths.checkpoint_path = session / "checkpoint.txt";
    return {DatasetSessionError::none, std::move(paths), {}};
  } catch (const std::exception & error) {
    return {DatasetSessionError::io_error, {}, error.what()};
  }
}

AtomicWriteResult atomic_write_text(
  const std::filesystem::path & session_root,
  const std::filesystem::path & relative_target,
  const std::string_view contents,
  const std::string_view operation_id) noexcept
{
  return atomic_write_text_with_durability(session_root, relative_target, contents, operation_id, true);
}

AtomicWriteResult atomic_write_text_with_durability(
  const std::filesystem::path & session_root,
  const std::filesystem::path & relative_target,
  const std::string_view contents,
  const std::string_view operation_id, bool durable) noexcept
{
  try {
    if (!is_safe_relative_path(relative_target)) {
      return {AtomicWriteError::unsafe_target, "target must be a safe relative path"};
    }
    const auto safe_operation_id = sanitize_windows_component(operation_id);
    if (operation_id.empty() || safe_operation_id != operation_id) {
      return {AtomicWriteError::unsafe_target, "operation ID is not a safe filename component"};
    }

    std::error_code error;
    const auto root = normalized_absolute(session_root, error);
    if (error || !std::filesystem::is_directory(root, error) || error) {
      return {AtomicWriteError::invalid_root, path_detail(error)};
    }

    const auto target = (root / relative_target).lexically_normal();
    const auto parent = normalized_absolute(target.parent_path(), error);
    if (error || !std::filesystem::is_directory(parent, error) || error) {
      return {AtomicWriteError::target_parent_missing, path_detail(error)};
    }
    if (!is_beneath_or_equal(parent, root)) {
      return {AtomicWriteError::unsafe_target, "target parent escaped session root"};
    }

    auto temporary = target;
    temporary += std::filesystem::u8path(".tmp." + safe_operation_id);
#ifdef _WIN32
    return write_and_replace_windows(temporary, target, contents, durable);
#else
    if (std::filesystem::exists(temporary, error)) {
      return {AtomicWriteError::temporary_file_exists, {}};
    }
    std::ofstream stream(temporary, std::ios::binary | std::ios::out | std::ios::trunc);
    stream.write(contents.data(), static_cast<std::streamsize>(contents.size()));
    stream.flush();
    if (!stream.good()) {
      stream.close();
      std::filesystem::remove(temporary, error);
      return {AtomicWriteError::io_error, "temporary file write failed"};
    }
    stream.close();
    std::filesystem::rename(temporary, target, error);
    if (error) {
      std::filesystem::remove(temporary, error);
      return {AtomicWriteError::io_error, path_detail(error)};
    }
    return {};
#endif
  } catch (const std::exception & error) {
    return {AtomicWriteError::io_error, error.what()};
  }
}

SegmentIndexError SegmentIndex::append(const SegmentRecord & record)
{
  if (record.segment_index != records_.size()) {
    return SegmentIndexError::unexpected_segment_index;
  }
  const auto path = std::filesystem::u8path(record.relative_path);
  if (!is_safe_relative_path(path) || path.begin() == path.end() || *path.begin() != "segments") {
    return SegmentIndexError::unsafe_relative_path;
  }
  if (record.first_event_id > record.last_event_id) {
    return SegmentIndexError::invalid_event_range;
  }
  if (record.frame_count == 0) {
    return SegmentIndexError::zero_frame_count;
  }
  records_.push_back(record);
  return SegmentIndexError::none;
}

std::string SegmentIndex::serialize_tsv() const
{
  std::ostringstream output;
  output << "segment_index\trelative_path\tfirst_event_id\tlast_event_id\tframe_count\n";
  for (const auto & record : records_) {
    output << record.segment_index << '\t' << record.relative_path << '\t' <<
      record.first_event_id << '\t' << record.last_event_id << '\t' <<
      record.frame_count << '\n';
  }
  return output.str();
}

std::string serialize_checkpoint(const DatasetCheckpoint & checkpoint)
{
  std::ostringstream output;
  output << "version=1\n";
  output << "next_segment_index=" << checkpoint.next_segment_index << '\n';
  output << "committed_frame_count=" << checkpoint.committed_frame_count << '\n';
  output << "last_committed_event_id=" << checkpoint.last_committed_event_id << '\n';
  return output.str();
}

bool parse_checkpoint(const std::string_view text, DatasetCheckpoint & checkpoint) noexcept
{
  try {
    std::istringstream input{std::string(text)};
    std::string version;
    std::string segment;
    std::string frames;
    std::string event;
    std::string extra;
    if (!std::getline(input, version) || !std::getline(input, segment) ||
      !std::getline(input, frames) || !std::getline(input, event) || std::getline(input, extra))
    {
      return false;
    }

    auto parse_value = [](const std::string & line, const std::string & prefix, std::uint64_t & value) {
        if (line.rfind(prefix, 0) != 0 || line.size() == prefix.size()) {
          return false;
        }
        const auto digits = line.substr(prefix.size());
        if (!std::all_of(digits.begin(), digits.end(), [](const unsigned char character) {
            return std::isdigit(character) != 0;
          }))
        {
          return false;
        }
        std::size_t consumed = 0;
        const auto parsed = std::stoull(digits, &consumed, 10);
        if (consumed != digits.size()) {
          return false;
        }
        value = parsed;
        return true;
      };

    DatasetCheckpoint parsed;
    if (version != "version=1" ||
      !parse_value(segment, "next_segment_index=", parsed.next_segment_index) ||
      !parse_value(frames, "committed_frame_count=", parsed.committed_frame_count) ||
      !parse_value(event, "last_committed_event_id=", parsed.last_committed_event_id))
    {
      return false;
    }
    checkpoint = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

}  // namespace ppbng_storage
