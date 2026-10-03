#include "ppbng_storage/dataset_session.hpp"
#include "ppbng_storage/durable_write_qualification.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

namespace
{
const char * usage = R"(PPBNG durable-write qualification (DOES NOT RUN WITHOUT EXPLICIT CONFIRMATION)

Required:
  --confirm-durable-write-qualification
  --output-root <existing-directory>

Bounded optional parameters:
  --duration-seconds <1..60>       default 10
  --block-mib <1..64>              default 8
  --maximum-test-mib <1..16384>    default 4096

The tool creates one unique probe with CREATE_NEW, writes with FILE_FLAG_WRITE_THROUGH,
calls FlushFileBuffers, deletes only that owned probe, and atomically publishes a unique
JSON evidence file. It never edits machine YAML or overwrites an existing file.
)";

#ifdef _WIN32
std::string windows_error(const char * operation)
{
  return std::string(operation) + " failed with Windows error " + std::to_string(GetLastError());
}

std::string json_escape(const std::string & value)
{
  std::ostringstream output;
  for (const unsigned char character : value) {
    switch (character) {
      case '"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (character < 0x20U) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0') <<
            static_cast<unsigned int>(character) << std::dec;
        } else {output << static_cast<char>(character);}
    }
  }
  return output.str();
}

std::string utc_now()
{
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
  if (gmtime_s(&utc, &time) != 0) {throw std::runtime_error("UTC conversion failed");}
  std::ostringstream output;
  output << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return output.str();
}

class OwnedExclusiveFile
{
public:
  explicit OwnedExclusiveFile(std::filesystem::path path, const bool temporary)
  : path_(std::move(path))
  {
    const DWORD attributes = FILE_FLAG_SEQUENTIAL_SCAN | FILE_FLAG_WRITE_THROUGH |
      (temporary ? FILE_ATTRIBUTE_TEMPORARY : FILE_ATTRIBUTE_NORMAL);
    handle_ = CreateFileW(path_.c_str(), GENERIC_WRITE | DELETE, 0, nullptr, CREATE_NEW,
      attributes, nullptr);
    if (handle_ == INVALID_HANDLE_VALUE) {throw std::runtime_error(windows_error("CreateFileW(CREATE_NEW)"));}
    owned_ = true;
  }

  ~OwnedExclusiveFile()
  {
    if (owned_ && handle_ != INVALID_HANDLE_VALUE) {
      FILE_DISPOSITION_INFO disposition{};
      disposition.DeleteFile = TRUE;
      if (SetFileInformationByHandle(handle_, FileDispositionInfo, &disposition,
        static_cast<DWORD>(sizeof(disposition))))
      {
        owned_ = false;
      }
    }
    close();
    if (owned_) {(void)DeleteFileW(path_.c_str());}
  }

  OwnedExclusiveFile(const OwnedExclusiveFile &) = delete;
  OwnedExclusiveFile & operator=(const OwnedExclusiveFile &) = delete;

  HANDLE handle() const noexcept {return handle_;}
  const std::filesystem::path & path() const noexcept {return path_;}

  void close() noexcept
  {
    if (handle_ != INVALID_HANDLE_VALUE) {
      (void)CloseHandle(handle_);
      handle_ = INVALID_HANDLE_VALUE;
    }
  }

  void delete_owned()
  {
    if (!owned_) {throw std::runtime_error("internal ownership error before probe deletion");}
    if (handle_ == INVALID_HANDLE_VALUE) {
      throw std::runtime_error("owned probe handle was closed before deletion");
    }
    FILE_DISPOSITION_INFO disposition{};
    disposition.DeleteFile = TRUE;
    if (!SetFileInformationByHandle(handle_, FileDispositionInfo, &disposition,
      static_cast<DWORD>(sizeof(disposition))))
    {
      throw std::runtime_error(windows_error("SetFileInformationByHandle(delete owned probe)"));
    }
    owned_ = false;
    close();
  }

  void relinquish_after_atomic_rename() noexcept {owned_ = false;}

private:
  std::filesystem::path path_;
  HANDLE handle_{INVALID_HANDLE_VALUE};
  bool owned_{false};
};

void write_all(HANDLE handle, const std::uint8_t * data, std::size_t size)
{
  std::size_t offset{};
  while (offset < size) {
    const auto remaining = size - offset;
    const auto request = static_cast<DWORD>((std::min)(remaining,
      static_cast<std::size_t>((std::numeric_limits<DWORD>::max)())));
    DWORD written{};
    if (!WriteFile(handle, data + offset, request, &written, nullptr) || written != request) {
      throw std::runtime_error(windows_error("WriteFile"));
    }
    offset += written;
  }
}

struct VolumeIdentity
{
  std::filesystem::path root;
  std::string serial_hex;
  std::string filesystem;
};

VolumeIdentity volume_identity(const std::filesystem::path & path)
{
  std::vector<wchar_t> volume_path(32768U, L'\0');
  if (!GetVolumePathNameW(path.c_str(), volume_path.data(),
    static_cast<DWORD>(volume_path.size())))
  {
    throw std::runtime_error(windows_error("GetVolumePathNameW"));
  }
  DWORD serial{};
  wchar_t filesystem[256]{};
  if (!GetVolumeInformationW(volume_path.data(), nullptr, 0U, &serial, nullptr, nullptr,
    filesystem, static_cast<DWORD>(std::size(filesystem))))
  {
    throw std::runtime_error(windows_error("GetVolumeInformationW"));
  }
  std::ostringstream serial_text;
  serial_text << std::hex << std::uppercase << std::setw(8) << std::setfill('0') << serial;
  return {std::filesystem::path(volume_path.data()), serial_text.str(),
    std::filesystem::path(filesystem).u8string()};
}

std::uint64_t available_bytes(const std::filesystem::path & root)
{
  ULARGE_INTEGER available{};
  if (!GetDiskFreeSpaceExW(root.c_str(), &available, nullptr, nullptr)) {
    throw std::runtime_error(windows_error("GetDiskFreeSpaceExW"));
  }
  return available.QuadPart;
}

void atomic_publish_evidence(const std::filesystem::path & temporary,
  const std::filesystem::path & final, const std::string & json)
{
  OwnedExclusiveFile file(temporary, false);
  write_all(file.handle(), reinterpret_cast<const std::uint8_t *>(json.data()), json.size());
  if (!FlushFileBuffers(file.handle())) {throw std::runtime_error(windows_error("FlushFileBuffers(evidence)"));}
  file.close();
  if (!MoveFileExW(temporary.c_str(), final.c_str(), MOVEFILE_WRITE_THROUGH)) {
    throw std::runtime_error(windows_error("MoveFileExW(unique evidence)"));
  }
  file.relinquish_after_atomic_rename();
}
#endif
}  // namespace

int main(int argc, char ** argv)
{
  std::vector<std::string> arguments;
  for (int index = 1; index < argc; ++index) {arguments.emplace_back(argv[index]);}
  const auto parsed = ppbng_storage::parse_durable_write_qualification_arguments(arguments);
  if (!parsed.parsed) {
    std::cerr << "REFUSED: " << parsed.detail << "\n\n" << usage;
    return 2;
  }
  if (parsed.options.show_help) {std::cout << usage; return 0;}
  const auto validation = ppbng_storage::validate_durable_write_qualification_options(parsed.options);
  if (!validation.valid) {
    std::cerr << "REFUSED: " << validation.detail << "\n\n" << usage;
    return 2;
  }

#ifndef _WIN32
  std::cerr << "REFUSED: this qualification implementation requires Windows\n";
  return 2;
#else
  try {
    std::error_code error;
    const auto canonical_root = std::filesystem::weakly_canonical(
      std::filesystem::absolute(parsed.options.output_root, error), error);
    if (error || !std::filesystem::is_directory(canonical_root, error) || error) {
      throw std::runtime_error("output_root must be an existing canonical directory");
    }
    const auto before_available = available_bytes(canonical_root);
    if (before_available < validation.plan.required_available_bytes) {
      throw std::runtime_error("insufficient capacity: maximum probe plus 100 GiB reserve is required");
    }
    const auto identity = ppbng_storage::make_session_identity_now();
    const auto probe_path = canonical_root /
      std::filesystem::u8path(".ppbng_durable_probe_" + identity.unique_id + ".tmp");
    const auto evidence_temporary = canonical_root /
      std::filesystem::u8path(".ppbng_durable_evidence_" + identity.unique_id + ".tmp");
    const auto evidence_final = canonical_root /
      std::filesystem::u8path("ppbng_durable_evidence_" + identity.unique_id + ".json");
    const auto volume = volume_identity(canonical_root);
    const auto started_utc = utc_now();
    std::vector<std::uint8_t> block(static_cast<std::size_t>(validation.plan.block_bytes));
    for (std::size_t index = 0U; index < block.size(); ++index) {
      block[index] = static_cast<std::uint8_t>((index * 131U + 17U) & 0xffU);
    }
    std::uint64_t bytes_written{};
    double elapsed_seconds{};
    {
      OwnedExclusiveFile probe(probe_path, true);
      const auto started = std::chrono::steady_clock::now();
      do {
        if (bytes_written > validation.plan.maximum_test_bytes - validation.plan.block_bytes) {break;}
        write_all(probe.handle(), block.data(), block.size());
        bytes_written += validation.plan.block_bytes;
      } while (std::chrono::steady_clock::now() - started <
        std::chrono::seconds(parsed.options.duration_seconds));
      if (bytes_written == 0U) {throw std::runtime_error("qualification wrote no complete block");}
      if (!FlushFileBuffers(probe.handle())) {throw std::runtime_error(windows_error("FlushFileBuffers(probe)"));}
      const auto finished = std::chrono::steady_clock::now();
      elapsed_seconds = std::chrono::duration<double>(finished - started).count();
      if (!(elapsed_seconds > 0.0)) {throw std::runtime_error("invalid measured duration");}
      probe.delete_owned();
    }
    if (available_bytes(canonical_root) < validation.plan.reserve_bytes) {
      throw std::runtime_error("100 GiB reserve was not preserved after probe cleanup");
    }
    const auto completed_utc = utc_now();
    const auto bytes_per_second = static_cast<double>(bytes_written) / elapsed_seconds;
    std::ostringstream evidence;
    evidence << std::fixed << std::setprecision(6);
    evidence << "{\n  \"schema_version\": 1,\n";
    evidence << "  \"tool\": \"ppbng_storage_durable_write_qualification\",\n";
    evidence << "  \"run_id\": \"" << json_escape(identity.unique_id) << "\",\n";
    evidence << "  \"normalized_output_root\": \"" << json_escape(canonical_root.u8string()) << "\",\n";
    evidence << "  \"volume_root\": \"" << json_escape(volume.root.u8string()) << "\",\n";
    evidence << "  \"volume_serial_hex\": \"" << json_escape(volume.serial_hex) << "\",\n";
    evidence << "  \"filesystem\": \"" << json_escape(volume.filesystem) << "\",\n";
    evidence << "  \"started_utc\": \"" << json_escape(started_utc) << "\",\n";
    evidence << "  \"completed_utc\": \"" << json_escape(completed_utc) << "\",\n";
    evidence << "  \"bytes_written\": " << bytes_written << ",\n";
    evidence << "  \"duration_seconds\": " << elapsed_seconds << ",\n";
    evidence << "  \"bytes_per_second\": " << bytes_per_second << ",\n";
    evidence << "  \"mib_per_second\": " << bytes_per_second / (1024.0 * 1024.0) << ",\n";
    evidence << "  \"parameters\": {\n";
    evidence << "    \"requested_duration_seconds\": " << parsed.options.duration_seconds << ",\n";
    evidence << "    \"block_bytes\": " << validation.plan.block_bytes << ",\n";
    evidence << "    \"maximum_test_bytes\": " << validation.plan.maximum_test_bytes << ",\n";
    evidence << "    \"reserve_bytes\": " << validation.plan.reserve_bytes << ",\n";
    evidence << "    \"write_through\": true,\n";
    evidence << "    \"flush_file_buffers\": true\n  }\n}\n";
    atomic_publish_evidence(evidence_temporary, evidence_final, evidence.str());

    std::cout << "QUALIFICATION EVIDENCE CREATED; machine YAML was not modified.\n";
    std::cout << "Review the JSON, then manually copy only these qualification references:\n";
    std::cout << "  evidence_path: \"" << evidence_final.u8string() << "\"\n";
    std::cout << "  expected_volume_serial_hex: \"" << volume.serial_hex << "\"\n";
    std::cout << "  measured_bytes_per_second: " << std::fixed << std::setprecision(0) <<
      bytes_per_second << "\n";
    std::cout << "  qualified_utc: \"" << completed_utc << "\"\n";
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << "QUALIFICATION FAILED CLOSED: " << exception.what() << '\n';
    return 1;
  }
#endif
}
