#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
namespace fs = std::filesystem;
using Spinnaker::GenApi::CEnumEntryPtr;
using Spinnaker::GenApi::CEnumerationPtr;
using Spinnaker::GenApi::CValuePtr;
using Spinnaker::GenApi::INodeMap;
using Spinnaker::GenApi::IsReadable;
using Spinnaker::GenApi::IsWritable;

std::string value(INodeMap & map, const char * name)
{
  CValuePtr node = map.GetNode(name);
  return IsReadable(node) ? std::string(node->ToString().c_str()) : std::string{};
}

void set_enum(INodeMap & map, const char * name, const char * target,
  std::vector<std::function<void()>> & rollback)
{
  CEnumerationPtr node = map.GetNode(name);
  if (!IsReadable(node) || !IsWritable(node)) throw std::runtime_error(std::string(name) + " is not writable");
  CEnumEntryPtr old_entry = node->GetCurrentEntry();
  CEnumEntryPtr new_entry = node->GetEntryByName(target);
  if (!IsReadable(old_entry) || !IsReadable(new_entry)) throw std::runtime_error(std::string(name) + " target unavailable");
  const auto old_value = old_entry->GetValue();
  node->SetIntValue(new_entry->GetValue());
  rollback.emplace_back([node, old_value]() {try {node->SetIntValue(old_value);} catch (...) {}});
  if (value(map, name) != target) throw std::runtime_error(std::string(name) + " readback mismatch");
}

std::uint64_t hash_payload(const void * data, std::size_t size)
{
  const auto * bytes = static_cast<const unsigned char *>(data);
  std::uint64_t hash = 1469598103934665603ULL;
  for (std::size_t i = 0; i < size; ++i) {hash ^= bytes[i]; hash *= 1099511628211ULL;}
  return hash;
}

bool heartbeat_fresh(const fs::path & heartbeat)
{
  if (!fs::exists(heartbeat)) return false;
  const auto age = fs::file_time_type::clock::now() - fs::last_write_time(heartbeat);
  return age <= std::chrono::seconds(5);
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 4) {
    std::cerr << "usage: ppbng_a6701_focus_recorder DEVICE_ID OUTPUT_DIRECTORY CONTROL_DIRECTORY" << std::endl;
    return 64;
  }
  const std::string requested_id = argv[1];
  const fs::path output = fs::absolute(argv[2]);
  const fs::path control = fs::absolute(argv[3]);
  const fs::path heartbeat = control / "HOST_HEARTBEAT";
  const fs::path stop_request = control / "STOP_REQUESTED";
  if (!fs::is_directory(control) || !heartbeat_fresh(heartbeat)) {
    std::cerr << "FOCUS_CAPTURE_FAILED host heartbeat missing or stale" << std::endl;
    return 1;
  }
  if (fs::exists(output) || !fs::create_directories(output)) {
    std::cerr << "FOCUS_CAPTURE_FAILED output directory must be new and creatable" << std::endl;
    return 1;
  }

  Spinnaker::SystemPtr system;
  Spinnaker::CameraList cameras;
  Spinnaker::CameraPtr camera;
  bool initialized = false;
  bool acquiring = false;
  std::vector<std::function<void()>> rollback;
  auto restore = [&]() {
    if (camera && acquiring) {try {camera->EndAcquisition();} catch (...) {} acquiring = false;}
    for (auto it = rollback.rbegin(); it != rollback.rend(); ++it) (*it)();
    rollback.clear();
    if (camera && initialized) {
      try {
        Spinnaker::GenApi::CCommandPtr stop = camera->GetNodeMap().GetNode("AcquisitionStop");
        if (IsWritable(stop)) stop->Execute();
      } catch (...) {}
      try {camera->DeInit();} catch (...) {}
      initialized = false;
    }
  };

  std::uint64_t complete = 0;
  std::string final_reason = "unknown";
  int exit_code = 1;
  try {
    system = Spinnaker::System::GetInstance();
    cameras = system->GetCameras();
    for (unsigned int i = 0; i < cameras.GetSize(); ++i) {
      auto candidate = cameras.GetByIndex(i);
      if (value(candidate->GetTLDeviceNodeMap(), "DeviceID") == requested_id) {
        if (camera) throw std::runtime_error("DeviceID is not unique");
        camera = candidate;
      }
    }
    if (!camera) throw std::runtime_error("exact DeviceID not found");
    camera->Init();
    initialized = true;
    auto & map = camera->GetNodeMap();
    if (value(map, "CameraModel") != "A6701" || value(map, "Ready") != "1" ||
      value(map, "FPACold") != "1") {
      throw std::runtime_error("A6701 identity/Ready/FPACold preflight failed");
    }
    if (value(map, "Width") != "640" || value(map, "Height") != "513" ||
      value(map, "PixelFormat") != "Mono16" || value(map, "IRFormat") != "Radiometric" ||
      value(map, "TriggerMode") != "FreeRun") {
      throw std::runtime_error("image/trigger contract unsuitable for focus capture");
    }
    set_enum(map, "FrameSyncMode", "Integration", rollback);
    set_enum(map, "FrameSyncPolarity", "ActiveHigh", rollback);
    set_enum(map, "FrameSyncSource", "External", rollback);

    std::ofstream manifest(output / "frames.ndjson", std::ios::binary);
    if (!manifest) throw std::runtime_error("cannot create frames.ndjson");
    manifest << "{\"event\":\"configuration\",\"device_id\":\"" << requested_id
             << "\",\"camera_model\":\"A6701\",\"frame_sync_source\":\"External\""
             << ",\"frame_sync_mode\":\"Integration\",\"frame_sync_polarity\":\"ActiveHigh\""
             << ",\"width\":640,\"transport_height\":513,\"image_height\":512"
             << ",\"pixel_format\":\"Mono16\",\"ir_format\":\"Radiometric\"}\n";
    manifest.flush();

    camera->BeginAcquisition();
    acquiring = true;
    std::cout << "ARMED_WAITING_FOR_EXTERNAL_PULSES output=" << output.string() << std::endl;
    while (true) {
      if (fs::exists(stop_request)) {final_reason = "operator_stop"; break;}
      if (!heartbeat_fresh(heartbeat)) {final_reason = "host_heartbeat_timeout"; break;}
      if (fs::space(output).available < 10ULL * 1024ULL * 1024ULL * 1024ULL) {
        final_reason = "free_space_below_10GiB";
        break;
      }
      try {
        auto image = camera->GetNextImage(2000ULL);
        const bool good = !image->IsIncomplete() && image->GetWidth() == 640 &&
          image->GetHeight() == 513 && image->GetStride() == 1280 && image->GetBufferSize() == 656640;
        if (!good) {image->Release(); throw std::runtime_error("incomplete frame or payload mismatch");}
        ++complete;
        char filename[80]{};
        std::snprintf(filename, sizeof(filename), "frame_%08llu_640x513_mono16.raw",
          static_cast<unsigned long long>(complete));
        const fs::path final_path = output / filename;
        const fs::path temporary_path = final_path.string() + ".partial";
        std::ofstream raw(temporary_path, std::ios::binary);
        raw.write(static_cast<const char *>(image->GetData()), static_cast<std::streamsize>(image->GetBufferSize()));
        raw.close();
        if (!raw) {image->Release(); throw std::runtime_error("raw frame write failed");}
        fs::rename(temporary_path, final_path);
        const auto host_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
        const auto hash = hash_payload(image->GetData(), image->GetBufferSize());
        manifest << "{\"event\":\"frame\",\"sample\":" << complete
                 << ",\"frame_id\":" << image->GetFrameID()
                 << ",\"camera_timestamp_ns\":" << image->GetTimeStamp()
                 << ",\"host_receive_monotonic_ns\":" << host_ns
                 << ",\"bytes\":" << image->GetBufferSize()
                 << ",\"fnv1a64\":" << hash << ",\"file\":\"" << filename << "\"}\n";
        manifest.flush();
        image->Release();
        std::cout << "FRAME_READY sample=" << complete << " file=" << filename << std::endl;
      } catch (const Spinnaker::Exception & error) {
        if (error.GetError() == Spinnaker::SPINNAKER_ERR_TIMEOUT) continue;
        throw;
      }
    }
    exit_code = 0;
  } catch (const std::exception & error) {
    final_reason = std::string("error: ") + error.what();
    std::cerr << "FOCUS_CAPTURE_FAILED " << error.what() << std::endl;
  }

  restore();
  camera = nullptr;
  cameras.Clear();
  if (system) {try {system->ReleaseInstance();} catch (...) {}}
  std::ofstream status(output / "capture_complete.json", std::ios::binary);
  status << "{\"complete\":" << (exit_code == 0 ? "true" : "false")
         << ",\"frames\":" << complete << ",\"reason\":\"" << final_reason << "\"}\n";
  status.close();
  std::cout << "FOCUS_CAPTURE_COMPLETE frames=" << complete << " reason=" << final_reason
            << " configuration_restored=true" << std::endl;
  return exit_code;
}
