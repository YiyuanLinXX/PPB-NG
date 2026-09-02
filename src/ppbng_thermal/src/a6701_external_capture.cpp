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
#include <vector>

namespace
{
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
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 4) {
    std::cerr << "usage: ppbng_a6701_external_capture DEVICE_ID OUTPUT_DIRECTORY FRAME_COUNT" << std::endl;
    return 64;
  }
  const std::string requested_id = argv[1];
  const std::filesystem::path output = std::filesystem::absolute(argv[2]);
  const unsigned long requested_frames = std::stoul(argv[3]);
  if (requested_frames == 0 || requested_frames > 1000) throw std::runtime_error("FRAME_COUNT must be 1..1000");
  if (std::filesystem::exists(output) || !std::filesystem::create_directories(output)) {
    throw std::runtime_error("output directory must be new and creatable");
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
    if (camera && initialized) {try {camera->DeInit();} catch (...) {} initialized = false;}
  };

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
    const auto camera_model = value(map, "CameraModel");
    const auto ready = value(map, "Ready");
    const auto fpa_cold = value(map, "FPACold");
    if (camera_model != "A6701" || ready != "1" || fpa_cold != "1") {
      throw std::runtime_error(
        "A6701 preflight readback failed: CameraModel='" + camera_model +
        "' Ready='" + ready + "' FPACold='" + fpa_cold + "'");
    }
    if (value(map, "Width") != "640" || value(map, "Height") != "513" ||
      value(map, "PixelFormat") != "Mono16" || value(map, "IRFormat") != "Radiometric" ||
      value(map, "TriggerMode") != "FreeRun") {
      throw std::runtime_error("current image/trigger contract is unsuitable for external-sync test");
    }

    // Program mode and polarity before selecting the external source. Every
    // feature write is read back and restored in reverse order on every exit.
    set_enum(map, "FrameSyncMode", "Integration", rollback);
    set_enum(map, "FrameSyncPolarity", "ActiveHigh", rollback);
    set_enum(map, "FrameSyncSource", "External", rollback);

    std::ofstream manifest(output / "frames.ndjson", std::ios::out | std::ios::binary);
    if (!manifest) throw std::runtime_error("cannot create frames.ndjson");
    manifest << "{\"event\":\"configuration\",\"device_id\":\"" << requested_id
             << "\",\"camera_model\":\"A6701\",\"frame_sync_source\":\"External\""
             << ",\"frame_sync_mode\":\"Integration\",\"frame_sync_polarity\":\"ActiveHigh\""
             << ",\"width\":640,\"height\":513,\"pixel_format\":\"Mono16\",\"ir_format\":\"Radiometric\"}\n";
    manifest.flush();

    camera->BeginAcquisition();
    acquiring = true;
    std::cout << "ARMED_WAITING_FOR_EXTERNAL_PULSES output=" << output.string() << std::endl;
    unsigned long complete = 0;
    for (unsigned long i = 0; i < requested_frames; ++i) {
      auto image = camera->GetNextImage(i == 0 ? 20000ULL : 2000ULL);
      const bool good = !image->IsIncomplete() && image->GetWidth() == 640 &&
        image->GetHeight() == 513 && image->GetStride() == 1280 && image->GetBufferSize() == 656640;
      if (!good) {
        image->Release();
        throw std::runtime_error("incomplete frame or payload contract mismatch");
      }
      const auto host_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      const auto hash = hash_payload(image->GetData(), image->GetBufferSize());
      char filename[64]{};
      std::snprintf(filename, sizeof(filename), "frame_%04lu_640x513_mono16.raw", i + 1);
      std::ofstream raw(output / filename, std::ios::out | std::ios::binary);
      raw.write(static_cast<const char *>(image->GetData()), static_cast<std::streamsize>(image->GetBufferSize()));
      raw.close();
      if (!raw) {image->Release(); throw std::runtime_error("raw frame write failed");}
      manifest << "{\"event\":\"frame\",\"sample\":" << (i + 1)
               << ",\"frame_id\":" << image->GetFrameID() << ",\"camera_timestamp_ns\":"
               << image->GetTimeStamp() << ",\"host_receive_monotonic_ns\":" << host_ns
               << ",\"bytes\":" << image->GetBufferSize() << ",\"fnv1a64\":" << hash
               << ",\"file\":\"" << filename << "\"}\n";
      manifest.flush();
      std::cout << "FRAME " << (i + 1) << '/' << requested_frames << " id=" << image->GetFrameID()
                << " timestamp=" << image->GetTimeStamp() << " bytes=" << image->GetBufferSize() << std::endl;
      ++complete;
      image->Release();
    }
    restore();
    camera = nullptr;
    cameras.Clear();
    system->ReleaseInstance();
    std::cout << "CAPTURE_COMPLETE frames=" << complete << " configuration_restored=true" << std::endl;
    return 0;
  } catch (const std::exception & error) {
    restore();
    camera = nullptr;
    cameras.Clear();
    if (system) {try {system->ReleaseInstance();} catch (...) {}}
    std::cerr << "CAPTURE_FAILED " << error.what() << " configuration_restore_attempted=true" << std::endl;
    return 1;
  }
}
