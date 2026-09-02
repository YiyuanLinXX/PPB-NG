#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
std::string value(Spinnaker::GenApi::INodeMap & map, const char * name)
{
  Spinnaker::GenApi::CValuePtr node = map.GetNode(name);
  return Spinnaker::GenApi::IsReadable(node) ? std::string(node->ToString().c_str()) : std::string{};
}

std::uint64_t sample_hash(const void * data, std::size_t size)
{
  const auto * bytes = static_cast<const unsigned char *>(data);
  std::uint64_t hash = 1469598103934665603ULL;
  for (std::size_t i = 0; i < size; ++i) {
    hash ^= bytes[i];
    hash *= 1099511628211ULL;
  }
  return hash;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: ppbng_a6701_acquisition_probe DEVICE_ID" << std::endl;
    return 64;
  }
  const std::string requested_id = argv[1];
  constexpr unsigned int frame_count = 20;
  Spinnaker::SystemPtr system;
  Spinnaker::CameraList cameras;
  Spinnaker::CameraPtr camera;
  bool acquiring = false;
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
    auto & map = camera->GetNodeMap();
    if (value(map, "CameraModel") != "A6701") throw std::runtime_error("exact device is not A6701");
    if (value(map, "Width") != "640" || value(map, "Height") != "513" ||
      value(map, "PixelFormat") != "Mono16" || value(map, "IRFormat") != "Radiometric" ||
      value(map, "TriggerMode") != "FreeRun" || value(map, "TriggerSource") != "Internal") {
      throw std::runtime_error("current camera configuration is not the safe free-run probe contract");
    }

    // This test intentionally preserves every current feature value.
    camera->BeginAcquisition();
    acquiring = true;
    const auto started = std::chrono::steady_clock::now();
    std::uint64_t first_camera_timestamp = 0;
    std::uint64_t last_camera_timestamp = 0;
    unsigned int complete = 0;
    for (unsigned int i = 0; i < frame_count; ++i) {
      auto image = camera->GetNextImage(2000);
      const bool good = !image->IsIncomplete() && image->GetWidth() == 640 &&
        image->GetHeight() == 513 && image->GetStride() == 1280 && image->GetBufferSize() == 656640;
      const auto timestamp = image->GetTimeStamp();
      if (i == 0) first_camera_timestamp = timestamp;
      last_camera_timestamp = timestamp;
      if (good) ++complete;
      std::cout << "{\"frame\":" << i << ",\"frame_id\":" << image->GetFrameID()
                << ",\"timestamp\":" << timestamp << ",\"complete\":"
                << (good ? "true" : "false") << ",\"width\":" << image->GetWidth()
                << ",\"height\":" << image->GetHeight() << ",\"stride\":" << image->GetStride()
                << ",\"bytes\":" << image->GetBufferSize() << ",\"hash\":"
                << sample_hash(image->GetData(), image->GetBufferSize()) << '}' << std::endl;
      image->Release();
    }
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    camera->EndAcquisition();
    acquiring = false;
    camera->DeInit();
    camera = nullptr;
    cameras.Clear();
    system->ReleaseInstance();
    std::cout << "{\"event\":\"summary\",\"requested\":" << frame_count
              << ",\"complete\":" << complete << ",\"elapsed_seconds\":" << elapsed
              << ",\"host_observed_fps\":" << (frame_count / elapsed)
              << ",\"first_camera_timestamp\":" << first_camera_timestamp
              << ",\"last_camera_timestamp\":" << last_camera_timestamp << "}" << std::endl;
    return complete == frame_count ? 0 : 3;
  } catch (const std::exception & error) {
    if (camera) {
      try {if (acquiring) camera->EndAcquisition();} catch (...) {}
      try {if (camera->IsInitialized()) camera->DeInit();} catch (...) {}
    }
    camera = nullptr;
    cameras.Clear();
    if (system) {try {system->ReleaseInstance();} catch (...) {}}
    std::cerr << "probe error: " << error.what() << std::endl;
    return 1;
  }
}
