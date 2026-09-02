#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
std::string value(Spinnaker::GenApi::INodeMap & map, const char * name)
{
  try {
    Spinnaker::GenApi::CValuePtr node = map.GetNode(name);
    return Spinnaker::GenApi::IsReadable(node) ? std::string(node->ToString().c_str()) : std::string{};
  } catch (...) {
    return {};
  }
}

Spinnaker::CameraPtr exact_camera(Spinnaker::CameraList & cameras, const std::string & id)
{
  Spinnaker::CameraPtr match;
  for (unsigned int i = 0; i < cameras.GetSize(); ++i) {
    auto candidate = cameras.GetByIndex(i);
    if (value(candidate->GetTLDeviceNodeMap(), "DeviceID") == id) {
      if (match) throw std::runtime_error("DeviceID is not unique");
      match = candidate;
    }
  }
  return match;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3 || std::string(argv[2]) != "--execute-software-reboot") {
    std::cerr << "usage: ppbng_a6701_software_reset DEVICE_ID --execute-software-reboot" << std::endl;
    return 64;
  }
  const std::string requested_id = argv[1];
  Spinnaker::SystemPtr system;
  Spinnaker::CameraList cameras;
  Spinnaker::CameraPtr camera;
  try {
    system = Spinnaker::System::GetInstance();
    cameras = system->GetCameras();
    camera = exact_camera(cameras, requested_id);
    if (!camera) throw std::runtime_error("exact DeviceID not found");
    const auto model = value(camera->GetTLDeviceNodeMap(), "DeviceModelName");
    if (model != "Xsc Series") throw std::runtime_error("transport model is not the expected Xsc Series");
    Spinnaker::GenApi::CCommandPtr reset = camera->GetTLDeviceNodeMap().GetNode("DeviceReset");
    if (Spinnaker::GenApi::IsWritable(reset)) {
      std::cout << "SOFTWARE_RESET_EXECUTING path=TLDevice device_id=" << requested_id << std::endl;
      reset->Execute();
    } else {
      camera->Init();
      Spinnaker::GenApi::CCommandPtr device_reset = camera->GetNodeMap().GetNode("DeviceReset");
      if (!Spinnaker::GenApi::IsWritable(device_reset)) {
        camera->DeInit();
        throw std::runtime_error("both TLDevice and initialized DeviceReset are unavailable or not writable");
      }
      std::cout << "SOFTWARE_RESET_EXECUTING path=Device device_id=" << requested_id << std::endl;
      device_reset->Execute();
    }
    camera = nullptr;
    cameras.Clear();
    system->ReleaseInstance();
    system = nullptr;

    for (unsigned int attempt = 1; attempt <= 45; ++attempt) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      try {
        system = Spinnaker::System::GetInstance();
        cameras = system->GetCameras();
        camera = exact_camera(cameras, requested_id);
        if (camera) {
          std::cout << "REDISCOVERED attempt=" << attempt << std::endl;
          camera->Init();
          auto & map = camera->GetNodeMap();
          const auto camera_model = value(map, "CameraModel");
          const auto ready = value(map, "Ready");
          const auto cold = value(map, "FPACold");
          const auto source = value(map, "FrameSyncSource");
          const auto trigger = value(map, "TriggerMode");
          std::cout << "POST_RESET CameraModel='" << camera_model << "' Ready='" << ready
                    << "' FPACold='" << cold << "' FrameSyncSource='" << source
                    << "' TriggerMode='" << trigger << "'" << std::endl;
          camera->DeInit();
          camera = nullptr;
          cameras.Clear();
          system->ReleaseInstance();
          system = nullptr;
          if (camera_model == "A6701" && ready == "1" && cold == "1" &&
            !source.empty() && !trigger.empty()) {
            std::cout << "SOFTWARE_RESET_RECOVERY_CONFIRMED" << std::endl;
            return 0;
          }
        }
        camera = nullptr;
        cameras.Clear();
        system->ReleaseInstance();
        system = nullptr;
      } catch (...) {
        camera = nullptr;
        cameras.Clear();
        if (system) {try {system->ReleaseInstance();} catch (...) {}}
        system = nullptr;
      }
    }
    throw std::runtime_error("device did not return with valid core readback within 45 seconds");
  } catch (const std::exception & error) {
    try {if (camera && camera->IsInitialized()) camera->DeInit();} catch (...) {}
    camera = nullptr;
    cameras.Clear();
    if (system) {try {system->ReleaseInstance();} catch (...) {}}
    std::cerr << "SOFTWARE_RESET_FAILED " << error.what() << std::endl;
    return 1;
  }
}
