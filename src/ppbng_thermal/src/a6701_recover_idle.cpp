#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#include <iostream>
#include <stdexcept>
#include <string>

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
  if (argc != 3 || std::string(argv[2]) != "--execute-stop") {
    std::cerr << "usage: ppbng_a6701_recover_idle DEVICE_ID --execute-stop" << std::endl;
    return 64;
  }

  Spinnaker::SystemPtr system;
  Spinnaker::CameraList cameras;
  Spinnaker::CameraPtr camera;
  try {
    const std::string requested_id = argv[1];
    system = Spinnaker::System::GetInstance();
    cameras = system->GetCameras();
    camera = exact_camera(cameras, requested_id);
    if (!camera) throw std::runtime_error("exact DeviceID not found");
    if (value(camera->GetTLDeviceNodeMap(), "DeviceModelName") != "Xsc Series") {
      throw std::runtime_error("transport model is not the expected Xsc Series");
    }

    camera->Init();
    auto & map = camera->GetNodeMap();
    Spinnaker::GenApi::CCommandPtr stop = map.GetNode("AcquisitionStop");
    std::cout << "BEFORE CameraModel='" << value(map, "CameraModel")
              << "' Ready='" << value(map, "Ready")
              << "' AcquisitionStop_writable=" << (Spinnaker::GenApi::IsWritable(stop) ? "true" : "false")
              << std::endl;
    if (!Spinnaker::GenApi::IsWritable(stop)) {
      throw std::runtime_error("AcquisitionStop is unavailable or not writable; no command sent");
    }
    stop->Execute();
    std::cout << "ACQUISITION_STOP_EXECUTED device_id=" << requested_id << std::endl;
    std::cout << "AFTER CameraModel='" << value(map, "CameraModel")
              << "' Ready='" << value(map, "Ready")
              << "' FPACold='" << value(map, "FPACold")
              << "' FrameSyncSource='" << value(map, "FrameSyncSource")
              << "' TriggerMode='" << value(map, "TriggerMode") << "'" << std::endl;

    camera->DeInit();
    camera = nullptr;
    cameras.Clear();
    system->ReleaseInstance();
    return 0;
  } catch (const std::exception & error) {
    try {if (camera && camera->IsInitialized()) camera->DeInit();} catch (...) {}
    camera = nullptr;
    cameras.Clear();
    if (system) {try {system->ReleaseInstance();} catch (...) {}}
    std::cerr << "RECOVER_IDLE_FAILED " << error.what() << std::endl;
    return 1;
  }
}
