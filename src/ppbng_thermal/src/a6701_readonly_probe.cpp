#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#include <iostream>
#include <string>
#include <vector>

namespace
{
std::string escape(const std::string & input)
{
  std::string output;
  for (const unsigned char c : input) {
    if (c == '\\') output += "\\\\";
    else if (c == '"') output += "\\\"";
    else if (c == '\n') output += "\\n";
    else if (c == '\r') output += "\\r";
    else if (c == '\t') output += "\\t";
    else if (c >= 0x20) output += static_cast<char>(c);
  }
  return output;
}

std::string value(Spinnaker::GenApi::INodeMap & map, const char * name)
{
  try {
    Spinnaker::GenApi::CValuePtr node = map.GetNode(name);
    return Spinnaker::GenApi::IsReadable(node) ? std::string(node->ToString().c_str()) : std::string{};
  } catch (...) {
    return {};
  }
}

void emit(const char * name, const std::string & result)
{
  std::cout << "{\"node\":\"" << name << "\",\"readable\":"
            << (result.empty() ? "false" : "true");
  if (!result.empty()) std::cout << ",\"value\":\"" << escape(result) << '"';
  std::cout << '}' << std::endl;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2 || std::string(argv[1]).empty()) {
    std::cerr << "usage: ppbng_a6701_readonly_probe DEVICE_ID" << std::endl;
    return 64;
  }
  const std::string requested_id = argv[1];
  Spinnaker::SystemPtr system;
  Spinnaker::CameraList cameras;
  Spinnaker::CameraPtr selected;
  try {
    system = Spinnaker::System::GetInstance();
    cameras = system->GetCameras();
    for (unsigned int i = 0; i < cameras.GetSize(); ++i) {
      auto candidate = cameras.GetByIndex(i);
      if (value(candidate->GetTLDeviceNodeMap(), "DeviceID") == requested_id) {
        if (selected) throw std::runtime_error("DeviceID is not unique");
        selected = candidate;
      }
    }
    if (!selected) throw std::runtime_error("exact DeviceID not found");

    // SAFETY CONTRACT: exact-device selection precedes Init. This probe reads
    // nodes only: no SetValue(), BeginAcquisition(), or command execution.
    selected->Init();
    auto & map = selected->GetNodeMap();
    std::cout << "{\"event\":\"readonly_probe\",\"device_id\":\""
              << escape(requested_id) << "\"}" << std::endl;
    const std::vector<const char *> nodes{
      "DeviceModelName", "CameraModel", "DeviceSerialNumber", "SensorModel",
      "Width", "Height", "PayloadSize", "PixelFormat", "IRFormat",
      "AcquisitionMode", "AcquisitionFrameRate", "FrameSyncSource",
      "FrameSyncMode", "FrameSyncPolarity", "TriggerMode", "TriggerSource",
      "Ready", "FPACold", "SensorTemperature", "DeviceTemperature",
      "GevCCP", "CorrectionDigitalEnabled", "CorrectionAutoEnabled",
      "CorrectionAutoUseDeltaTemp", "CorrectionAutoDeltaTemp",
      "CorrectionAutoUseDeltaTime", "CorrectionAutoDeltaTime",
      "CorrectionAutoInProgress"
    };
    for (const char * node : nodes) emit(node, value(map, node));

    selected->DeInit();
    selected = nullptr;
    cameras.Clear();
    system->ReleaseInstance();
    return 0;
  } catch (const std::exception & error) {
    try {if (selected && selected->IsInitialized()) selected->DeInit();} catch (...) {}
    selected = nullptr;
    cameras.Clear();
    if (system) {try {system->ReleaseInstance();} catch (...) {}}
    std::cerr << "{\"event\":\"error\",\"message\":\"" << escape(error.what()) << "\"}" << std::endl;
    return 1;
  }
}
