#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace
{
std::string json_escape(const std::string & input)
{
  std::string output;
  output.reserve(input.size());
  for (const unsigned char c : input) {
    switch (c) {
      case '\\': output += "\\\\"; break;
      case '"': output += "\\\""; break;
      case '\b': output += "\\b"; break;
      case '\f': output += "\\f"; break;
      case '\n': output += "\\n"; break;
      case '\r': output += "\\r"; break;
      case '\t': output += "\\t"; break;
      default:
        if (c >= 0x20) {
          output += static_cast<char>(c);
        }
    }
  }
  return output;
}

std::string read_value(Spinnaker::GenApi::INodeMap & map, const char * name)
{
  try {
    Spinnaker::GenApi::CValuePtr node = map.GetNode(name);
    if (!Spinnaker::GenApi::IsReadable(node)) {
      return {};
    }
    return node->ToString().c_str();
  } catch (...) {
    return {};
  }
}

void print_field(const char * name, const std::string & value, bool & first)
{
  if (value.empty()) {
    return;
  }
  if (!first) {
    std::cout << ',';
  }
  first = false;
  std::cout << '\"' << name << "\":\"" << json_escape(value) << '\"';
}
}  // namespace

int main()
{
  // SAFETY CONTRACT: this program must remain enumeration-only. In particular,
  // do not add Camera::Init(), BeginAcquisition(), or any SetValue() call here.
  Spinnaker::SystemPtr system;
  Spinnaker::CameraList cameras;
  try {
    system = Spinnaker::System::GetInstance();
    cameras = system->GetCameras();
    const auto count = cameras.GetSize();
    std::cout << "{\"event\":\"spinnaker_inventory\",\"camera_count\":" << count
              << ",\"access\":\"transport_layer_read_only\"}" << std::endl;

    const std::vector<std::pair<const char *, const char *>> fields{
      {"device_id", "DeviceID"},
      {"serial_number", "DeviceSerialNumber"},
      {"model_name", "DeviceModelName"},
      {"vendor_name", "DeviceVendorName"},
      {"device_version", "DeviceVersion"},
      {"display_name", "DeviceDisplayName"},
      {"user_id", "DeviceUserID"},
      {"device_type", "DeviceType"},
      {"access_status", "DeviceAccessStatus"},
      {"ip_address", "GevDeviceIPAddress"},
      {"subnet_mask", "GevDeviceSubnetMask"},
      {"gateway", "GevDeviceGateway"},
      {"mac_address", "GevDeviceMACAddress"}
    };

    for (unsigned int index = 0; index < count; ++index) {
      auto camera = cameras.GetByIndex(index);
      auto & device_map = camera->GetTLDeviceNodeMap();
      std::cout << "{\"event\":\"camera\",\"index\":" << index;
      bool first = false;
      for (const auto & field : fields) {
        print_field(field.first, read_value(device_map, field.second), first);
      }
      std::cout << '}' << std::endl;
    }

    cameras.Clear();
    system->ReleaseInstance();
    return count == 0 ? 2 : 0;
  } catch (const Spinnaker::Exception & error) {
    cameras.Clear();
    if (system) {
      try {system->ReleaseInstance();} catch (...) {}
    }
    std::cerr << "{\"event\":\"error\",\"message\":\""
              << json_escape(error.what() ? error.what() : "Spinnaker exception")
              << "\"}" << std::endl;
    return 1;
  }
}
