#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>
#include <ppbng_interfaces/msg/frame_metadata.hpp>
#include <rclcpp/rclcpp.hpp>

#ifdef _WIN32
#include <windows.h>
#endif

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <mutex>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
namespace fs = std::filesystem;
using namespace std::chrono_literals;
using Spinnaker::GenApi::CBooleanPtr;
using Spinnaker::GenApi::CEnumEntryPtr;
using Spinnaker::GenApi::CEnumerationPtr;
using Spinnaker::GenApi::CFloatPtr;
using Spinnaker::GenApi::CIntegerPtr;
using Spinnaker::GenApi::CValuePtr;
using Spinnaker::GenApi::INodeMap;
using Spinnaker::GenApi::IsReadable;
using Spinnaker::GenApi::IsWritable;

std::uint64_t monotonic_ns()
{
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
}

std::string json_quote(const std::string & input)
{
  std::ostringstream out;
  out << '"';
  for (const unsigned char c : input) {
    if (c == '"' || c == '\\') out << '\\' << static_cast<char>(c);
    else if (c == '\n') out << "\\n";
    else if (c == '\r') out << "\\r";
    else if (c == '\t') out << "\\t";
    else if (c < 0x20) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c);
    else out << static_cast<char>(c);
  }
  out << '"';
  return out.str();
}

std::string node_value(INodeMap & map, const char * name)
{
  CValuePtr node = map.GetNode(name);
  return IsReadable(node) ? std::string(node->ToString().c_str()) : std::string("UNAVAILABLE");
}

void set_enum(
  INodeMap & map, const char * name, const char * target,
  std::vector<std::function<void()>> & rollback)
{
  CEnumerationPtr node = map.GetNode(name);
  if (!IsReadable(node) || !IsWritable(node)) throw std::runtime_error(std::string(name) + " is not writable");
  CEnumEntryPtr old_entry = node->GetCurrentEntry();
  CEnumEntryPtr new_entry = node->GetEntryByName(target);
  if (!IsReadable(old_entry) || !IsReadable(new_entry)) throw std::runtime_error(std::string(name) + " entry unavailable");
  const auto old_value = old_entry->GetValue();
  node->SetIntValue(new_entry->GetValue());
  rollback.emplace_back([node, old_value]() {try {node->SetIntValue(old_value);} catch (...) {}});
  if (node_value(map, name) != target) throw std::runtime_error(std::string(name) + " readback mismatch");
}

void set_integer(
  INodeMap & map, const char * name, std::int64_t target,
  std::vector<std::function<void()>> & rollback)
{
  CIntegerPtr node = map.GetNode(name);
  if (!IsReadable(node) || !IsWritable(node)) {
    throw std::runtime_error(std::string(name) + " is not writable");
  }
  const auto old_value = node->GetValue();
  node->SetValue(target);
  rollback.emplace_back([node, old_value]() {try {node->SetValue(old_value);} catch (...) {}});
  if (node->GetValue() != target) throw std::runtime_error(std::string(name) + " readback mismatch");
}

void set_boolean(
  INodeMap & map, const char * name, bool target,
  std::vector<std::function<void()>> & rollback)
{
  CBooleanPtr node = map.GetNode(name);
  if (!IsReadable(node) || !IsWritable(node)) {
    throw std::runtime_error(std::string(name) + " is not writable");
  }
  const bool old_value = node->GetValue();
  node->SetValue(target);
  rollback.emplace_back([node, old_value]() {try {node->SetValue(old_value);} catch (...) {}});
  if (node->GetValue() != target) throw std::runtime_error(std::string(name) + " readback mismatch");
}

void set_float(
  INodeMap & map, const char * name, double target,
  std::vector<std::function<void()>> & rollback)
{
  CFloatPtr node = map.GetNode(name);
  if (!IsReadable(node) || !IsWritable(node) || target < node->GetMin() || target > node->GetMax()) {
    throw std::runtime_error(std::string(name) + " is not writable or target is out of range");
  }
  const double old_value = node->GetValue();
  node->SetValue(target);
  rollback.emplace_back([node, old_value]() {try {node->SetValue(old_value);} catch (...) {}});
}

constexpr const char * rgb_required_chunks[] = {
  "FrameID", "Timestamp", "ExposureTime", "Gain", "BlackLevel"};

void configure_rgb_chunks(
  INodeMap & map, std::vector<std::function<void()>> & rollback)
{
  set_boolean(map, "ChunkModeActive", true, rollback);
  CEnumerationPtr selector = map.GetNode("ChunkSelector");
  if (!IsReadable(selector) || !IsWritable(selector) || !IsReadable(selector->GetCurrentEntry())) {
    throw std::runtime_error("RGB ChunkSelector is not readable/writable");
  }
  const auto original_selector = selector->GetIntValue();
  rollback.emplace_back([selector, original_selector]() {
    try {selector->SetIntValue(original_selector);} catch (...) {}
  });
  for (const char * name : rgb_required_chunks) {
    CEnumEntryPtr entry = selector->GetEntryByName(name);
    if (!IsReadable(entry)) throw std::runtime_error(std::string("required RGB chunk unavailable: ") + name);
    selector->SetIntValue(entry->GetValue());
    CBooleanPtr enabled = map.GetNode("ChunkEnable");
    if (!IsReadable(enabled) || !IsWritable(enabled)) {
      throw std::runtime_error(std::string("required RGB chunk is not writable: ") + name);
    }
    const bool old_value = enabled->GetValue();
    enabled->SetValue(true);
    const auto selected_value = entry->GetValue();
    rollback.emplace_back([selector, enabled, selected_value, old_value]() {
      try {selector->SetIntValue(selected_value); enabled->SetValue(old_value);} catch (...) {}
    });
    if (!enabled->GetValue()) throw std::runtime_error(std::string("RGB chunk readback failed: ") + name);
  }
  selector->SetIntValue(original_selector);
}

bool read_balance_ratio(INodeMap & map, const char * channel, double & value)
{
  CEnumerationPtr selector = map.GetNode("BalanceRatioSelector");
  CFloatPtr ratio = map.GetNode("BalanceRatio");
  if (!IsReadable(selector) || !IsWritable(selector) || !IsReadable(selector->GetCurrentEntry()) ||
    !IsReadable(ratio)) return false;
  const auto original = selector->GetIntValue();
  try {
    CEnumEntryPtr entry = selector->GetEntryByName(channel);
    if (!IsReadable(entry)) return false;
    selector->SetIntValue(entry->GetValue());
    value = ratio->GetValue();
    selector->SetIntValue(original);
    return true;
  } catch (...) {
    try {selector->SetIntValue(original);} catch (...) {}
    return false;
  }
}

struct RgbFrameSettings
{
  std::uint64_t chunk_frame_id{0},chunk_timestamp{0};
  double exposure_time_us{0.0},gain_db{0.0},black_level{0.0};
  bool white_balance_red_valid{false},white_balance_blue_valid{false};
  double white_balance_red{0.0},white_balance_blue{0.0};
  std::string exposure_auto,gain_auto,balance_white_auto;
};

std::string session_stamp()
{
  const auto now = std::chrono::system_clock::now();
  const auto value = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
#ifdef _WIN32
  localtime_s(&local, &value);
#else
  localtime_r(&value, &local);
#endif
  std::ostringstream out;
  out << std::put_time(&local, "%Y%m%d_%H%M%S");
  return out.str();
}

#ifdef _WIN32
class SerialPort
{
public:
  ~SerialPort() {close();}
  void open(const std::string & port)
  {
    const std::string path = "\\\\.\\" + port;
    handle_ = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (handle_ == INVALID_HANDLE_VALUE) throw std::runtime_error("cannot open Arduino port " + port);
    DCB dcb{}; dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(handle_, &dcb)) throw std::runtime_error("GetCommState failed");
    dcb.BaudRate = CBR_115200; dcb.ByteSize = 8; dcb.Parity = NOPARITY; dcb.StopBits = ONESTOPBIT;
    dcb.fDtrControl = DTR_CONTROL_ENABLE; dcb.fRtsControl = RTS_CONTROL_ENABLE;
    if (!SetCommState(handle_, &dcb)) throw std::runtime_error("SetCommState failed");
    COMMTIMEOUTS timeouts{}; timeouts.ReadIntervalTimeout = MAXDWORD;
    if (!SetCommTimeouts(handle_, &timeouts)) throw std::runtime_error("SetCommTimeouts failed");
    PurgeComm(handle_, PURGE_RXCLEAR | PURGE_TXCLEAR);
  }
  void close() noexcept
  {
    if (handle_ != INVALID_HANDLE_VALUE) {CloseHandle(handle_); handle_ = INVALID_HANDLE_VALUE;}
  }
  void write_line(const std::string & line)
  {
    const std::string payload = line + "\n"; DWORD written = 0;
    if (!WriteFile(handle_, payload.data(), static_cast<DWORD>(payload.size()), &written, nullptr) ||
      written != payload.size()) throw std::runtime_error("Arduino serial write failed");
  }
  std::string read_available()
  {
    COMSTAT stat{}; DWORD errors = 0;
    if (!ClearCommError(handle_, &errors, &stat)) throw std::runtime_error("Arduino serial status failed");
    if (stat.cbInQue == 0) return {};
    std::string result(stat.cbInQue, '\0'); DWORD count = 0;
    if (!ReadFile(handle_, result.data(), stat.cbInQue, &count, nullptr)) throw std::runtime_error("Arduino serial read failed");
    result.resize(count); return result;
  }
private:
  HANDLE handle_{INVALID_HANDLE_VALUE};
};
#else
class SerialPort {public: void open(const std::string &) {throw std::runtime_error("UNO R4 test is Windows-only");}};
#endif

class A6701UnoR4TestNode final : public rclcpp::Node
{
public:
  A6701UnoR4TestNode() : Node("ppbng_a6701_uno_r4_test")
  {
    dataset_name_ = declare_parameter<std::string>("dataset_name", "");
    device_id_ = declare_parameter<std::string>("device_id", "00111C0408CD");
    serial_port_ = declare_parameter<std::string>("serial_port", "COM9");
    output_root_ = declare_parameter<std::string>("output_root", "hardware_test_data");
    duration_sec_ = declare_parameter<double>("duration_sec", 0.0);
    fault_test_ = declare_parameter<bool>("fault_recovery_test", false);
    rgb_enabled_ = declare_parameter<bool>("rgb_enabled", false);
    rgb_device_id_ = declare_parameter<std::string>("rgb_device_id", "22209867");
    rgb_exposure_auto_ = declare_parameter<std::string>("rgb_exposure_auto", "Continuous");
    rgb_gain_auto_ = declare_parameter<std::string>("rgb_gain_auto", "Continuous");
    rgb_balance_white_auto_ = declare_parameter<std::string>("rgb_balance_white_auto", "Once");
    rgb_balance_white_auto_profile_ =
      declare_parameter<std::string>("rgb_balance_white_auto_profile", "Outdoor");
    rgb_auto_exposure_control_priority_ =
      declare_parameter<std::string>("rgb_auto_exposure_control_priority", "Gain");
    rgb_auto_exposure_time_upper_limit_us_ =
      declare_parameter<double>("rgb_auto_exposure_time_upper_limit_us", 5000.0);
    rgb_auto_exposure_gain_upper_limit_db_ =
      declare_parameter<double>("rgb_auto_exposure_gain_upper_limit_db", 12.0);
    if (!std::regex_match(dataset_name_, std::regex("[A-Za-z0-9_-]{1,48}"))) {
      throw std::invalid_argument("dataset_name must contain 1-48 letters, numbers, '_' or '-'");
    }
    publisher_ = create_publisher<ppbng_interfaces::msg::FrameMetadata>("~/frame_metadata", 32);
    rgb_publisher_ = create_publisher<ppbng_interfaces::msg::FrameMetadata>("~/rgb_frame_metadata", 32);
    worker_ = std::thread([this]() {run();});
  }

  ~A6701UnoR4TestNode() override
  {
    stop_.store(true);
    if (worker_.joinable()) worker_.join();
  }

private:
  void drain_serial(SerialPort & serial, std::ofstream & events)
  {
#ifdef _WIN32
    serial_buffer_ += serial.read_available();
    std::size_t newline = 0;
    while ((newline = serial_buffer_.find_first_of("\r\n")) != std::string::npos) {
      const std::string line = serial_buffer_.substr(0, newline);
      serial_buffer_.erase(0, newline + 1);
      while (!serial_buffer_.empty() && (serial_buffer_.front() == '\r' || serial_buffer_.front() == '\n')) serial_buffer_.erase(0, 1);
      if (line.empty()) continue;
      events << "{\"host_monotonic_ns\":" << monotonic_ns() << ",\"line\":" << json_quote(line) << "}\n";
      events.flush();
      RCLCPP_INFO(get_logger(), "UNO R4: %s", line.c_str());
      if (line.rfind("PULSE,", 0) == 0) {
        std::istringstream fields(line.substr(6));
        std::uint64_t seq = 0, tick = 0; char comma = '\0';
        if ((fields >> seq >> comma >> tick) && comma == ',') {
          last_pulse_sequence_ = seq;
          last_pulse_tick_ = tick;
        }
      }
      if (line == "STOPPED,host_watchdog_timeout") watchdog_stop_seen_ = true;
    }
#else
    (void)serial; (void)events;
#endif
  }

  void write_calibration_metadata(INodeMap & map, const fs::path & output)
  {
    const std::vector<const char *> keys = {
      "CameraModel", "ActivePreset", "PS0CalibrationTag", "CalibrationIsFactoryCalibrated",
      "CalibrationQueryIndex", "CalibrationQueryIndexMax", "CalibrationQueryLens",
      "CalibrationQueryLensFilter", "CalibrationQueryName", "CalibrationQueryTag",
      "CalibrationQueryFilterID", "CalibrationQueryCorrection", "CalibrationQueryMinCounts",
      "CalibrationQueryMaxCounts", "CalibrationQueryMinTemp", "CalibrationQueryMaxTemp",
      "CalibrationQueryMinRadiance", "CalibrationQueryMaxRadiance", "CalibrationQueryLUTSize",
      "CalibrationQueryOrder", "CalibrationQueryCoeff0", "CalibrationQueryCoeff1",
      "CalibrationQueryCoeff2", "CalibrationQueryCoeff3", "CalibrationQueryCoeff4",
      "CalibrationQueryCoeff5", "CalibrationQueryCoeff6", "CalibrationQueryBackgroundValue",
      "CalibrationQueryBandpassLow", "CalibrationQueryBahdpassHigh", "CalibrationQueryTempOrder",
      "CalibrationQueryTempCoeff0", "CalibrationQueryTempCoeff1", "CalibrationQueryTempCoeff2",
      "CalibrationQueryTempCoeff3", "CalibrationQueryTempCoeff4", "CalibrationQueryTempCoeff5",
      "CalibrationQueryTempCoeff6", "CalibrationQueryR", "CalibrationQueryB", "CalibrationQueryF",
      "CalibrationQueryA1", "CalibrationQueryA2", "CalibrationQueryB1", "CalibrationQueryB2",
      "CalibrationQueryX", "ObjectEmissivity", "ReflectedTemperature",
      "AtmosphericTemperature", "ObjectDistance", "RelativeHumidity", "EstimatedTransmission",
      "ExtOpticsTemperature", "ExtOpticsTransmission", "DeviceTemperatureSelector",
      "DeviceTemperature", "FPACold", "Ready", "GigeHeaderDisable", "Height", "ImageHeight",
      "PixelFormat", "IRFormat", "CorrectionAutoEnabled", "CorrectionAutoUseDeltaTemp",
      "CorrectionAutoDeltaTemp", "CorrectionAutoUseDeltaTime", "CorrectionAutoDeltaTime",
      "CorrectionAutoInProgress"};
    std::ofstream file(output / "calibration_metadata.json", std::ios::binary);
    file << "{\n  \"captured_before_acquisition\": true,\n  \"values\": {\n";
    for (std::size_t i = 0; i < keys.size(); ++i) {
      file << "    " << json_quote(keys[i]) << ": " << json_quote(node_value(map, keys[i]));
      file << (i + 1 == keys.size() ? "\n" : ",\n");
    }
    file << "  },\n  \"temperature_semantics\": "
         << json_quote("DeviceTemperature uses DeviceTemperatureSelector; FPA near -196 C is expected for a nominal 77 K cooled detector")
         << ",\n  \"transport_semantics\": "
         << json_quote("Height includes the GigE header; ImageHeight excludes it; full payload is preserved") << "\n}\n";
  }

  void write_rgb_metadata(INodeMap & map, const fs::path & output)
  {
    const std::vector<const char *> keys = {
      "DeviceModelName", "DeviceSerialNumber", "AcquisitionMode", "Width", "Height",
      "PayloadSize", "PixelFormat", "TriggerMode", "TriggerSelector", "TriggerSource",
      "TriggerActivation", "ExposureAuto", "ExposureTime", "GainAuto", "Gain",
      "BalanceWhiteAuto", "BalanceWhiteAutoProfile", "AutoExposureControlPriority",
      "AutoExposureExposureTimeUpperLimit", "AutoExposureGainUpperLimit", "ChunkModeActive"};
    std::ofstream file(output / "camera_metadata.json", std::ios::binary);
    file << "{\n  \"captured_before_acquisition\": true,\n  \"values\": {\n";
    for (std::size_t i = 0; i < keys.size(); ++i) {
      file << "    " << json_quote(keys[i]) << ": " << json_quote(node_value(map, keys[i]));
      file << (i + 1 == keys.size() ? "\n" : ",\n");
    }
    file << "  },\n  \"raw_semantics\": "
         << json_quote("4096x3000 BayerRG8 is preserved without debayering or color conversion")
         << "\n}\n";
  }

  void publish_frame(std::uint64_t sample, Spinnaker::ImagePtr image, std::uint64_t host_ns)
  {
    ppbng_interfaces::msg::FrameMetadata message;
    message.stamp.session_id = dataset_name_;
    message.stamp.device_id = device_id_;
    message.stamp.sample_sequence = sample;
    message.stamp.trigger_channel = "uno_r4_d12_thermal_sync";
    message.stamp.trigger_sequence = last_pulse_sequence_;
    message.stamp.controller_tick = last_pulse_tick_;
    message.stamp.controller_ticks_per_second = 1000000;
    message.stamp.time_quality.status = message.stamp.time_quality.UNSYNCED;
    message.stamp.time_quality.hardware_tick = last_pulse_tick_;
    message.stamp.time_quality.detail = "UNO R4 pulse and camera timestamps recorded; UTC unavailable because no GPS/PPS source is present";
    message.stamp.camera_frame_id_valid = true;
    message.stamp.camera_frame_id = image->GetFrameID();
    message.stamp.camera_timestamp_valid = true;
    message.stamp.camera_timestamp = image->GetTimeStamp();
    message.stamp.host_receive_monotonic_ns = host_ns;
    message.transport_width = 640; message.transport_height = 513; message.row_stride_bytes = 1280;
    message.payload_size_bytes = 656640; message.pixel_format = "Mono16";
    message.byte_order = message.BYTE_ORDER_LITTLE_ENDIAN;
    message.image_x = 0; message.image_y = 1; message.image_width = 640; message.image_height = 512;
    message.image_data_offset_bytes = 1280; message.image_data_length_bytes = 655360;
    message.auxiliary_row_count = 1; message.auxiliary_data_offset_bytes = 0;
    message.auxiliary_data_length_bytes = 1280; message.auxiliary_type = "FLIR_GIGE_IMAGE_HEADER";
    message.auxiliary_schema_version = "vendor-opaque";
    message.geometry_source = message.GEOMETRY_SOURCE_VENDOR_DOCUMENTED;
    message.raw_payload_preserved = true; message.complete = true;
    message.validation_detail = "640x513 transport: first row GigE header, following 640x512 radiometric image";
    publisher_->publish(message);
  }

  void publish_rgb_frame(
    std::uint64_t sample, Spinnaker::ImagePtr image, std::uint64_t host_ns,
    const RgbFrameSettings & settings)
  {
    ppbng_interfaces::msg::FrameMetadata message;
    message.stamp.session_id = dataset_name_;
    message.stamp.device_id = rgb_device_id_;
    message.stamp.sample_sequence = sample;
    message.stamp.trigger_channel = "uno_r4_d11_rgb_trigger";
    message.stamp.trigger_sequence = last_pulse_sequence_;
    message.stamp.controller_tick = last_pulse_tick_;
    message.stamp.controller_ticks_per_second = 1000000;
    message.stamp.time_quality.status = message.stamp.time_quality.UNSYNCED;
    message.stamp.time_quality.hardware_tick = last_pulse_tick_;
    message.stamp.time_quality.detail =
      "same atomic UNO R4 Port 4 edge as D12 thermal; UTC unavailable because GPS/PPS is absent";
    message.stamp.camera_frame_id_valid = true;
    message.stamp.camera_frame_id = image->GetFrameID();
    message.stamp.camera_timestamp_valid = true;
    message.stamp.camera_timestamp = image->GetTimeStamp();
    message.stamp.host_receive_monotonic_ns = host_ns;
    message.transport_width = 4096; message.transport_height = 3000;
    message.row_stride_bytes = 4096; message.payload_size_bytes = 12288000;
    message.pixel_format = "BayerRG8"; message.byte_order = message.BYTE_ORDER_UNKNOWN;
    message.image_x = 0; message.image_y = 0; message.image_width = 4096; message.image_height = 3000;
    message.image_data_offset_bytes = 0; message.image_data_length_bytes = 12288000;
    message.auxiliary_row_count = 0; message.auxiliary_data_offset_bytes = 0;
    message.auxiliary_data_length_bytes = 0; message.auxiliary_type = "NONE";
    message.geometry_source = message.GEOMETRY_SOURCE_CAMERA_READBACK;
    message.raw_payload_preserved = true; message.complete = true;
    message.validation_detail = "4096x3000 BayerRG8 packed raw payload";
    message.chunk_data_valid = true;
    message.chunk_frame_id_valid = true; message.chunk_frame_id = settings.chunk_frame_id;
    message.chunk_timestamp_valid = true; message.chunk_timestamp = settings.chunk_timestamp;
    message.exposure_time_valid = true; message.exposure_time_us = settings.exposure_time_us;
    message.gain_valid = true; message.gain_db = settings.gain_db;
    message.black_level_valid = true; message.black_level = settings.black_level;
    message.white_balance_red_valid = settings.white_balance_red_valid;
    message.white_balance_red = settings.white_balance_red;
    message.white_balance_blue_valid = settings.white_balance_blue_valid;
    message.white_balance_blue = settings.white_balance_blue;
    message.exposure_auto = settings.exposure_auto;
    message.gain_auto = settings.gain_auto;
    message.balance_white_auto = settings.balance_white_auto;
    message.settings_source = "SPINNAKER_CHUNK_PLUS_POST_FRAME_WHITE_BALANCE_READBACK";
    rgb_publisher_->publish(message);
  }

  void run()
  {
    fs::path output, thermal_output, rgb_output;
    Spinnaker::SystemPtr system;
    Spinnaker::CameraList cameras;
    Spinnaker::CameraPtr camera, rgb_camera;
    bool initialized = false, acquiring = false;
    bool rgb_initialized = false, rgb_acquiring = false, arduino_running = false;
    SerialPort serial;
    std::vector<std::function<void()>> rollback, rgb_rollback;
    std::ofstream manifest, rgb_manifest, pair_manifest, serial_events;
    std::uint64_t complete = 0, rgb_complete = 0, paired_complete = 0;
    std::string reason = "unknown";
    int result = 1;
    auto cleanup = [&]() {
#ifdef _WIN32
      try {serial.write_line("STOP"); std::this_thread::sleep_for(100ms); if (serial_events) drain_serial(serial, serial_events);} catch (...) {}
#endif
      arduino_running = false;
      if (rgb_camera && rgb_acquiring) {try {rgb_camera->EndAcquisition();} catch (...) {} rgb_acquiring = false;}
      if (camera && acquiring) {try {camera->EndAcquisition();} catch (...) {} acquiring = false;}
      for (auto it = rgb_rollback.rbegin(); it != rgb_rollback.rend(); ++it) (*it)();
      rgb_rollback.clear();
      for (auto it = rollback.rbegin(); it != rollback.rend(); ++it) (*it)();
      rollback.clear();
      if (rgb_camera && rgb_initialized) {try {rgb_camera->DeInit();} catch (...) {} rgb_initialized = false;}
      if (camera && initialized) {try {camera->DeInit();} catch (...) {} initialized = false;}
      rgb_camera = nullptr; camera = nullptr; cameras.Clear();
      if (system) {try {system->ReleaseInstance();} catch (...) {}}
    };

    try {
      const fs::path root = fs::absolute(output_root_);
      if (!fs::is_directory(root)) throw std::runtime_error("output_root must already exist");
      output = root / ((rgb_enabled_ ? "rgb_thermal_ros2_" : "thermal_ros2_") + dataset_name_ + "_" + session_stamp());
      if (fs::exists(output) || !fs::create_directory(output)) throw std::runtime_error("new output directory could not be created");
      thermal_output = rgb_enabled_ ? output / "thermal" : output;
      rgb_output = output / "rgb";
      if (rgb_enabled_ && (!fs::create_directory(thermal_output) || !fs::create_directory(rgb_output))) {
        throw std::runtime_error("synchronized camera subdirectories could not be created");
      }
      manifest.open(thermal_output / "frames.ndjson", std::ios::binary);
      if (rgb_enabled_) {
        rgb_manifest.open(rgb_output / "frames.ndjson", std::ios::binary);
        pair_manifest.open(output / "synchronized_pairs.ndjson", std::ios::binary);
      }
      serial_events.open(output / "arduino_events.ndjson", std::ios::binary);
      if (!manifest || !serial_events || (rgb_enabled_ && (!rgb_manifest || !pair_manifest))) {
        throw std::runtime_error("metadata files could not be created");
      }

      serial.open(serial_port_);
      std::this_thread::sleep_for(2s);
#ifdef _WIN32
      drain_serial(serial, serial_events);
      serial.write_line("STOP"); std::this_thread::sleep_for(200ms); drain_serial(serial, serial_events);
      serial.write_line("STATUS"); std::this_thread::sleep_for(300ms); drain_serial(serial, serial_events);
#endif
      system = Spinnaker::System::GetInstance(); cameras = system->GetCameras();
      for (unsigned i = 0; i < cameras.GetSize(); ++i) {
        auto candidate = cameras.GetByIndex(i);
        auto & tl_map = candidate->GetTLDeviceNodeMap();
        if (node_value(tl_map, "DeviceID") == device_id_) {
          if (camera) throw std::runtime_error("DeviceID is not unique");
          camera = candidate;
        }
        if (rgb_enabled_ && node_value(tl_map, "DeviceSerialNumber") == rgb_device_id_) {
          if (rgb_camera) throw std::runtime_error("RGB DeviceSerialNumber is not unique");
          rgb_camera = candidate;
        }
      }
      if (!camera) throw std::runtime_error("exact A6701 DeviceID not found");
      if (rgb_enabled_ && !rgb_camera) throw std::runtime_error("exact RGB DeviceSerialNumber not found");
      if (rgb_enabled_ && rgb_camera == camera) throw std::runtime_error("thermal and RGB identities selected the same camera");
      camera->Init(); initialized = true;
      auto & map = camera->GetNodeMap();
      if (node_value(map, "CameraModel") != "A6701" || node_value(map, "Ready") != "1" || node_value(map, "FPACold") != "1") {
        throw std::runtime_error("A6701 identity/Ready/FPACold preflight failed");
      }
      if (node_value(map, "Width") != "640" || node_value(map, "Height") != "513" ||
        node_value(map, "ImageHeight") != "512" || node_value(map, "PixelFormat") != "Mono16" ||
        node_value(map, "IRFormat") != "Radiometric") throw std::runtime_error("A6701 transport/radiometric contract mismatch");
      write_calibration_metadata(map, thermal_output);
      set_enum(map, "FrameSyncMode", "Integration", rollback);
      set_enum(map, "FrameSyncPolarity", "ActiveHigh", rollback);
      set_enum(map, "FrameSyncSource", "External", rollback);

      if (rgb_enabled_) {
        rgb_camera->Init(); rgb_initialized = true;
        auto & rgb_map = rgb_camera->GetNodeMap();
        const auto model = node_value(rgb_camera->GetTLDeviceNodeMap(), "DeviceModelName");
        if (model.find("Blackfly S BFS-U3-123S6") == std::string::npos) {
          throw std::runtime_error("RGB model mismatch: " + model);
        }
        set_enum(rgb_map, "TriggerMode", "Off", rgb_rollback);
        set_enum(rgb_map, "AcquisitionMode", "Continuous", rgb_rollback);
        set_enum(rgb_map, "PixelFormat", "BayerRG8", rgb_rollback);
        set_integer(rgb_map, "Width", 4096, rgb_rollback);
        set_integer(rgb_map, "Height", 3000, rgb_rollback);
        set_enum(rgb_map, "TriggerSelector", "FrameStart", rgb_rollback);
        set_enum(rgb_map, "TriggerSource", "Line0", rgb_rollback);
        set_enum(rgb_map, "TriggerActivation", "RisingEdge", rgb_rollback);
        set_enum(rgb_map, "BalanceWhiteAutoProfile", rgb_balance_white_auto_profile_.c_str(), rgb_rollback);
        set_enum(
          rgb_map, "AutoExposureControlPriority",
          rgb_auto_exposure_control_priority_.c_str(), rgb_rollback);
        set_float(
          rgb_map, "AutoExposureExposureTimeUpperLimit",
          rgb_auto_exposure_time_upper_limit_us_, rgb_rollback);
        set_float(
          rgb_map, "AutoExposureGainUpperLimit",
          rgb_auto_exposure_gain_upper_limit_db_, rgb_rollback);
        set_enum(rgb_map, "ExposureAuto", rgb_exposure_auto_.c_str(), rgb_rollback);
        set_enum(rgb_map, "GainAuto", rgb_gain_auto_.c_str(), rgb_rollback);
        set_enum(rgb_map, "BalanceWhiteAuto", rgb_balance_white_auto_.c_str(), rgb_rollback);
        configure_rgb_chunks(rgb_map, rgb_rollback);
        set_enum(rgb_map, "TriggerMode", "On", rgb_rollback);
        if (node_value(rgb_map, "Width") != "4096" || node_value(rgb_map, "Height") != "3000" ||
          std::stoull(node_value(rgb_map, "PayloadSize")) < 12288000ULL ||
          node_value(rgb_map, "PixelFormat") != "BayerRG8" ||
          node_value(rgb_map, "TriggerMode") != "On" || node_value(rgb_map, "TriggerSource") != "Line0" ||
          node_value(rgb_map, "TriggerActivation") != "RisingEdge" ||
          node_value(rgb_map, "ExposureAuto") != rgb_exposure_auto_ ||
          node_value(rgb_map, "GainAuto") != rgb_gain_auto_ ||
          node_value(rgb_map, "BalanceWhiteAuto") != rgb_balance_white_auto_ ||
          node_value(rgb_map, "BalanceWhiteAutoProfile") != rgb_balance_white_auto_profile_ ||
          node_value(rgb_map, "AutoExposureControlPriority") !=
          rgb_auto_exposure_control_priority_ ||
          node_value(rgb_map, "ChunkModeActive") != "1") {
          throw std::runtime_error("RGB external-trigger/raw-Bayer readback mismatch");
        }
        write_rgb_metadata(rgb_map, rgb_output);
      }
      if (rgb_enabled_) {rgb_camera->BeginAcquisition(); rgb_acquiring = true;}
      camera->BeginAcquisition(); acquiring = true;
#ifdef _WIN32
      serial.write_line("START"); arduino_running = true;
#endif
      RCLCPP_INFO(
        get_logger(), "ROS 2 %s + UNO R4 test running; output=%s",
        rgb_enabled_ ? "synchronized RGB/A6701" : "A6701", output.string().c_str());
      RCLCPP_INFO(get_logger(), "Press Ctrl+C for an orderly stop (UNO STOP is sent before camera EndAcquisition)");
      const auto started = std::chrono::steady_clock::now();
      auto last_keepalive = started - 2s;
      bool pause_started = false, pause_restarted = false, watchdog_phase = false, watchdog_restarted = false;
      std::uint64_t recovery_frame_baseline = 0;
      std::uint64_t last_paired_pulse = 0;
      std::vector<std::uint64_t> camera_times;
      while (!stop_.load() && rclcpp::ok()) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - started).count();
        if (duration_sec_ > 0.0 && elapsed >= duration_sec_) {reason = "duration_complete"; result = 0; break;}
#ifdef _WIN32
        drain_serial(serial, serial_events);
        if (fault_test_) {
          if (!pause_started && elapsed >= 4.0) {serial.write_line("STOP"); pause_started = true; recovery_frame_baseline = complete; RCLCPP_WARN(get_logger(), "FAULT TEST: commanded 4-second trigger pause");}
          if (pause_started && !pause_restarted && elapsed >= 8.0) {serial.write_line("START"); pause_restarted = true; last_keepalive = now; RCLCPP_WARN(get_logger(), "FAULT TEST: trigger restarted without reopening camera");}
          if (pause_restarted && !watchdog_phase && elapsed >= 12.0) {watchdog_phase = true; RCLCPP_WARN(get_logger(), "FAULT TEST: withholding keepalive to exercise UNO watchdog");}
          if (watchdog_phase && watchdog_stop_seen_ && !watchdog_restarted) {serial.write_line("START"); watchdog_restarted = true; last_keepalive = now; recovery_frame_baseline = complete; RCLCPP_WARN(get_logger(), "FAULT TEST: watchdog stop observed; trigger restarted");}
        }
        const bool withhold = fault_test_ && watchdog_phase && !watchdog_stop_seen_;
        if (!withhold && now - last_keepalive >= 800ms) {serial.write_line("KEEPALIVE"); last_keepalive = now;}
#endif
        try {
          auto image = camera->GetNextImage(250ULL);
          const auto host_ns = monotonic_ns();
          if (!image) throw std::runtime_error("Spinnaker returned null image");
          const bool good = !image->IsIncomplete() && image->GetWidth() == 640 && image->GetHeight() == 513 &&
            image->GetStride() == 1280 && image->GetBufferSize() == 656640;
          if (!good) {image->Release(); throw std::runtime_error("incomplete frame or payload mismatch");}
          Spinnaker::ImagePtr rgb_image;
          std::uint64_t rgb_host_ns = 0;
          if (rgb_enabled_) {
            try {
              rgb_image = rgb_camera->GetNextImage(250ULL);
              rgb_host_ns = monotonic_ns();
              if (!rgb_image) throw std::runtime_error("Spinnaker returned null RGB image");
              const bool rgb_good = !rgb_image->IsIncomplete() && rgb_image->GetWidth() == 4096 &&
                rgb_image->GetHeight() == 3000 && rgb_image->GetStride() == 4096 &&
                rgb_image->GetImageSize() == 12288000;
              if (!rgb_good) throw std::runtime_error("incomplete RGB frame or Bayer payload mismatch");
            } catch (const Spinnaker::Exception & error) {
              if (rgb_image) {try {rgb_image->Release();} catch (...) {}}
              image->Release();
              throw std::runtime_error(std::string("RGB acquisition failed after thermal frame: ") + error.what());
            } catch (...) {
              if (rgb_image) {try {rgb_image->Release();} catch (...) {}}
              image->Release();
              throw;
            }
          }
#ifdef _WIN32
          std::this_thread::sleep_for(3ms); drain_serial(serial, serial_events);
          if (rgb_enabled_) {
            const auto pulse_wait_started = std::chrono::steady_clock::now();
            while (last_pulse_sequence_ <= last_paired_pulse &&
              std::chrono::steady_clock::now() - pulse_wait_started < 50ms)
            {
              std::this_thread::sleep_for(1ms);
              drain_serial(serial, serial_events);
            }
            if (last_pulse_sequence_ <= last_paired_pulse) {
              if (rgb_image) rgb_image->Release();
              image->Release();
              throw std::runtime_error("Arduino pulse record did not arrive for synchronized frame pair");
            }
          }
#endif
          ++complete;
          const auto thermal_camera_frame_id = image->GetFrameID();
          char filename[80]{};
          std::snprintf(filename, sizeof(filename), "frame_%08llu_640x513_mono16.raw", static_cast<unsigned long long>(complete));
          const fs::path final_path = thermal_output / filename;
          const fs::path partial = final_path.string() + ".partial";
          std::ofstream raw(partial, std::ios::binary);
          raw.write(static_cast<const char *>(image->GetData()), static_cast<std::streamsize>(image->GetBufferSize())); raw.close();
          if (!raw) {
            if (rgb_image) rgb_image->Release();
            image->Release(); throw std::runtime_error("raw frame write failed");
          }
          fs::rename(partial, final_path);
          manifest << "{\"event\":\"frame\",\"sample\":" << complete
                   << ",\"frame_id\":" << image->GetFrameID()
                   << ",\"camera_timestamp_ns\":" << image->GetTimeStamp()
                   << ",\"host_receive_monotonic_ns\":" << host_ns
                   << ",\"arduino_pulse_sequence\":" << last_pulse_sequence_
                   << ",\"arduino_rise_us\":" << last_pulse_tick_
                   << ",\"time_status\":\"UNSYNCED\",\"file\":" << json_quote(filename) << "}\n";
          manifest.flush(); camera_times.push_back(image->GetTimeStamp());
          publish_frame(complete, image, host_ns); image->Release();

          if (rgb_enabled_) {
            const Spinnaker::ChunkData chunk = rgb_image->GetChunkData();
            if (chunk.GetFrameID() < 0 || chunk.GetTimestamp() < 0)
            {rgb_image->Release(); throw std::runtime_error("RGB Chunk FrameID/Timestamp is negative");}
            RgbFrameSettings settings;
            settings.chunk_frame_id = static_cast<std::uint64_t>(chunk.GetFrameID());
            settings.chunk_timestamp = static_cast<std::uint64_t>(chunk.GetTimestamp());
            settings.exposure_time_us = chunk.GetExposureTime();
            settings.gain_db = chunk.GetGain();
            settings.black_level = chunk.GetBlackLevel();
            auto & rgb_map = rgb_camera->GetNodeMap();
            settings.white_balance_red_valid =
              read_balance_ratio(rgb_map, "Red", settings.white_balance_red);
            settings.white_balance_blue_valid =
              read_balance_ratio(rgb_map, "Blue", settings.white_balance_blue);
            settings.exposure_auto = node_value(rgb_map, "ExposureAuto");
            settings.gain_auto = node_value(rgb_map, "GainAuto");
            settings.balance_white_auto = node_value(rgb_map, "BalanceWhiteAuto");
            ++rgb_complete;
            char rgb_filename[80]{};
            std::snprintf(
              rgb_filename, sizeof(rgb_filename), "frame_%08llu_4096x3000_bayerrg8.raw",
              static_cast<unsigned long long>(rgb_complete));
            const fs::path rgb_final_path = rgb_output / rgb_filename;
            const fs::path rgb_partial = rgb_final_path.string() + ".partial";
            std::ofstream rgb_raw(rgb_partial, std::ios::binary);
            rgb_raw.write(
              static_cast<const char *>(rgb_image->GetData()),
              static_cast<std::streamsize>(rgb_image->GetImageSize()));
            rgb_raw.close();
            if (!rgb_raw) {rgb_image->Release(); throw std::runtime_error("RGB raw frame write failed");}
            fs::rename(rgb_partial, rgb_final_path);
            rgb_manifest << "{\"event\":\"frame\",\"sample\":" << rgb_complete
                         << ",\"frame_id\":" << rgb_image->GetFrameID()
                         << ",\"camera_timestamp_ns\":" << rgb_image->GetTimeStamp()
                         << ",\"host_receive_monotonic_ns\":" << rgb_host_ns
                         << ",\"arduino_pulse_sequence\":" << last_pulse_sequence_
                         << ",\"arduino_rise_us\":" << last_pulse_tick_
                         << ",\"chunk_data_valid\":true"
                         << ",\"chunk_frame_id\":" << settings.chunk_frame_id
                         << ",\"chunk_timestamp\":" << settings.chunk_timestamp
                         << ",\"exposure_time_us\":" << settings.exposure_time_us
                         << ",\"gain_db\":" << settings.gain_db
                         << ",\"black_level\":" << settings.black_level
                         << ",\"white_balance_red_valid\":"
                         << (settings.white_balance_red_valid ? "true" : "false")
                         << ",\"white_balance_red\":" << settings.white_balance_red
                         << ",\"white_balance_blue_valid\":"
                         << (settings.white_balance_blue_valid ? "true" : "false")
                         << ",\"white_balance_blue\":" << settings.white_balance_blue
                         << ",\"exposure_auto\":" << json_quote(settings.exposure_auto)
                         << ",\"gain_auto\":" << json_quote(settings.gain_auto)
                         << ",\"balance_white_auto\":" << json_quote(settings.balance_white_auto)
                         << ",\"settings_source\":\"SPINNAKER_CHUNK_PLUS_POST_FRAME_WHITE_BALANCE_READBACK\""
                         << ",\"time_status\":\"UNSYNCED\",\"file\":"
                         << json_quote(rgb_filename) << "}\n";
            rgb_manifest.flush();
            publish_rgb_frame(rgb_complete, rgb_image, rgb_host_ns, settings);
            const auto rgb_frame_id = rgb_image->GetFrameID();
            rgb_image->Release();
            ++paired_complete;
            pair_manifest << "{\"pair\":" << paired_complete
                          << ",\"arduino_pulse_sequence\":" << last_pulse_sequence_
                          << ",\"arduino_rise_us\":" << last_pulse_tick_
                          << ",\"thermal_sample\":" << complete
                          << ",\"thermal_frame_id\":" << thermal_camera_frame_id
                          << ",\"rgb_sample\":" << rgb_complete
                          << ",\"rgb_frame_id\":" << rgb_frame_id
                          << ",\"host_receive_delta_ns\":"
                          << static_cast<std::int64_t>(rgb_host_ns) - static_cast<std::int64_t>(host_ns)
                          << ",\"note\":\"camera clocks are independent; do not subtract camera timestamps\"}\n";
            pair_manifest.flush();
            last_paired_pulse = last_pulse_sequence_;
          }
          if (fault_test_ && watchdog_restarted && complete >= recovery_frame_baseline + 4) {reason = "fault_recovery_suite_complete"; result = 0; break;}
        } catch (const Spinnaker::Exception & error) {
          if (error.GetError() != Spinnaker::SPINNAKER_ERR_TIMEOUT) throw;
        }
        if (fs::space(output).available < 10ULL * 1024ULL * 1024ULL * 1024ULL) throw std::runtime_error("free space below 10 GiB");
      }
      if (reason == "unknown") {reason = "operator_stop"; result = 0;}
      if (rgb_enabled_ && (complete != rgb_complete || complete != paired_complete)) {
        throw std::runtime_error("RGB/thermal/pair counts diverged");
      }
      if (camera_times.size() >= 2) {
        std::uint64_t min_delta = UINT64_MAX, max_delta = 0, sum = 0;
        for (std::size_t i = 1; i < camera_times.size(); ++i) {const auto d = camera_times[i] - camera_times[i - 1]; min_delta = std::min(min_delta, d); max_delta = std::max(max_delta, d); sum += d;}
        std::ofstream timing(output / "timing_summary.json", std::ios::binary);
        timing << "{\"time_status\":\"UNSYNCED\",\"gps_present\":false,\"frames\":" << complete
               << ",\"camera_interval_mean_ns\":" << sum / (camera_times.size() - 1)
               << ",\"camera_interval_min_ns\":" << min_delta << ",\"camera_interval_max_ns\":" << max_delta
               << ",\"note\":\"interval and sequence validation only; absolute UTC accuracy cannot be tested without GPS/PPS\"}\n";
      }
    } catch (const std::exception & error) {
      reason = std::string("error: ") + error.what();
      RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
    }
    cleanup();
    if (!output.empty() && fs::exists(output)) {
      std::ofstream status(output / "capture_complete.json", std::ios::binary);
      status << "{\"complete\":" << (result == 0 ? "true" : "false")
             << ",\"frames\":" << complete
             << ",\"thermal_frames\":" << complete
             << ",\"rgb_enabled\":" << (rgb_enabled_ ? "true" : "false")
             << ",\"rgb_frames\":" << rgb_complete
             << ",\"synchronized_pairs\":" << paired_complete
             << ",\"reason\":" << json_quote(reason) << ",\"configuration_restored\":true}\n";
    }
    RCLCPP_INFO(
      get_logger(),
      "capture complete: thermal=%llu rgb=%llu pairs=%llu reason=%s configuration_restored=true",
      static_cast<unsigned long long>(complete), static_cast<unsigned long long>(rgb_complete),
      static_cast<unsigned long long>(paired_complete), reason.c_str());
    if (rclcpp::ok()) rclcpp::shutdown();
  }

  std::string dataset_name_, device_id_, rgb_device_id_, serial_port_, output_root_;
  std::string rgb_exposure_auto_,rgb_gain_auto_,rgb_balance_white_auto_;
  std::string rgb_balance_white_auto_profile_,rgb_auto_exposure_control_priority_;
  double rgb_auto_exposure_time_upper_limit_us_{5000.0};
  double rgb_auto_exposure_gain_upper_limit_db_{12.0};
  double duration_sec_{0.0};
  bool fault_test_{false}, rgb_enabled_{false};
  std::atomic<bool> stop_{false};
  std::thread worker_;
  std::string serial_buffer_;
  std::uint64_t last_pulse_sequence_{0}, last_pulse_tick_{0};
  bool watchdog_stop_seen_{false};
  rclcpp::Publisher<ppbng_interfaces::msg::FrameMetadata>::SharedPtr publisher_;
  rclcpp::Publisher<ppbng_interfaces::msg::FrameMetadata>::SharedPtr rgb_publisher_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<A6701UnoR4TestNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    std::fprintf(stderr, "A6701_UNO_R4_ROS2_TEST_FAILED %s\n", error.what());
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
  return 0;
}
