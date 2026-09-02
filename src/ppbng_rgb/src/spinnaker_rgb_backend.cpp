#include "ppbng_rgb/spinnaker_rgb_backend.hpp"

#include <Spinnaker.h>

#include <algorithm>
#include <cstring>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

static_assert(FLIR_SPINNAKER_VERSION_MAJOR == 4 && FLIR_SPINNAKER_VERSION_MINOR == 4,
  "ppbng_rgb was validated against Spinnaker SDK 4.4");

namespace ppbng_rgb
{
namespace
{
using namespace Spinnaker;
using namespace Spinnaker::GenApi;

class ScopedImageRelease
{
public:
  explicit ScopedImageRelease(ImagePtr image) : image_(image) {}
  ~ScopedImageRelease() noexcept
  {
    // Official Spinnaker acquisition examples explicitly Release every image.
    // Keeping this in a guard covers success, early return, and exception paths.
    if (image_) {try {image_->Release();} catch (...) {}}
  }
  ScopedImageRelease(const ScopedImageRelease &) = delete;
  ScopedImageRelease & operator=(const ScopedImageRelease &) = delete;
  ImagePtr & get() noexcept {return image_;}

private:
  ImagePtr image_;
};

Status preflight(std::chrono::milliseconds timeout, const StopToken & stop)
{
  if (stop.stop_requested()) {return {ErrorCode::cancelled, "stop requested"};}
  if (timeout.count() <= 0) {return {ErrorCode::timeout, "positive timeout required"};}
  return {};
}

Status exception_status(const Exception & error, const char * operation)
{
  ErrorCode code = ErrorCode::invalid_state;
  if (error.GetError() == SPINNAKER_ERR_TIMEOUT) {code = ErrorCode::timeout;}
  else if (error.GetError() == SPINNAKER_ERR_ACCESS_DENIED ||
    error.GetError() == SPINNAKER_ERR_RESOURCE_IN_USE)
  {code = ErrorCode::access_denied;}
  else if (error.GetError() == SPINNAKER_ERR_INVALID_PARAMETER ||
    error.GetError() == SPINNAKER_ERR_INVALID_VALUE ||
    error.GetError() == SPINNAKER_ERR_GENICAM_OUT_OF_RANGE)
  {code = ErrorCode::invalid_configuration;}
  std::ostringstream detail;
  detail << operation << ": Spinnaker error " << static_cast<int>(error.GetError()) << ": " << error.what();
  return {code, detail.str()};
}

std::string enum_value(INodeMap & map, const char * name)
{
  CEnumerationPtr node = map.GetNode(name);
  if (!IsReadable(node) || !IsReadable(node->GetCurrentEntry())) {
    throw std::runtime_error(std::string("unreadable enum node: ") + name);
  }
  return std::string(node->GetCurrentEntry()->GetSymbolic().c_str());
}

std::size_t integer_value(INodeMap & map, const char * name)
{
  CIntegerPtr node = map.GetNode(name);
  if (!IsReadable(node) || node->GetValue() < 0) {
    throw std::runtime_error(std::string("unreadable integer node: ") + name);
  }
  return static_cast<std::size_t>(node->GetValue());
}

void transactional_set_enum(
  INodeMap & map, const char * name, const std::string & value,
  std::vector<std::function<void()>> & rollback)
{
  CEnumerationPtr node = map.GetNode(name);
  if (!IsReadable(node) || !IsWritable(node) || !IsReadable(node->GetCurrentEntry())) {
    throw std::runtime_error(std::string("enum node is not readable/writable: ") + name);
  }
  const auto original = node->GetIntValue();
  CEnumEntryPtr entry = node->GetEntryByName(value.c_str());
  if (!IsReadable(entry)) {
    throw std::runtime_error(std::string("unsupported ") + name + " value: " + value);
  }
  node->SetIntValue(entry->GetValue());
  rollback.emplace_back([node, original]() {if (IsWritable(node)) {node->SetIntValue(original);}});
}

void transactional_set_integer(
  INodeMap & map, const char * name, std::size_t value,
  std::vector<std::function<void()>> & rollback)
{
  if (value > static_cast<std::size_t>((std::numeric_limits<std::int64_t>::max)())) {
    throw std::runtime_error(std::string("integer value too large: ") + name);
  }
  CIntegerPtr node = map.GetNode(name);
  if (!IsReadable(node) || !IsWritable(node)) {
    throw std::runtime_error(std::string("integer node is not readable/writable: ") + name);
  }
  const auto original = node->GetValue();
  node->SetValue(static_cast<std::int64_t>(value));
  rollback.emplace_back([node, original]() {if (IsWritable(node)) {node->SetValue(original);}});
}

void transactional_set_bool(
  INodeMap & map, const char * name, bool value,
  std::vector<std::function<void()>> & rollback)
{
  CBooleanPtr node = map.GetNode(name);
  if (!IsReadable(node) || !IsWritable(node)) {
    throw std::runtime_error(std::string("boolean node is not readable/writable: ") + name);
  }
  const bool original = node->GetValue();
  node->SetValue(value);
  rollback.emplace_back([node, original]() {if (IsWritable(node)) {node->SetValue(original);}});
}

void transactional_set_float(
  INodeMap & map, const char * name, double value,
  std::vector<std::function<void()>> & rollback)
{
  CFloatPtr node = map.GetNode(name);
  if (!IsReadable(node) || !IsWritable(node) || value < node->GetMin() || value > node->GetMax()) {
    throw std::runtime_error(std::string("float node is not writable or value is out of range: ") + name);
  }
  const double original = node->GetValue();
  node->SetValue(value);
  rollback.emplace_back([node, original]() {if (IsWritable(node)) {node->SetValue(original);}});
}

double float_value(INodeMap & map, const char * name)
{
  CFloatPtr node = map.GetNode(name);
  if (!IsReadable(node)) {throw std::runtime_error(std::string("unreadable float node: ") + name);}
  return node->GetValue();
}

constexpr const char * required_chunks[] = {
  "FrameID", "Timestamp", "ExposureTime", "Gain", "BlackLevel"};

void configure_required_chunks(
  INodeMap & map, std::vector<std::function<void()>> & rollback)
{
  transactional_set_bool(map, "ChunkModeActive", true, rollback);
  CEnumerationPtr selector = map.GetNode("ChunkSelector");
  if (!IsReadable(selector) || !IsWritable(selector) || !IsReadable(selector->GetCurrentEntry())) {
    throw std::runtime_error("ChunkSelector is not readable/writable");
  }
  const auto original_selector = selector->GetIntValue();
  rollback.emplace_back([selector, original_selector]() {
    if (IsWritable(selector)) {selector->SetIntValue(original_selector);}
  });
  for (const char * name : required_chunks) {
    CEnumEntryPtr entry = selector->GetEntryByName(name);
    if (!IsReadable(entry)) {throw std::runtime_error(std::string("required chunk unavailable: ") + name);}
    selector->SetIntValue(entry->GetValue());
    CBooleanPtr enabled = map.GetNode("ChunkEnable");
    if (!IsReadable(enabled) || !IsWritable(enabled)) {
      throw std::runtime_error(std::string("required chunk cannot be enabled: ") + name);
    }
    const bool original = enabled->GetValue();
    enabled->SetValue(true);
    const auto selected_value = entry->GetValue();
    rollback.emplace_back([selector, enabled, selected_value, original]() {
      if (IsWritable(selector)) {selector->SetIntValue(selected_value);}
      if (IsWritable(enabled)) {enabled->SetValue(original);}
    });
    if (!enabled->GetValue()) {throw std::runtime_error(std::string("chunk enable readback failed: ") + name);}
  }
  selector->SetIntValue(original_selector);
}

bool required_chunks_enabled(INodeMap & map)
{
  CBooleanPtr mode = map.GetNode("ChunkModeActive");
  CEnumerationPtr selector = map.GetNode("ChunkSelector");
  if (!IsReadable(mode) || !mode->GetValue() || !IsReadable(selector) ||
    !IsWritable(selector) || !IsReadable(selector->GetCurrentEntry()))
  {return false;}
  const auto original_selector = selector->GetIntValue();
  bool result = true;
  try {
    for (const char * name : required_chunks) {
      CEnumEntryPtr entry = selector->GetEntryByName(name);
      if (!IsReadable(entry)) {result = false; break;}
      selector->SetIntValue(entry->GetValue());
      CBooleanPtr enabled = map.GetNode("ChunkEnable");
      if (!IsReadable(enabled) || !enabled->GetValue()) {result = false; break;}
    }
    selector->SetIntValue(original_selector);
  } catch (...) {
    try {selector->SetIntValue(original_selector);} catch (...) {}
    throw;
  }
  return result;
}

bool read_balance_ratio(INodeMap & map, const char * channel, double & value)
{
  CEnumerationPtr selector = map.GetNode("BalanceRatioSelector");
  CFloatPtr ratio = map.GetNode("BalanceRatio");
  if (!IsReadable(selector) || !IsWritable(selector) || !IsReadable(selector->GetCurrentEntry()) ||
    !IsReadable(ratio))
  {return false;}
  const auto original = selector->GetIntValue();
  try {
    CEnumEntryPtr entry = selector->GetEntryByName(channel);
    if (!IsReadable(entry)) {return false;}
    selector->SetIntValue(entry->GetValue());
    value = ratio->GetValue();
    selector->SetIntValue(original);
    return true;
  } catch (...) {
    try {selector->SetIntValue(original);} catch (...) {}
    return false;
  }
}

std::string camera_serial(const CameraPtr & camera)
{
  INodeMap & tl = camera->GetTLDeviceNodeMap();
  CStringPtr serial = tl.GetNode("DeviceSerialNumber");
  if (!IsReadable(serial)) {throw std::runtime_error("DeviceSerialNumber is unreadable");}
  return std::string(serial->GetValue().c_str());
}
}  // namespace

class SpinnakerRgbBackend::Impl
{
public:
  ~Impl() {shutdown();}

  void shutdown() noexcept
  {
    try {if (acquiring_ && camera_) {camera_->EndAcquisition();}} catch (...) {}
    acquiring_ = false;
    try {if (initialized_ && camera_) {camera_->DeInit();}} catch (...) {}
    initialized_ = false;
    camera_ = nullptr;
    try {if (system_) {system_->ReleaseInstance();}} catch (...) {}
    system_ = nullptr;
  }

  Status discover(std::chrono::milliseconds timeout, const StopToken & stop, std::vector<std::string> & ids)
  {
    auto status = preflight(timeout, stop); if (!status.ok()) {return status;}
    ids.clear(); shutdown();
    try {
      system_ = System::GetInstance();
      CameraList cameras = system_->GetCameras();
      for (unsigned int index = 0; index < cameras.GetSize(); ++index) {
        const auto serial = camera_serial(cameras.GetByIndex(index));
        if (serial.empty()) {throw std::runtime_error("camera reported an empty serial number");}
        ids.push_back(serial);
      }
      cameras.Clear();
      std::sort(ids.begin(), ids.end());
      if (std::adjacent_find(ids.begin(), ids.end()) != ids.end()) {
        state_ = LifecycleState::faulted;
        return {ErrorCode::invalid_configuration, "duplicate camera serial numbers discovered"};
      }
      state_ = LifecycleState::discovered;
      return {};
    } catch (const Exception & error) {
      state_ = LifecycleState::faulted; return exception_status(error, "discover");
    } catch (const std::exception & error) {
      state_ = LifecycleState::faulted; return {ErrorCode::invalid_state, std::string("discover: ") + error.what()};
    }
  }

  Status open(const std::string & id, std::chrono::milliseconds timeout, const StopToken & stop)
  {
    auto status = preflight(timeout, stop); if (!status.ok()) {return status;}
    if (state_ != LifecycleState::discovered || !system_) {
      return {ErrorCode::invalid_state, "discover must succeed before open"};
    }
    try {
      CameraList cameras = system_->GetCameras();
      std::vector<std::string> ids;
      for (unsigned int index = 0; index < cameras.GetSize(); ++index) {ids.push_back(camera_serial(cameras.GetByIndex(index)));}
      std::string selected;
      status = select_unique_device_id(id, ids, selected);
      if (!status.ok()) {cameras.Clear(); return status;}
      for (unsigned int index = 0; index < cameras.GetSize(); ++index) {
        if (camera_serial(cameras.GetByIndex(index)) == selected) {camera_ = cameras.GetByIndex(index); break;}
      }
      cameras.Clear();
      if (!camera_) {return {ErrorCode::not_found, "selected camera disappeared before open"};}
      camera_->Init(); initialized_ = true; selected_id_ = selected; state_ = LifecycleState::open;
      return {};
    } catch (const Exception & error) {
      state_ = LifecycleState::faulted; return exception_status(error, "open");
    } catch (const std::exception & error) {
      state_ = LifecycleState::faulted; return {ErrorCode::invalid_state, std::string("open: ") + error.what()};
    }
  }

  Status configure(const RgbConfiguration & requested, std::chrono::milliseconds timeout, const StopToken & stop)
  {
    auto status = preflight(timeout, stop); if (!status.ok()) {return status;}
    if (state_ != LifecycleState::open || !initialized_) {return {ErrorCode::invalid_state, "open first"};}
    if (!requested.hardware_trigger || requested.trigger_source.empty() || requested.trigger_source == "Software") {
      return {ErrorCode::invalid_configuration, "an external hardware trigger source is required"};
    }
    std::vector<std::function<void()>> rollback;
    try {
      INodeMap & map = camera_->GetNodeMap();
      // TriggerMode must be Off while selector/source/activation are changed.
      transactional_set_enum(map, "TriggerMode", "Off", rollback);
      transactional_set_enum(map, "AcquisitionMode", requested.acquisition_mode, rollback);
      transactional_set_enum(map, "PixelFormat", requested.pixel_format, rollback);
      transactional_set_integer(map, "Width", requested.width, rollback);
      transactional_set_integer(map, "Height", requested.height, rollback);
      transactional_set_enum(map, "TriggerSelector", requested.trigger_selector, rollback);
      transactional_set_enum(map, "TriggerSource", requested.trigger_source, rollback);
      transactional_set_enum(map, "TriggerActivation", requested.trigger_activation, rollback);
      transactional_set_enum(map, "BalanceWhiteAutoProfile", requested.balance_white_auto_profile, rollback);
      transactional_set_enum(
        map, "AutoExposureControlPriority", requested.auto_exposure_control_priority, rollback);
      transactional_set_float(
        map, "AutoExposureExposureTimeUpperLimit",
        requested.auto_exposure_time_upper_limit_us, rollback);
      transactional_set_float(
        map, "AutoExposureGainUpperLimit",
        requested.auto_exposure_gain_upper_limit_db, rollback);
      transactional_set_enum(map, "ExposureAuto", requested.exposure_auto, rollback);
      transactional_set_enum(map, "GainAuto", requested.gain_auto, rollback);
      transactional_set_enum(map, "BalanceWhiteAuto", requested.balance_white_auto, rollback);
      if (requested.chunk_data_enabled) {configure_required_chunks(map, rollback);}
      transactional_set_enum(map, "TriggerMode", "On", rollback);
      RgbReadback actual;
      status = readback(timeout, stop, actual);
      if (!status.ok()) {throw std::runtime_error(status.detail);}
      if (configuration_hash(requested) != configuration_hash(actual)) {
        throw std::runtime_error("configuration readback does not exactly match request");
      }
      status = validate_raw_bayer_contract(actual);
      if (!status.ok()) {throw std::runtime_error(status.detail);}
      configured_ = requested; state_ = LifecycleState::configured; return {};
    } catch (const Exception & error) {
      for (auto it = rollback.rbegin(); it != rollback.rend(); ++it) {try {(*it)();} catch (...) {}}
      state_ = LifecycleState::open; return exception_status(error, "configure transaction rolled back");
    } catch (const std::exception & error) {
      for (auto it = rollback.rbegin(); it != rollback.rend(); ++it) {try {(*it)();} catch (...) {}}
      state_ = LifecycleState::open;
      const std::string detail = error.what();
      const auto code = detail.find("readback") == std::string::npos ? ErrorCode::invalid_configuration : ErrorCode::readback_mismatch;
      return {code, std::string("configure transaction rolled back: ") + detail};
    }
  }

  Status readback(std::chrono::milliseconds timeout, const StopToken & stop, RgbReadback & result)
  {
    auto status = preflight(timeout, stop); if (!status.ok()) {return status;}
    if (!initialized_ || !camera_) {return {ErrorCode::invalid_state, "camera is not open"};}
    try {
      INodeMap & map = camera_->GetNodeMap();
      result.pixel_format = enum_value(map, "PixelFormat");
      result.width = integer_value(map, "Width"); result.height = integer_value(map, "Height");
      const auto transport_payload_bytes = integer_value(map, "PayloadSize");
      if (result.pixel_format != "BayerRG8" || result.height == 0 ||
        result.width > (std::numeric_limits<std::size_t>::max)() / result.height)
      {return {ErrorCode::readback_mismatch, "unsupported or invalid RGB image layout"};}
      // GenICam PayloadSize can grow when chunks are appended. The stored raw
      // image remains the packed Bayer image returned by Image::GetImageSize().
      result.row_stride_bytes = result.width;
      result.payload_bytes = result.width * result.height;
      if (transport_payload_bytes < result.payload_bytes) {
        return {ErrorCode::readback_mismatch, "transport PayloadSize is smaller than the Bayer image"};
      }
      result.acquisition_mode = enum_value(map, "AcquisitionMode");
      result.trigger_selector = enum_value(map, "TriggerSelector");
      result.trigger_source = enum_value(map, "TriggerSource");
      result.trigger_activation = enum_value(map, "TriggerActivation");
      result.exposure_auto = enum_value(map, "ExposureAuto");
      result.gain_auto = enum_value(map, "GainAuto");
      result.balance_white_auto = enum_value(map, "BalanceWhiteAuto");
      result.balance_white_auto_profile = enum_value(map, "BalanceWhiteAutoProfile");
      result.auto_exposure_control_priority = enum_value(map, "AutoExposureControlPriority");
      result.auto_exposure_time_upper_limit_us =
        float_value(map, "AutoExposureExposureTimeUpperLimit");
      result.auto_exposure_gain_upper_limit_db = float_value(map, "AutoExposureGainUpperLimit");
      result.chunk_data_enabled = required_chunks_enabled(map);
      result.hardware_trigger = enum_value(map, "TriggerMode") == "On";
      result.control_access = true;
      return {};
    } catch (const Exception & error) {return exception_status(error, "readback");}
    catch (const std::exception & error) {return {ErrorCode::readback_mismatch, std::string("readback: ") + error.what()};}
  }

  Status arm(std::chrono::milliseconds timeout, const StopToken & stop)
  {
    auto status = preflight(timeout, stop); if (!status.ok()) {return status;}
    if (state_ != LifecycleState::configured) {return {ErrorCode::invalid_state, "configure first"};}
    RgbReadback actual; status = readback(timeout, stop, actual); if (!status.ok()) {return status;}
    if (configuration_hash(configured_) != configuration_hash(actual)) {
      return {ErrorCode::readback_mismatch, "configuration changed before arm"};
    }
    status = validate_raw_bayer_contract(actual); if (!status.ok()) {return status;}
    try {camera_->BeginAcquisition(); acquiring_ = true; state_ = LifecycleState::armed; return {};}
    catch (const Exception & error) {state_ = LifecycleState::faulted; return exception_status(error, "arm");}
  }

  FrameResult next_frame(std::chrono::milliseconds timeout, const StopToken & stop)
  {
    auto status = preflight(timeout, stop); if (!status.ok()) {return {status, {}};}
    if ((state_ != LifecycleState::armed && state_ != LifecycleState::streaming) || !acquiring_) {
      return {{ErrorCode::invalid_state, "arm first"}, {}};
    }
    try {
      ScopedImageRelease release_guard(
        camera_->GetNextImage(static_cast<std::uint64_t>(timeout.count())));
      ImagePtr & image = release_guard.get();
      const auto host_receive_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
      if (stop.stop_requested()) {return {{ErrorCode::cancelled, "stop requested"}, {}};}
      if (image->IsIncomplete()) {
        return {{ErrorCode::incomplete_frame, Image::GetImageStatusDescription(image->GetImageStatus())}, {}};
      }
      const std::size_t size = image->GetImageSize();
      const std::size_t stride = image->GetStride();
      const std::size_t height = image->GetHeight();
      if (stride == 0 || height == 0 || height > (std::numeric_limits<std::size_t>::max)() / stride ||
        size != stride * height || image->GetData() == nullptr)
      {return {{ErrorCode::incomplete_frame, "image layout differs from packed raw Bayer contract"}, {}};}
      auto bytes = std::make_shared<std::vector<std::byte>>(size);
      std::memcpy(bytes->data(), image->GetData(), size);
      const ChunkData chunk = image->GetChunkData();
      const auto chunk_frame_id = chunk.GetFrameID();
      const auto chunk_timestamp = chunk.GetTimestamp();
      if (chunk_frame_id < 0 || chunk_timestamp < 0)
      {return {{ErrorCode::incomplete_frame, "RGB Chunk FrameID/Timestamp is negative"}, {}};}
      FrameInfo info{image->GetFrameID(), segment_index_, size, true, configured_.pixel_format,
        image->GetTimeStamp(), host_receive_ns};
      info.chunk_data_valid = true;
      info.chunk_frame_id_valid = true; info.chunk_frame_id = static_cast<std::uint64_t>(chunk_frame_id);
      info.chunk_timestamp_valid = true; info.chunk_timestamp = static_cast<std::uint64_t>(chunk_timestamp);
      info.exposure_time_valid = true; info.exposure_time_us = chunk.GetExposureTime();
      info.gain_valid = true; info.gain_db = chunk.GetGain();
      info.black_level_valid = true; info.black_level = chunk.GetBlackLevel();
      INodeMap & map = camera_->GetNodeMap();
      info.white_balance_red_valid = read_balance_ratio(map, "Red", info.white_balance_red);
      info.white_balance_blue_valid = read_balance_ratio(map, "Blue", info.white_balance_blue);
      info.exposure_auto = enum_value(map, "ExposureAuto");
      info.gain_auto = enum_value(map, "GainAuto");
      info.balance_white_auto = enum_value(map, "BalanceWhiteAuto");
      state_ = LifecycleState::streaming;
      return {{}, FrameLease(info, std::move(bytes), []() {})};
    } catch (const Exception & error) {
      status = exception_status(error, "next_frame");
      if (status.code != ErrorCode::timeout) {state_ = LifecycleState::faulted;}
      return {status, {}};
    }
  }

  Status recover(std::chrono::milliseconds timeout, const StopToken & stop)
  {
    auto status = preflight(timeout, stop); if (!status.ok()) {return status;}
    if (selected_id_.empty()) {return {ErrorCode::invalid_state, "no selected camera to recover"};}
    const auto expected_id = selected_id_;
    std::vector<std::string> ids;
    status = discover(timeout, stop, ids);
    if (!status.ok()) {return status;}
    status = open(expected_id, timeout, stop);
    if (!status.ok()) {return status;}
    ++segment_index_;
    // Configuration is intentionally not restored silently: caller must transact/read back/arm again.
    return {};
  }

  Status stop(std::chrono::milliseconds timeout) noexcept
  {
    if (timeout.count() <= 0) {return {ErrorCode::timeout, "positive timeout required"};}
    try {
      // stop is the terminal/rejection cleanup contract, not merely EndAcquisition.
      // A failed start must never retain an initialized camera or System instance.
      shutdown(); state_ = LifecycleState::stopped; return {};
    } catch (const Exception & error) {state_ = LifecycleState::faulted; return exception_status(error, "stop");}
    catch (...) {state_ = LifecycleState::faulted; return {ErrorCode::invalid_state, "stop: unknown error"};}
  }

  LifecycleState state_{LifecycleState::idle};
  SystemPtr system_;
  CameraPtr camera_;
  bool initialized_{false};
  bool acquiring_{false};
  std::string selected_id_;
  RgbConfiguration configured_{};
  std::uint64_t segment_index_{0};
};

SpinnakerRgbBackend::SpinnakerRgbBackend() : impl_(std::make_unique<Impl>()) {}
SpinnakerRgbBackend::~SpinnakerRgbBackend() = default;
Status SpinnakerRgbBackend::discover(std::chrono::milliseconds t,const StopToken&s,std::vector<std::string>&i){return impl_->discover(t,s,i);}
Status SpinnakerRgbBackend::open(const std::string&i,std::chrono::milliseconds t,const StopToken&s){return impl_->open(i,t,s);}
Status SpinnakerRgbBackend::configure(const RgbConfiguration&c,std::chrono::milliseconds t,const StopToken&s){return impl_->configure(c,t,s);}
Status SpinnakerRgbBackend::readback(std::chrono::milliseconds t,const StopToken&s,RgbReadback&r){return impl_->readback(t,s,r);}
Status SpinnakerRgbBackend::arm(std::chrono::milliseconds t,const StopToken&s){return impl_->arm(t,s);}
FrameResult SpinnakerRgbBackend::next_frame(std::chrono::milliseconds t,const StopToken&s){return impl_->next_frame(t,s);}
Status SpinnakerRgbBackend::recover(std::chrono::milliseconds t,const StopToken&s){return impl_->recover(t,s);}
Status SpinnakerRgbBackend::stop(std::chrono::milliseconds t)noexcept{return impl_->stop(t);}
LifecycleState SpinnakerRgbBackend::state()const noexcept{return impl_->state_;}
}  // namespace ppbng_rgb
