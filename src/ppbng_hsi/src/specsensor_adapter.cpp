#include "ppbng_hsi/specsensor_adapter.hpp"

#include "ppbng_hsi/hsi_format.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <filesystem>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>

#if PPBNG_HSI_ENABLE_SPECSENSOR
#include <SI_errors.h>
#include <SI_sensor.h>
#endif

namespace ppbng_hsi
{

struct BoundedSpecSensorFrameQueue::Impl
{
  struct Slot
  {
    explicit Slot(const std::size_t maximum) : storage(maximum) {}
    std::vector<std::uint8_t> storage;
    std::size_t size{0U};
    std::int64_t frame_number{0};
    std::uint64_t host_receive_monotonic_ns{0U};
  };

  Impl(const std::size_t capacity, const std::size_t maximum)
  : maximum_frame_bytes(maximum)
  {
    slots.reserve(capacity);
    for (std::size_t index = 0; index < capacity; ++index) {
      slots.emplace_back(maximum);
    }
  }

  mutable std::mutex mutex;
  std::vector<Slot> slots;
  std::size_t maximum_frame_bytes{0U};
  std::size_t read_index{0U};
  std::size_t write_index{0U};
  std::size_t count{0U};
  std::uint64_t overflows{0U};
  std::uint64_t invalid_frames{0U};
};

BoundedSpecSensorFrameQueue::BoundedSpecSensorFrameQueue(
  const std::size_t capacity, const std::size_t maximum_frame_bytes)
: impl_(std::make_unique<Impl>(capacity, maximum_frame_bytes))
{
  if (capacity == 0U || maximum_frame_bytes == 0U) {
    throw std::invalid_argument("SpecSensor callback queue dimensions must be nonzero");
  }
}

BoundedSpecSensorFrameQueue::~BoundedSpecSensorFrameQueue() = default;

bool BoundedSpecSensorFrameQueue::try_push(
  const std::uint8_t * data, const std::size_t size,
  const std::int64_t frame_number, const std::uint64_t host_receive_monotonic_ns) noexcept
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (data == nullptr || size == 0U || size > impl_->maximum_frame_bytes || frame_number < 0) {
    ++impl_->invalid_frames;
    return false;
  }
  if (impl_->count == impl_->slots.size()) {
    ++impl_->overflows;
    return false;
  }
  auto & slot = impl_->slots[impl_->write_index];
  std::memcpy(slot.storage.data(), data, size);
  slot.size = size;
  slot.frame_number = frame_number;
  slot.host_receive_monotonic_ns = host_receive_monotonic_ns;
  impl_->write_index = (impl_->write_index + 1U) % impl_->slots.size();
  ++impl_->count;
  return true;
}

bool BoundedSpecSensorFrameQueue::try_pop(SpecSensorFrame & output)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->count == 0U) {
    return false;
  }
  auto & slot = impl_->slots[impl_->read_index];
  output.sdk_frame_number = slot.frame_number;
  output.host_receive_monotonic_ns = slot.host_receive_monotonic_ns;
  output.bytes.assign(slot.storage.begin(), slot.storage.begin() + slot.size);
  slot.size = 0U;
  impl_->read_index = (impl_->read_index + 1U) % impl_->slots.size();
  --impl_->count;
  return true;
}

void BoundedSpecSensorFrameQueue::clear() noexcept
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->read_index = 0U;
  impl_->write_index = 0U;
  impl_->count = 0U;
  for (auto & slot : impl_->slots) {
    slot.size = 0U;
  }
}

std::size_t BoundedSpecSensorFrameQueue::size() const noexcept
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->count;
}

std::size_t BoundedSpecSensorFrameQueue::capacity() const noexcept
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->slots.size();
}

std::uint64_t BoundedSpecSensorFrameQueue::overflow_count() const noexcept
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->overflows;
}

std::uint64_t BoundedSpecSensorFrameQueue::invalid_frame_count() const noexcept
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->invalid_frames;
}

SpecSensorQueueStats BoundedSpecSensorFrameQueue::stats() const noexcept
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return {impl_->slots.size(), impl_->count, impl_->overflows, impl_->invalid_frames};
}

OperationResult validate_specsensor_options(const SpecSensorBackendOptions & options)
{
  if (options.device_index < 0) {
    return {false, "device_index must be explicitly configured"};
  }
  if (options.expected_profile_name.empty()) {
    return {false, "expected_profile_name must be explicitly configured"};
  }
  if (options.expected_sensor_serial.empty()) {
    return {false, "expected_sensor_serial must be explicitly configured"};
  }
  if (options.calibration_pack_path.empty()) {
    return {false, "calibration_pack_path must be explicitly configured"};
  }
  if (options.initialization_timeout_ms == 0U) {
    return {false, "initialization_timeout_ms must be nonzero"};
  }
  if (options.callback_queue_capacity == 0U || options.maximum_frame_bytes == 0U) {
    return {false, "callback queue bounds must be nonzero"};
  }
  if (options.callback_queue_capacity > 512U) {
    return {false, "callback_queue_capacity exceeds the 512-frame safety limit"};
  }
  if (options.maximum_frame_bytes > 64U * 1024U * 1024U) {
    return {false, "maximum_frame_bytes exceeds the 64 MiB safety limit"};
  }
  constexpr std::size_t maximum_callback_storage = 1024U * 1024U * 1024U;
  if (options.maximum_frame_bytes > maximum_callback_storage /
    options.callback_queue_capacity)
  {
    return {false, "preallocated callback storage exceeds the 1 GiB safety limit"};
  }
  if (options.kind == CameraKind::fx10e &&
    options.transport != SpecSensorTransport::pleora_gige)
  {
    return {false, "FX10e requires the documented Pleora GigE profile"};
  }
  if (options.kind == CameraKind::fx10e && options.pleora_packet_size == 0U) {
    return {false, "FX10e requires an explicit Pleora packet size"};
  }
  if (options.kind == CameraKind::fx10e &&
    (options.grabber_channel.empty() || options.grabber_channel == L"ui"))
  {
    return {false, "FX10e requires a non-interactive grabber_channel; 'ui' is forbidden"};
  }
  if (options.kind == CameraKind::swir &&
    options.transport != SpecSensorTransport::ni_camera_link)
  {
    return {false, "SWIR requires the documented NI Camera Link profile"};
  }
  if (options.kind == CameraKind::swir &&
    (options.ni_grabber_channel.empty() || options.ni_camera_file_path.empty()))
  {
    return {false, "SWIR requires explicit NI grabber channel and camera file"};
  }
  return {true, "SpecSensor options are structurally valid"};
}

bool specsensor_backend_compiled() noexcept
{
#if PPBNG_HSI_ENABLE_SPECSENSOR
  return true;
#else
  return false;
#endif
}

namespace
{
OperationResult unavailable()
{
  return {false,
    "SpecSensor backend is disabled; rebuild with PPBNG_HSI_ENABLE_SPECSENSOR=ON"};
}

#if PPBNG_HSI_ENABLE_SPECSENSOR
std::recursive_mutex g_sdk_mutex;
std::size_t g_sdk_users{0U};
std::wstring g_loaded_license;

std::string narrow_ascii(const std::wstring & input)
{
  std::string output;
  output.reserve(input.size());
  for (const wchar_t character : input) {
    output.push_back(character >= 0 && character <= 127 ? static_cast<char>(character) : '?');
  }
  return output;
}

OperationResult si_result(const int code, const char * operation)
{
  if (SI_SUCCEEDED(code)) {
    return {true, operation};
  }
  const SI_WC * description = SI_GetErrorString(code);
  return {false, std::string(operation) + " failed (" + std::to_string(code) + "): " +
      (description == nullptr ? "unknown SpecSensor error" : narrow_ascii(description))};
}

OperationResult acquire_sdk(const std::wstring & license)
{
  std::lock_guard<std::recursive_mutex> lock(g_sdk_mutex);
  if (g_sdk_users == 0U) {
    const auto result = si_result(SI_Load(license.c_str()), "SI_Load");
    if (!result.success) {
      return result;
    }
    g_loaded_license = license;
  } else if (license != g_loaded_license) {
    return {false, "all SpecSensor instances must share one license path"};
  }
  ++g_sdk_users;
  return {true, "SpecSensor global lifetime acquired"};
}

OperationResult release_sdk()
{
  std::lock_guard<std::recursive_mutex> lock(g_sdk_mutex);
  if (g_sdk_users == 0U) {
    return {true, "SpecSensor already released"};
  }
  --g_sdk_users;
  if (g_sdk_users == 0U) {
    const auto result = si_result(SI_Unload(), "SI_Unload");
    g_loaded_license.clear();
    return result;
  }
  return {true, "SpecSensor instance released"};
}

template<typename Function>
OperationResult locked_si(Function && function, const char * operation)
{
  std::lock_guard<std::recursive_mutex> lock(g_sdk_mutex);
  return si_result(function(), operation);
}

bool approximately_equal(
  const double first, const double second,
  const double absolute_tolerance, const double relative_tolerance)
{
  const double tolerance = std::max(absolute_tolerance, std::abs(first) * relative_tolerance);
  return std::abs(first - second) <= tolerance;
}
#endif
}  // namespace

struct SpecSensorHsiAdapter::Impl
{
  explicit Impl(SpecSensorBackendOptions value)
  : options(std::move(value)), queue(options.callback_queue_capacity, options.maximum_frame_bytes)
  {
    config.kind = options.kind;
  }

  SpecSensorBackendOptions options;
  mutable std::recursive_mutex mutex;
  HsiState state{HsiState::disconnected};
  HsiConfig config;
  SpecSensorReadback readback;
  BoundedSpecSensorFrameQueue queue;
  std::uint32_t segment{0U};
  std::size_t dark_lines_remaining{0U};
  std::uint64_t segment_line_index{0U};
  // Advance the persisted segment lazily on the first frame of a new epoch.
  // Repeated recoveries that produce no frames must not create segment gaps.
  bool segment_boundary_pending{false};
  std::uint64_t previous_trigger_sequence{0U};
  std::int64_t previous_sdk_frame_number{-1};
  bool callback_registered{false};
  bool sdk_acquired{false};
#if PPBNG_HSI_ENABLE_SPECSENSOR
  SI_H handle{nullptr};
#endif
};

SpecSensorHsiAdapter::SpecSensorHsiAdapter(SpecSensorBackendOptions options)
: impl_(std::make_unique<Impl>(std::move(options))) {}

SpecSensorHsiAdapter::~SpecSensorHsiAdapter()
{
#if PPBNG_HSI_ENABLE_SPECSENSOR
  std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
  if (impl_->handle != nullptr) {
    if (impl_->callback_registered) {
      (void)locked_si(
        [this]() {return SI_UnregisterDataCallback(impl_->handle);},
        "SI_UnregisterDataCallback");
    }
    (void)locked_si([this]() {return SI_Close(impl_->handle);}, "SI_Close");
    impl_->handle = nullptr;
  }
  if (impl_->sdk_acquired) {
    (void)release_sdk();
  }
#endif
}

CameraKind SpecSensorHsiAdapter::kind() const noexcept {return impl_->options.kind;}
HsiState SpecSensorHsiAdapter::state() const noexcept
{
  std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
  return impl_->state;
}
const HsiConfig & SpecSensorHsiAdapter::config() const noexcept {return impl_->config;}
std::uint32_t SpecSensorHsiAdapter::segment_id() const noexcept {return impl_->segment;}
SpecSensorReadback SpecSensorHsiAdapter::readback() const
{
  std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
  return impl_->readback;
}

SpecSensorQueueStats SpecSensorHsiAdapter::queue_stats() const noexcept
{
  if (!impl_) {return {};}
  return impl_->queue.stats();
}

#if PPBNG_HSI_ENABLE_SPECSENSOR
namespace
{
int SI_IMPEXP_CONV data_callback(
  SI_U8 * buffer, const SI_64 frame_size, const SI_64 frame_number, void * context)
{
  auto * queue = static_cast<BoundedSpecSensorFrameQueue *>(context);
  if (queue == nullptr || frame_size <= 0) {
    return 0;
  }
  const auto arrival_ns = static_cast<std::uint64_t>(std::chrono::duration_cast<
    std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
  (void)queue->try_push(buffer, static_cast<std::size_t>(frame_size),
    static_cast<std::int64_t>(frame_number), arrival_ns);
  return 0;
}
}  // namespace
#endif

OperationResult SpecSensorHsiAdapter::connect()
{
  std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
#if !PPBNG_HSI_ENABLE_SPECSENSOR
  return unavailable();
#else
  if (impl_->state != HsiState::disconnected) {
    return {false, "connect requires disconnected state"};
  }
  const auto valid = validate_specsensor_options(impl_->options);
  if (!valid.success) {
    return valid;
  }
  auto result = acquire_sdk(impl_->options.license_path);
  if (!result.success) {
    impl_->state = HsiState::fault;
    return result;
  }
  impl_->sdk_acquired = true;

  SI_WC profile_name[4096]{};
  result = locked_si([this, &profile_name]() {
      return SI_GetEnumStringByIndex(
        SI_SYSTEM, L"DeviceName", impl_->options.device_index, profile_name, 4096);
    }, "SI_GetEnumStringByIndex(DeviceName)");
  if (!result.success) {
    (void)release_sdk();
    impl_->sdk_acquired = false;
    impl_->state = HsiState::fault;
    return result;
  }
  impl_->readback.profile_name = narrow_ascii(profile_name);
  if (impl_->readback.profile_name != impl_->options.expected_profile_name) {
    (void)release_sdk();
    impl_->sdk_acquired = false;
    impl_->state = HsiState::fault;
    return {false, "device_index profile does not match expected_profile_name"};
  }

  result = locked_si(
    [this]() {return SI_Open(impl_->options.device_index, &impl_->handle);}, "SI_Open");
  if (!result.success) {
    (void)release_sdk();
    impl_->sdk_acquired = false;
    impl_->state = HsiState::fault;
    return result;
  }

  const auto fail_open = [this](OperationResult failure) {
      if (impl_->handle != nullptr) {
        (void)locked_si([this]() {return SI_Close(impl_->handle);}, "SI_Close");
        impl_->handle = nullptr;
      }
      if (impl_->sdk_acquired) {
        (void)release_sdk();
        impl_->sdk_acquired = false;
      }
      impl_->state = HsiState::fault;
      return failure;
    };
  const auto set_string = [this](const SI_WC * feature, const std::wstring & value,
      const char * operation) {
      if (value.empty()) {return OperationResult{true, operation};}
      std::vector<SI_WC> mutable_value(value.begin(), value.end());
      mutable_value.push_back(L'\0');
      return locked_si([this, feature, &mutable_value]() {
          return SI_SetString(impl_->handle, feature, mutable_value.data());
        }, operation);
    };

  if (impl_->options.kind == CameraKind::fx10e) {
    const auto requested = impl_->options.grabber_channel;
    SI_BOOL channels_implemented = SI_FALSE;
    result = locked_si([this, &channels_implemented]() {
        return SI_IsImplemented(
          impl_->handle, L"Grabber.Channels", &channels_implemented);
      }, "SI_IsImplemented(Grabber.Channels)");
    if (!result.success) {return fail_open(result);}
    if (channels_implemented == SI_TRUE) {
      int channel_count = 0;
      result = locked_si([this, &channel_count]() {
          return SI_GetEnumCount(impl_->handle, L"Grabber.Channels", &channel_count);
        }, "SI_GetEnumCount(Grabber.Channels)");
      if (!result.success) {return fail_open(result);}
      std::vector<std::wstring> available_channels;
      for (int index = 0; index < channel_count; ++index) {
        SI_WC channel[1024]{};
        result = locked_si([this, index, &channel]() {
            return SI_GetEnumStringByIndex(
              impl_->handle, L"Grabber.Channels", index, channel, 1024);
          }, "SI_GetEnumStringByIndex(Grabber.Channels)");
        if (!result.success) {return fail_open(result);}
        available_channels.emplace_back(channel);
      }
      if (std::find(available_channels.begin(), available_channels.end(), requested) ==
        available_channels.end())
      {
        std::string listed;
        for (const auto & channel : available_channels) {
          if (!listed.empty()) {listed += ", ";}
          listed += "'" + narrow_ascii(channel) + "'";
        }
        return fail_open({false, "configured grabber_channel '" + narrow_ascii(requested) +
            "' was not enumerated; available channels: [" + listed + "]"});
      }
    }
    result = set_string(L"Grabber.Channel", requested, "SI_SetString(Grabber.Channel)");
    if (!result.success) {return fail_open(result);}
    SI_WC channel_readback[1024]{};
    result = locked_si([this, &channel_readback]() {
        return SI_GetString(
          impl_->handle, L"Grabber.Channel", channel_readback, 1024);
      }, "SI_GetString(Grabber.Channel)");
    if (!result.success || std::wstring(channel_readback) != requested) {
      return fail_open(result.success ? OperationResult{false,
        "Grabber.Channel readback differs from the configured headless channel"} : result);
    }
  }

  if (!std::filesystem::is_regular_file(impl_->options.calibration_pack_path)) {
    return fail_open({false, "calibration_pack_path does not name an existing file"});
  }
  result = set_string(L"Camera.CalibrationPack", impl_->options.calibration_pack_path,
      "SI_SetString(Camera.CalibrationPack)");
  if (!result.success) {return fail_open(result);}

  if (impl_->options.kind == CameraKind::fx10e) {
    result = locked_si([this]() {
        return SI_SetInt(impl_->handle, L"Pleora.Stream.PacketSize",
          static_cast<SI_64>(impl_->options.pleora_packet_size));
      }, "SI_SetInt(Pleora.Stream.PacketSize)");
    if (!result.success) {return fail_open(result);}
  } else {
    if (!std::filesystem::is_regular_file(impl_->options.ni_camera_file_path)) {
      return fail_open({false, "ni_camera_file_path does not name an existing file"});
    }
    result = set_string(L"NiImaq.CameraFile", impl_->options.ni_camera_file_path,
        "SI_SetString(NiImaq.CameraFile)");
    if (result.success) {
      result = set_string(L"Grabber.Channel", impl_->options.ni_grabber_channel,
          "SI_SetString(Grabber.Channel)");
    }
    if (result.success && !impl_->options.ni_camera_serial_port.empty()) {
      result = set_string(L"Camera.Channel", impl_->options.ni_camera_serial_port,
          "SI_SetString(Camera.Channel)");
    }
    if (!result.success) {return fail_open(result);}
  }

  SI_BOOL timeout_implemented = SI_FALSE;
  SI_BOOL timeout_writable = SI_FALSE;
  const bool can_set_timeout =
    SI_SUCCEEDED(SI_IsImplemented(impl_->handle, L"Acquisition.Timeout", &timeout_implemented)) &&
    timeout_implemented == SI_TRUE &&
    SI_SUCCEEDED(SI_IsWritable(impl_->handle, L"Acquisition.Timeout", &timeout_writable)) &&
    timeout_writable == SI_TRUE;
  if (can_set_timeout) {
    result = locked_si([this]() {
        return SI_SetFloat(impl_->handle, L"Acquisition.Timeout",
          static_cast<double>(impl_->options.initialization_timeout_ms));
      }, "SI_SetFloat(Acquisition.Timeout)");
    if (!result.success) {return fail_open(result);}
  }

  result = locked_si([this]() {return SI_Command(impl_->handle, L"Initialize");},
      "SI_Command(Initialize)");
  if (!result.success) {
    return fail_open(result);
  }

  SI_BOOL initialized = SI_FALSE;
  result = locked_si([this, &initialized]() {
      return SI_GetBool(impl_->handle, L"IsInitialized", &initialized);
    }, "SI_GetBool(IsInitialized)");
  if (!result.success || initialized != SI_TRUE) {
    return fail_open(result.success ?
      OperationResult{false, "Initialize returned OK but IsInitialized is false"} : result);
  }

  SI_BOOL calpack_implemented = SI_FALSE;
  if (SI_SUCCEEDED(SI_IsImplemented(
      impl_->handle, L"Camera.CalibrationPack.IsLoaded", &calpack_implemented)) &&
    calpack_implemented == SI_TRUE)
  {
    SI_BOOL loaded = SI_FALSE;
    result = locked_si([this, &loaded]() {
        return SI_GetBool(impl_->handle, L"Camera.CalibrationPack.IsLoaded", &loaded);
      }, "SI_GetBool(Camera.CalibrationPack.IsLoaded)");
    if (!result.success || loaded != SI_TRUE) {
      return fail_open(result.success ?
        OperationResult{false, "calibration pack readback remained unloaded"} : result);
    }
    impl_->readback.calibration_pack_loaded = true;
  }

  if (impl_->options.kind == CameraKind::fx10e) {
    SI_64 packet_size = 0;
    result = locked_si([this, &packet_size]() {
        return SI_GetInt(impl_->handle, L"Pleora.Stream.PacketSize", &packet_size);
      }, "SI_GetInt(Pleora.Stream.PacketSize)");
    if (!result.success || packet_size != static_cast<SI_64>(impl_->options.pleora_packet_size)) {
      return fail_open(result.success ?
        OperationResult{false, "Pleora packet-size readback differs from requested value"} : result);
    }
    impl_->readback.pleora_packet_size = static_cast<std::uint32_t>(packet_size);
  }

  SI_WC serial[512]{};
  result = locked_si(
    [this, &serial]() {return SI_GetString(impl_->handle, L"Sensor.SerialNumber", serial, 512);},
    "SI_GetString(Sensor.SerialNumber)");
  if (!result.success) {return fail_open(result);}
  impl_->readback.sensor_serial = narrow_ascii(serial);
  if (impl_->readback.sensor_serial != impl_->options.expected_sensor_serial) {
    return fail_open({false, "opened sensor serial does not match expected_sensor_serial; read " +
        impl_->readback.sensor_serial});
  }
  impl_->state = HsiState::connected;
  return {true, "SpecSensor profile opened and initialized"};
#endif
}

OperationResult SpecSensorHsiAdapter::configure(const HsiConfig & requested)
{
  std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
#if !PPBNG_HSI_ENABLE_SPECSENSOR
  (void)requested;
  return unavailable();
#else
  if (impl_->state != HsiState::connected && impl_->state != HsiState::configured) {
    return {false, "configure requires connected or configured state"};
  }
  if (requested.kind != impl_->options.kind) {
    return {false, "camera kind does not match backend instance"};
  }
  const auto valid = HsiFormat::validate_config(requested);
  if (!valid.success) {
    return valid;
  }
  auto result = locked_si(
    [this, &requested]() {return SI_SetFloat(impl_->handle, L"Camera.FrameRate", requested.line_rate_hz);},
    "SI_SetFloat(Camera.FrameRate)");
  if (!result.success) {impl_->state = HsiState::fault; return result;}
  const double exposure_ms = requested.exposure_us / 1000.0;
  result = locked_si(
    [this, exposure_ms]() {return SI_SetFloat(impl_->handle, L"Camera.ExposureTime", exposure_ms);},
    "SI_SetFloat(Camera.ExposureTime)");
  if (!result.success) {impl_->state = HsiState::fault; return result;}
  std::vector<SI_WC> requested_trigger_mode(
    requested.trigger_mode.begin(), requested.trigger_mode.end());
  requested_trigger_mode.push_back(L'\0');
  result = locked_si(
    [this, &requested_trigger_mode]() {
      return SI_SetEnumIndexByString(
        impl_->handle, L"Camera.Trigger.Mode", requested_trigger_mode.data());
    }, "SI_SetEnumIndexByString(Camera.Trigger.Mode)");
  if (!result.success) {impl_->state = HsiState::fault; return result;}

  double rate = 0.0;
  double exposure = 0.0;
  SI_64 width = 0;
  SI_64 height = 0;
  SI_64 byte_depth = 0;
  SI_64 frame_bytes = 0;
  int trigger_index = -1;
  SI_WC trigger_mode[128]{};
  const auto read = [this](auto function, const char * name) {
      return locked_si(function, name);
    };
  if (!(read([this, &rate]() {return SI_GetFloat(impl_->handle, L"Camera.FrameRate", &rate);},
      "SI_GetFloat(Camera.FrameRate)").success &&
    read([this, &exposure]() {return SI_GetFloat(impl_->handle, L"Camera.ExposureTime", &exposure);},
      "SI_GetFloat(Camera.ExposureTime)").success &&
    read([this, &width]() {return SI_GetInt(impl_->handle, L"Camera.Image.Width", &width);},
      "SI_GetInt(Camera.Image.Width)").success &&
    read([this, &height]() {return SI_GetInt(impl_->handle, L"Camera.Image.Height", &height);},
      "SI_GetInt(Camera.Image.Height)").success &&
    read([this, &byte_depth]() {return SI_GetInt(impl_->handle, L"Camera.ByteDepth", &byte_depth);},
      "SI_GetInt(Camera.ByteDepth)").success &&
    read([this, &frame_bytes]() {return SI_GetInt(impl_->handle, L"Camera.Image.SizeBytes", &frame_bytes);},
      "SI_GetInt(Camera.Image.SizeBytes)").success &&
    read([this, &trigger_index]() {return SI_GetEnumIndex(impl_->handle, L"Camera.Trigger.Mode", &trigger_index);},
      "SI_GetEnumIndex(Camera.Trigger.Mode)").success &&
    read([this, &trigger_index, &trigger_mode]() {
      return SI_GetEnumStringByIndex(
        impl_->handle, L"Camera.Trigger.Mode", trigger_index, trigger_mode, 128);
    }, "SI_GetEnumStringByIndex(Camera.Trigger.Mode)").success))
  {
    impl_->state = HsiState::fault;
    return {false, "SpecSensor configuration readback failed"};
  }

  const auto sensor_serial = impl_->readback.sensor_serial;
  const auto profile_name = impl_->readback.profile_name;
  const bool calibration_pack_loaded = impl_->readback.calibration_pack_loaded;
  const auto pleora_packet_size = impl_->readback.pleora_packet_size;
  impl_->readback = {true, rate, exposure * 1000.0, narrow_ascii(trigger_mode),
    static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height),
    static_cast<std::uint32_t>(byte_depth), static_cast<std::uint64_t>(frame_bytes),
    sensor_serial, profile_name, calibration_pack_loaded, pleora_packet_size};
  const auto expected_bytes = static_cast<std::uint64_t>(requested.spatial_samples) *
    requested.spectral_bands * sizeof(std::uint16_t);
  // The SWIR/NI timing registers quantize requested floating-point values to
  // hardware clock ticks. These bounds accept that sub-tick rounding while
  // still rejecting a materially different acquisition configuration.
  if (!approximately_equal(rate, requested.line_rate_hz, 0.001, 1e-5) ||
    !approximately_equal(exposure * 1000.0, requested.exposure_us, 1.0, 1e-5) ||
    impl_->readback.trigger_mode != requested.trigger_mode || width != requested.spatial_samples ||
    height != requested.spectral_bands || byte_depth != 2 ||
    frame_bytes != static_cast<SI_64>(expected_bytes) ||
    frame_bytes > static_cast<SI_64>(impl_->options.maximum_frame_bytes))
  {
    impl_->state = HsiState::fault;
    return {false, "SpecSensor applied configuration mismatch: requested rate=" +
        std::to_string(requested.line_rate_hz) + " Hz, exposure=" +
        std::to_string(requested.exposure_us) + " us, trigger=" + requested.trigger_mode +
        ", geometry=" +
        std::to_string(requested.spatial_samples) + "x" +
        std::to_string(requested.spectral_bands) + "x2 bytes, frame_bytes=" +
        std::to_string(expected_bytes) + "; actual rate=" + std::to_string(rate) +
        " Hz, exposure=" + std::to_string(exposure * 1000.0) + " us, trigger=" +
        impl_->readback.trigger_mode + ", geometry=" + std::to_string(width) + "x" +
        std::to_string(height) + "x" + std::to_string(byte_depth) +
        " bytes, frame_bytes=" + std::to_string(frame_bytes)};
  }
  impl_->config = requested;
  impl_->config.line_rate_hz = rate;
  impl_->config.exposure_us = exposure * 1000.0;
  impl_->state = HsiState::configured;
  return {true, "SpecSensor configuration applied and read back"};
#endif
}

OperationResult SpecSensorHsiAdapter::close_shutter()
{
  std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
#if !PPBNG_HSI_ENABLE_SPECSENSOR
  return unavailable();
#else
  if (impl_->state != HsiState::configured && impl_->state != HsiState::ready) {
    return {false, "close_shutter requires configured or ready state"};
  }
  auto result = locked_si(
    [this]() {return SI_Command(impl_->handle, L"Camera.CloseShutter");},
    "SI_Command(Camera.CloseShutter)");
  SI_BOOL open = SI_TRUE;
  if (result.success) {
    result = locked_si(
      [this, &open]() {return SI_GetBool(impl_->handle, L"Camera.Shutter.IsOpen", &open);},
      "SI_GetBool(Camera.Shutter.IsOpen)");
  }
  if (!result.success || open == SI_TRUE) {impl_->state = HsiState::fault; return {false,
      result.success ? "shutter close readback remained open" : result.message};}
  impl_->state = HsiState::shutter_closed;
  return {true, "shutter closed and read back"};
#endif
}

OperationResult SpecSensorHsiAdapter::begin_dark_capture(const std::size_t line_count)
{
  std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
#if !PPBNG_HSI_ENABLE_SPECSENSOR
  (void)line_count;
  return unavailable();
#else
  if (impl_->state != HsiState::shutter_closed || line_count == 0U) {
    return {false, "dark capture requires closed shutter and nonzero line count"};
  }
  impl_->queue.clear();
  impl_->dark_lines_remaining = line_count;
  auto result = locked_si(
    [this]() {return SI_RegisterDataCallback(impl_->handle, data_callback, &impl_->queue);},
    "SI_RegisterDataCallback");
  if (!result.success) {impl_->state = HsiState::fault; return result;}
  impl_->callback_registered = true;
  result = locked_si(
    [this]() {return SI_Command(impl_->handle, L"Acquisition.Start");},
    "SI_Command(Acquisition.Start)");
  if (!result.success) {impl_->state = HsiState::fault; return result;}
  impl_->state = HsiState::dark_collecting;
  return {true, impl_->config.trigger_mode == "Internal" ?
    "dark acquisition started with internal line timing" :
    "dark acquisition armed; waiting for external trigger lines"};
#endif
}

OperationResult SpecSensorHsiAdapter::open_shutter()
{
  std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
#if !PPBNG_HSI_ENABLE_SPECSENSOR
  return unavailable();
#else
  if (impl_->state != HsiState::shutter_closed || impl_->dark_lines_remaining != 0U) {
    return {false, "open_shutter requires completed dark capture"};
  }
  auto result = locked_si(
    [this]() {return SI_Command(impl_->handle, L"Camera.OpenShutter");},
    "SI_Command(Camera.OpenShutter)");
  SI_BOOL open = SI_FALSE;
  if (result.success) {
    result = locked_si(
      [this, &open]() {return SI_GetBool(impl_->handle, L"Camera.Shutter.IsOpen", &open);},
      "SI_GetBool(Camera.Shutter.IsOpen)");
  }
  if (!result.success || open == SI_FALSE) {impl_->state = HsiState::fault; return {false,
      result.success ? "shutter open readback remained closed" : result.message};}
  impl_->state = HsiState::ready;
  return {true, "shutter opened and read back"};
#endif
}

OperationResult SpecSensorHsiAdapter::start_streaming()
{
  std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
#if !PPBNG_HSI_ENABLE_SPECSENSOR
  return unavailable();
#else
  if (impl_->state != HsiState::ready) {
    return {false, "start_streaming requires ready state"};
  }
  // Acquisition.Stop at the end of dark capture causes the camera SDK frame
  // counter to restart for the scene acquisition. Preserve that truth as a
  // new logical/source segment instead of placing a reset counter in the dark
  // segment. The increment is applied lazily when the first scene frame arrives.
  if (impl_->segment_line_index != 0U) {impl_->segment_boundary_pending = true;}
  impl_->queue.clear();
  auto result = locked_si(
    [this]() {return SI_RegisterDataCallback(impl_->handle, data_callback, &impl_->queue);},
    "SI_RegisterDataCallback");
  if (!result.success) {impl_->state = HsiState::fault; return result;}
  impl_->callback_registered = true;
  result = locked_si(
    [this]() {return SI_Command(impl_->handle, L"Acquisition.Start");},
    "SI_Command(Acquisition.Start)");
  if (!result.success) {impl_->state = HsiState::fault; return result;}
  impl_->previous_trigger_sequence = 0U;
  impl_->previous_sdk_frame_number = -1;
  impl_->state = HsiState::streaming;
  return {true, impl_->config.trigger_mode == "Internal" ?
    "continuous sample acquisition started with internal line timing" :
    "sample acquisition armed for external triggers"};
#endif
}

OperationResult SpecSensorHsiAdapter::stop_streaming()
{
  std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
#if !PPBNG_HSI_ENABLE_SPECSENSOR
  return unavailable();
#else
  if (impl_->state != HsiState::streaming && impl_->state != HsiState::dark_collecting) {
    return {true, "acquisition already stopped"};
  }
  auto result = locked_si(
    [this]() {return SI_Command(impl_->handle, L"Acquisition.Stop");},
    "SI_Command(Acquisition.Stop)");
  if (impl_->callback_registered) {
    const auto unregister_result = locked_si(
      [this]() {return SI_UnregisterDataCallback(impl_->handle);},
      "SI_UnregisterDataCallback");
    impl_->callback_registered = false;
    if (result.success && !unregister_result.success) {result = unregister_result;}
  }
  impl_->queue.clear();
  if (!result.success) {impl_->state = HsiState::fault; return result;}
  const bool dark_completed = impl_->state == HsiState::dark_collecting &&
    impl_->dark_lines_remaining == 0U;
  impl_->state = dark_completed ? HsiState::shutter_closed : HsiState::ready;
  return {true, "acquisition stopped and callback unregistered"};
#endif
}

OperationResult SpecSensorHsiAdapter::recover()
{
  std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
#if !PPBNG_HSI_ENABLE_SPECSENSOR
  return unavailable();
#else
  if (impl_->state == HsiState::disconnected || !impl_->readback.valid) {
    return {false, "recover requires a previously connected and read-back configuration"};
  }
  const auto saved = impl_->config;
  impl_->state = HsiState::recovering;
  if (impl_->callback_registered) {
    (void)locked_si([this]() {return SI_UnregisterDataCallback(impl_->handle);},
        "SI_UnregisterDataCallback");
    impl_->callback_registered = false;
  }
  if (impl_->handle != nullptr) {
    (void)locked_si([this]() {return SI_Close(impl_->handle);}, "SI_Close");
    impl_->handle = nullptr;
  }
  if (impl_->sdk_acquired) {(void)release_sdk(); impl_->sdk_acquired = false;}
  impl_->state = HsiState::disconnected;
  auto result = connect();
  if (result.success) {result = configure(saved);}
  if (result.success) {result = close_shutter();}
  if (result.success) {
    impl_->dark_lines_remaining = 0U;
    result = open_shutter();
  }
  if (!result.success) {impl_->state = HsiState::fault; return result;}
  impl_->segment_boundary_pending = true;
  impl_->previous_trigger_sequence = 0U;
  impl_->previous_sdk_frame_number = -1;
  return {true, "SpecSensor instance recovered; caller must start the new segment"};
#endif
}

LineResult SpecSensorHsiAdapter::on_trigger(const TriggerEvent & trigger)
{
  std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
#if !PPBNG_HSI_ENABLE_SPECSENSOR
  (void)trigger;
  return {LineStatus::fault, std::nullopt, unavailable().message};
#else
  if (impl_->state != HsiState::streaming && impl_->state != HsiState::dark_collecting) {
    return {LineStatus::not_ready, std::nullopt, "acquisition is not active"};
  }
  if (impl_->config.trigger_mode != "External") {
    return {LineStatus::invalid_trigger, std::nullopt,
      "external trigger events are invalid in Internal mode"};
  }
  if (trigger.channel != impl_->config.trigger_channel || trigger.ticks_per_second == 0U ||
    (trigger.time_status == TimeStatus::locked &&
    trigger.offset_ticks >= trigger.ticks_per_second) ||
    (impl_->previous_trigger_sequence != 0U &&
    trigger.channel_sequence <= impl_->previous_trigger_sequence))
  {
    return {LineStatus::invalid_trigger, std::nullopt, "invalid HSI trigger event"};
  }
  SpecSensorFrame frame;
  if (!impl_->queue.try_pop(frame)) {
    return {LineStatus::not_ready, std::nullopt, "trigger has no corresponding SDK frame yet"};
  }
  const auto expected_bytes = HsiFormat::payload_bytes_per_line(impl_->config);
  if (frame.bytes.size() != expected_bytes || (frame.bytes.size() % sizeof(std::uint16_t)) != 0U) {
    impl_->state = HsiState::fault;
    return {LineStatus::fault, std::nullopt, "SDK frame size differs from configuration readback"};
  }

  if (impl_->segment_boundary_pending) {
    ++impl_->segment;
    impl_->segment_line_index = 0U;
    impl_->segment_boundary_pending = false;
  }

  LineRecord record;
  record.device_id = impl_->config.device_id;
  record.camera_kind = impl_->options.kind;
  record.trigger = trigger;
  record.index.segment_id = impl_->segment;
  record.index.capture_kind = impl_->state == HsiState::dark_collecting ?
    CaptureKind::dark : CaptureKind::sample;
  record.index.segment_line_index = impl_->segment_line_index++;
  record.index.camera_line_sequence = static_cast<std::uint64_t>(frame.sdk_frame_number);
  record.index.trigger_sequence = trigger.channel_sequence;
  record.index.pps_sequence = trigger.pps_sequence;
  record.index.utc_time_ns = trigger.utc_time_ns;
  record.index.time_status = trigger.time_status;
  record.index.uncertainty_ns = trigger.uncertainty_ns;
  record.index.host_receive_monotonic_ns = frame.host_receive_monotonic_ns;
  record.index.raw_file_offset_bytes = record.index.segment_line_index * expected_bytes;
  record.index.payload_size_bytes = expected_bytes;
  if (impl_->previous_sdk_frame_number >= 0 &&
    frame.sdk_frame_number > impl_->previous_sdk_frame_number + 1)
  {
    record.index.sequence_gap_before = true;
    record.index.missing_trigger_count = static_cast<std::uint64_t>(
      frame.sdk_frame_number - impl_->previous_sdk_frame_number - 1);
  }
  impl_->previous_sdk_frame_number = frame.sdk_frame_number;
  impl_->previous_trigger_sequence = trigger.channel_sequence;
  record.pixels.resize(frame.bytes.size() / sizeof(std::uint16_t));
  std::memcpy(record.pixels.data(), frame.bytes.data(), frame.bytes.size());

  if (impl_->state == HsiState::dark_collecting && --impl_->dark_lines_remaining == 0U) {
    const auto stopped = stop_streaming();
    if (!stopped.success) {
      return {LineStatus::fault, std::nullopt, stopped.message};
    }
  }
  return {LineStatus::produced, std::move(record), "SpecSensor frame associated with trigger"};
#endif
}

LineResult SpecSensorHsiAdapter::poll_internal()
{
  std::lock_guard<std::recursive_mutex> lock(impl_->mutex);
#if !PPBNG_HSI_ENABLE_SPECSENSOR
  return {LineStatus::fault, std::nullopt, unavailable().message};
#else
  if (impl_->state != HsiState::streaming && impl_->state != HsiState::dark_collecting) {
    return {LineStatus::not_ready, std::nullopt, "acquisition is not active"};
  }
  if (impl_->config.trigger_mode != "Internal") {
    return {LineStatus::invalid_trigger, std::nullopt,
      "internal polling is invalid in External mode"};
  }
  SpecSensorFrame frame;
  if (!impl_->queue.try_pop(frame)) {
    return {LineStatus::not_ready, std::nullopt, "no SDK frame is queued"};
  }
  const auto expected_bytes = HsiFormat::payload_bytes_per_line(impl_->config);
  if (frame.bytes.size() != expected_bytes ||
    (frame.bytes.size() % sizeof(std::uint16_t)) != 0U)
  {
    impl_->state = HsiState::fault;
    return {LineStatus::fault, std::nullopt,
      "SDK frame size differs from configuration readback"};
  }

  if (impl_->segment_boundary_pending) {
    ++impl_->segment;
    impl_->segment_line_index = 0U;
    impl_->segment_boundary_pending = false;
  }

  LineRecord record;
  record.device_id = impl_->config.device_id;
  record.camera_kind = impl_->options.kind;
  record.trigger.channel = "internal:" + impl_->config.device_id;
  record.trigger.time_status = TimeStatus::unsynced;
  record.index.segment_id = impl_->segment;
  record.index.sdk_segment_id = impl_->segment;
  record.index.capture_kind = impl_->state == HsiState::dark_collecting ?
    CaptureKind::dark : CaptureKind::sample;
  record.index.segment_line_index = impl_->segment_line_index++;
  record.index.camera_line_sequence = static_cast<std::uint64_t>(frame.sdk_frame_number);
  record.index.time_status = TimeStatus::unsynced;
  record.index.host_receive_monotonic_ns = frame.host_receive_monotonic_ns;
  record.index.raw_file_offset_bytes = record.index.segment_line_index * expected_bytes;
  record.index.payload_size_bytes = expected_bytes;
  record.index.association_status = AssociationStatus::unverified;
  if (impl_->previous_sdk_frame_number >= 0 &&
    frame.sdk_frame_number > impl_->previous_sdk_frame_number + 1)
  {
    record.index.sequence_gap_before = true;
    record.index.missing_trigger_count = static_cast<std::uint64_t>(
      frame.sdk_frame_number - impl_->previous_sdk_frame_number - 1);
  }
  impl_->previous_sdk_frame_number = frame.sdk_frame_number;
  record.pixels.resize(frame.bytes.size() / sizeof(std::uint16_t));
  std::memcpy(record.pixels.data(), frame.bytes.data(), frame.bytes.size());

  if (impl_->state == HsiState::dark_collecting && --impl_->dark_lines_remaining == 0U) {
    const auto stopped = stop_streaming();
    if (!stopped.success) {
      return {LineStatus::fault, std::nullopt, stopped.message};
    }
  }
  return {LineStatus::produced, std::move(record),
    "SpecSensor internally timed frame received"};
#endif
}

}  // namespace ppbng_hsi
