#include "ppbng_runtime/hardware_safety_gate.hpp"
#include "ppbng_runtime/production_workflow.hpp"
#include "ppbng_runtime/storage_capacity_guard.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ppbng_interfaces/msg/acquisition_status.hpp>
#include <ppbng_interfaces/msg/device_status.hpp>
#include <ppbng_interfaces/msg/fault_event.hpp>
#include <ppbng_interfaces/msg/trigger_event.hpp>
#include <ppbng_interfaces/srv/abort_acquisition.hpp>
#include <ppbng_interfaces/srv/confirm_dark_ready.hpp>
#include <ppbng_interfaces/srv/confirm_sample_ready.hpp>
#include <ppbng_interfaces/srv/prepare_device.hpp>
#include <ppbng_interfaces/srv/start_acquisition.hpp>
#include <ppbng_interfaces/srv/stop_acquisition.hpp>
#include <ppbng_storage/dataset_session.hpp>
#include <ppbng_storage/session_manifest.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace ppbng_runtime
{
using namespace std::chrono_literals;

namespace
{
std::string operation_name(const DeviceOperation operation)
{
  switch (operation) {
    case DeviceOperation::prepare: return "prepare";
    case DeviceOperation::arm: return "arm";
    case DeviceOperation::start: return "start";
    case DeviceOperation::begin_dark: return "begin_dark";
    case DeviceOperation::start_sample: return "start_sample";
    case DeviceOperation::arm_next_pps: return "arm_next_pps";
    case DeviceOperation::arm_immediate: return "arm_immediate";
    case DeviceOperation::disarm_keep_configuration: return "disarm_keep_config";
    case DeviceOperation::stop: return "stop";
  }
  return "invalid";
}

std::string endpoint(const DeviceAction & action)
{
  return "/" + action.device + "/" + operation_name(action.operation);
}

std::uint8_t status_state(const ProductionWorkflowState state)
{
  return state == ProductionWorkflowState::fault ? 9U : static_cast<std::uint8_t>(state);
}

void upsert_setting(
  std::vector<std::pair<std::string, std::string>> & settings,
  const std::string & key, const std::string & value)
{
  const auto found = std::find_if(settings.begin(), settings.end(),
    [&key](const auto & item) {return item.first == key;});
  if (found == settings.end()) {settings.emplace_back(key, value);}
  else {found->second = value;}
}
}  // namespace

class ProductionAcquisitionManagerNode final : public rclcpp::Node
{
public:
  ProductionAcquisitionManagerNode()
  : Node("production_acquisition_manager"),
    workflow_(declare_parameter<bool>("pps_required", true))
  {
    HardwareSafetyInputs safety;
    safety.hardware_enabled = declare_parameter<bool>("hardware_enabled", false);
    safety.launch_authorized = declare_parameter<bool>("launch_authorized", false);
    safety.machine_config_path = declare_parameter<std::string>("machine_config_path", "");
    safety.machine_id = declare_parameter<std::string>("machine_id", "");
    machine_config_path_ = safety.machine_config_path;
    machine_config_snapshot_ = declare_parameter<std::string>(
      "machine_config_snapshot", "");
    machine_id_ = safety.machine_id;
    output_root_ = std::filesystem::u8path(declare_parameter<std::string>("output_root", ""));
    const auto add_device = [this](std::string role, std::string status_device_id,
      std::string backend, std::string model, std::string expected_identity) {
        if (!status_device_id.empty() &&
          !status_role_by_device_id_.emplace(status_device_id, role).second)
        {
          throw std::runtime_error("duplicate production DeviceStatus device_id mapping");
        }
        device_descriptors_.push_back({std::move(role), std::move(status_device_id),
          std::move(backend), std::move(model), std::move(expected_identity)});
      };
    add_device("fx10e", "fx10e", "SpecSensor/Pleora", "Specim FX10e",
      declare_parameter<std::string>("fx10e_expected_identity", ""));
    add_device("swir", "swir", "SpecSensor/NI", "Specim SWIR",
      declare_parameter<std::string>("swir_expected_identity", ""));
    const auto rgb_identity = declare_parameter<std::string>("rgb_expected_identity", "");
    add_device("rgb", declare_parameter<std::string>("rgb_status_device_id", rgb_identity),
      "Spinnaker", "FLIR Blackfly-S BFS-U3-122S6C-C", rgb_identity);
    const auto thermal_identity = declare_parameter<std::string>(
      "thermal_expected_identity", "");
    add_device("thermal",
      declare_parameter<std::string>("thermal_status_device_id", thermal_identity),
      "Spinnaker", "FLIR A6701", thermal_identity);
    add_device("gnss", "gnss", "Win32 serial receive-only", "Unicore UM982", "");
    add_device("rsm400", "rsm400", "Win32 serial", "SOMAG RSM400", "");
    add_device("timing", "timing", "Win32 serial", "PPBNG timing controller", "");
    add_device("context", "", "ROS 2 association", "FrameContext", "");
    const auto timeout_ms = declare_parameter<std::int64_t>("batch_timeout_ms", 5000);
    if (timeout_ms <= 0) {throw std::runtime_error("batch_timeout_ms must be positive");}
    batch_timeout_ = std::chrono::milliseconds(timeout_ms);
    const auto estimated_rate = declare_parameter<std::int64_t>(
      "estimated_bytes_per_second", 0);
    const auto planned_duration = declare_parameter<std::int64_t>(
      "planned_duration_seconds", 7200);
    const auto reserve_bytes = declare_parameter<std::int64_t>(
      "minimum_reserve_bytes", 107374182400LL);
    const auto headroom = declare_parameter<std::int64_t>(
      "capacity_headroom_basis_points", 2500);
    const auto disk_interval_ms = declare_parameter<std::int64_t>(
      "disk_check_interval_ms", 1000);
    if (estimated_rate <= 0 || planned_duration <= 0 || reserve_bytes < 0 ||
      headroom < 0 || headroom > 10'000 || disk_interval_ms <= 0)
    {
      throw std::runtime_error("invalid production storage-capacity policy");
    }
    storage_policy_.estimated_bytes_per_second = static_cast<std::uint64_t>(estimated_rate);
    storage_policy_.planned_duration_seconds = static_cast<std::uint64_t>(planned_duration);
    storage_policy_.minimum_reserve_bytes = static_cast<std::uint64_t>(reserve_bytes);
    storage_policy_.headroom_basis_points = static_cast<std::uint32_t>(headroom);
    if (!evaluate_storage_capacity(storage_policy_,
      (std::numeric_limits<std::uint64_t>::max)(), 0U).valid)
    {
      throw std::runtime_error("storage-capacity policy arithmetic is not representable");
    }
    throughput_qualified_ = declare_parameter<bool>("throughput_qualified", false);
    const auto qualified_rate = declare_parameter<std::int64_t>(
      "qualified_durable_bytes_per_second", 0);
    if (qualified_rate < 0) throw std::runtime_error("qualified durable rate must be nonnegative");
    qualified_durable_bytes_per_second_ = static_cast<std::uint64_t>(qualified_rate);
    throughput_qualification_detail_ = declare_parameter<std::string>(
      "throughput_qualification_detail", "not qualified");
    throughput_qualification_output_root_ = std::filesystem::u8path(
      declare_parameter<std::string>("throughput_qualification_output_root", ""));
    const auto decision = evaluate_hardware_safety(safety);
    if (!decision.allowed) {throw std::runtime_error("production manager refused: " + decision.reason);}
    std::ifstream config_stream(machine_config_path_, std::ios::binary);
    std::ostringstream current_config;
    current_config << config_stream.rdbuf();
    if (!config_stream.is_open() || machine_config_snapshot_.empty()) {
      throw std::runtime_error("production manager refused: machine configuration snapshot failed");
    }
    if (current_config.str() != machine_config_snapshot_) {
      throw std::runtime_error(
              "production manager refused: machine configuration changed during launch handoff");
    }
    std::error_code error;
    if (output_root_.empty() || !std::filesystem::is_directory(output_root_, error) || error) {
      throw std::runtime_error("production manager refused: output_root must already be a directory");
    }

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    status_publisher_ = create_publisher<ppbng_interfaces::msg::AcquisitionStatus>(
      "/acquisition/status", rclcpp::QoS(1).reliable().transient_local());
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = callback_group_;
    device_status_subscription_ = create_subscription<ppbng_interfaces::msg::DeviceStatus>(
      "/acquisition/device_status", 100,
      [this](const ppbng_interfaces::msg::DeviceStatus & message) {observe_device_status(message);},
      subscription_options);
    fault_subscription_ = create_subscription<ppbng_interfaces::msg::FaultEvent>(
      "/acquisition/fault_event", 100,
      [this](const ppbng_interfaces::msg::FaultEvent & message) {observe_fault(message);},
      subscription_options);
    trigger_subscription_ = create_subscription<ppbng_interfaces::msg::TriggerEvent>(
      "/acquisition/trigger", 100,
      [this](const ppbng_interfaces::msg::TriggerEvent &) {observe_trigger();},
      subscription_options);

    start_service_ = create_service<ppbng_interfaces::srv::StartAcquisition>(
      "/acquisition/start", [this](
        std::shared_ptr<ppbng_interfaces::srv::StartAcquisition::Request> request,
        std::shared_ptr<ppbng_interfaces::srv::StartAcquisition::Response> response) {
        start(request, response);
      },
      rmw_qos_profile_services_default, callback_group_);
    dark_service_ = create_service<ppbng_interfaces::srv::ConfirmDarkReady>(
      "/acquisition/confirm_dark_ready", [this](
        std::shared_ptr<ppbng_interfaces::srv::ConfirmDarkReady::Request> request,
        std::shared_ptr<ppbng_interfaces::srv::ConfirmDarkReady::Response> response) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        command(request->request_id, "dark", [this]() {return workflow_.confirm_dark_cover();},
          *response);
      }, rmw_qos_profile_services_default, callback_group_);
    sample_service_ = create_service<ppbng_interfaces::srv::ConfirmSampleReady>(
      "/acquisition/confirm_sample_ready", [this](
        std::shared_ptr<ppbng_interfaces::srv::ConfirmSampleReady::Request> request,
        std::shared_ptr<ppbng_interfaces::srv::ConfirmSampleReady::Response> response) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        command(request->request_id, "sample", [this]() {return workflow_.confirm_sample_ready();},
          *response);
      }, rmw_qos_profile_services_default, callback_group_);
    stop_service_ = create_service<ppbng_interfaces::srv::StopAcquisition>(
      "/acquisition/stop", [this](
        std::shared_ptr<ppbng_interfaces::srv::StopAcquisition::Request> request,
        std::shared_ptr<ppbng_interfaces::srv::StopAcquisition::Response> response) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        command(request->request_id, "stop:" + request->reason,
          [this, reason = request->reason]() {return workflow_.request_stop(reason);}, *response);
      }, rmw_qos_profile_services_default, callback_group_);
    abort_service_ = create_service<ppbng_interfaces::srv::AbortAcquisition>(
      "/acquisition/abort", [this](
        std::shared_ptr<ppbng_interfaces::srv::AbortAcquisition::Request> request,
        std::shared_ptr<ppbng_interfaces::srv::AbortAcquisition::Response> response) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        command(request->request_id, "abort:" + request->reason,
          [this, reason = request->reason]() {
            return workflow_.report_global_fault("operator abort: " + reason);
          }, *response);
      }, rmw_qos_profile_services_default, callback_group_);
    timeout_timer_ = create_wall_timer(100ms, [this]() {check_timeout();}, callback_group_);
    disk_timer_ = create_wall_timer(std::chrono::milliseconds(disk_interval_ms),
      [this]() {check_disk_capacity();}, callback_group_);
    publish_status("production manager ready; all device processes remain inert");
  }

private:
  struct CommandRecord
  {
    std::string payload;
    bool accepted{};
    std::string session_id;
    std::string session_directory;
    std::string message;
  };
  struct SessionRecord
  {
    ppbng_storage::DatasetSessionPaths paths;
    ppbng_storage::SessionManifest manifest;
    std::uint64_t revision{};
  };
  struct DeviceDescriptor
  {
    std::string role;
    std::string status_device_id;
    std::string backend;
    std::string model;
    std::string expected_identity;
  };
  struct ActiveBatch
  {
    std::uint64_t id{};
    std::size_t expected{};
    std::chrono::steady_clock::time_point deadline;
    std::vector<ActionOutcome> outcomes;
  };

  template<typename ResponseT>
  void fill_response(const CommandRecord & record, const bool duplicate, ResponseT & response)
  {
    response.accepted = record.accepted;
    response.duplicate_request = duplicate;
    response.session_id = record.session_id;
    response.message = record.message;
    if constexpr (std::is_same_v<ResponseT,
      ppbng_interfaces::srv::StartAcquisition::Response>)
    {
      response.session_directory = record.session_directory;
    } else if constexpr (!std::is_same_v<ResponseT,
      ppbng_interfaces::srv::StopAcquisition::Response>)
    {
      response.resulting_state = status_state(workflow_.state());
    }
  }

  template<typename OperationT, typename ResponseT>
  void command(
    const std::string & request_id, const std::string & payload, OperationT operation,
    ResponseT & response)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const auto existing = history_.find(request_id);
    if (request_id.empty()) {
      fill_response(CommandRecord{payload, false, workflow_.session_id(), session_directory_,
        "request_id is required"}, false, response);
      return;
    }
    if (existing != history_.end()) {
      if (existing->second.payload != payload) {
        fill_response(CommandRecord{payload, false, workflow_.session_id(), session_directory_,
          "request_id payload mismatch"}, false, response);
      } else {
        fill_response(existing->second, true, response);
      }
      return;
    }
    auto result = operation();
    CommandRecord record{payload, result.accepted, workflow_.session_id(), session_directory_,
      result.detail};
    history_.emplace(request_id, record);
    fill_response(record, false, response);
    process_result(std::move(result));
  }

  void start(
    const std::shared_ptr<ppbng_interfaces::srv::StartAcquisition::Request> request,
    std::shared_ptr<ppbng_interfaces::srv::StartAcquisition::Response> response)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const auto payload = request->dataset_name + (request->force_degraded ? ":degraded" : ":strict");
    const auto existing = history_.find(request->request_id);
    if (request->request_id.empty()) {
      fill_response(CommandRecord{payload, false, {}, {}, "request_id is required"}, false, *response);
      return;
    }
    if (existing != history_.end()) {
      if (existing->second.payload != payload) {
        fill_response(CommandRecord{payload, false, workflow_.session_id(), session_directory_,
          "request_id payload mismatch"}, false, *response);
      } else {fill_response(existing->second, true, *response);}
      return;
    }
    if (workflow_.state() != ProductionWorkflowState::inert) {
      CommandRecord record{payload, false, workflow_.session_id(), session_directory_,
        "production workflow is not inert"};
      history_.emplace(request->request_id, record);
      fill_response(record, false, *response);
      return;
    }
    warnings_.clear();
    warning_indices_.clear();
    std::string qualification_error;
    if (!throughput_qualification_valid(qualification_error)) {
      CommandRecord record{payload, false, {}, {},
        "durable-write qualification refused start: " + qualification_error};
      history_.emplace(request->request_id, record);
      fill_response(record, false, *response);
      return;
    }
    std::uint64_t available_bytes{};
    std::string capacity_error;
    if (!read_available_space(available_bytes, capacity_error)) {
      CommandRecord record{payload, false, {}, {},
        "storage capacity query failed: " + capacity_error};
      history_.emplace(request->request_id, record);
      fill_response(record, false, *response);
      return;
    }
    const auto capacity = evaluate_storage_capacity(storage_policy_, available_bytes, 0U);
    if (!capacity.valid || !capacity.sufficient) {
      std::ostringstream message;
      message << capacity.detail << "; available=" << available_bytes <<
        "; required=" << capacity.required_available_bytes;
      CommandRecord record{payload, false, {}, {}, message.str()};
      history_.emplace(request->request_id, record);
      fill_response(record, false, *response);
      return;
    }
    const auto identity = ppbng_storage::make_session_identity_now();
    auto created = ppbng_storage::create_dataset_session(output_root_, request->dataset_name, identity);
    if (!created.ok()) {
      CommandRecord record{payload, false, {}, {}, "session creation failed: " + created.detail};
      history_.emplace(request->request_id, record);
      fill_response(record, false, *response);
      return;
    }
    SessionRecord session;
    session.paths = created.paths;
    session.manifest.session_id = identity.unique_id;
    session.manifest.user_dataset_name = request->dataset_name;
    session.manifest.machine_id = machine_id_;
    session.manifest.configuration_snapshot = "machine_config.snapshot.yaml";
    session.manifest.created_utc = identity.utc_timestamp;
    session.manifest.hardware_enabled = true;
    session.manifest.storage.planned_duration_seconds = storage_policy_.planned_duration_seconds;
    session.manifest.storage.estimated_bytes_per_second =
      storage_policy_.estimated_bytes_per_second;
    session.manifest.storage.capacity_headroom_basis_points =
      storage_policy_.headroom_basis_points;
    session.manifest.storage.minimum_reserve_bytes = storage_policy_.minimum_reserve_bytes;
    session.manifest.storage.start_available_valid = true;
    session.manifest.storage.start_available_bytes = available_bytes;
    session.manifest.storage.throughput_qualified = throughput_qualified_;
    session.manifest.storage.qualified_durable_bytes_per_second =
      qualified_durable_bytes_per_second_;
    session.manifest.storage.throughput_qualification_detail = throughput_qualification_detail_;
    for (const auto & descriptor : device_descriptors_) {
      ppbng_storage::DeviceManifest device;
      device.role = descriptor.role;
      device.backend = descriptor.backend;
      device.model = descriptor.model;
      device.requested_settings = {{"configuration_source", "machine_config.snapshot.yaml"}};
      if (!descriptor.expected_identity.empty()) {
        device.requested_settings.emplace_back(
          "expected_identity", descriptor.expected_identity);
      }
      session.manifest.devices.push_back(std::move(device));
    }
    const auto snapshot_result = ppbng_storage::atomic_write_text(
      created.paths.session_directory, session.manifest.configuration_snapshot,
      machine_config_snapshot_, "machine_config_snapshot");
    if (!snapshot_result.ok()) {
      CommandRecord record{payload, false, identity.unique_id,
        created.paths.session_directory.u8string(),
        "machine configuration snapshot write failed; no device request was sent: " +
        snapshot_result.detail};
      history_.emplace(request->request_id, record);
      fill_response(record, false, *response);
      return;
    }
    sessions_.emplace(identity.unique_id, std::move(session));
    task_started_at_ = std::chrono::steady_clock::now();
    disk_fault_reported_ = false;
    session_directory_ = created.paths.session_directory.u8string();
    if (!write_manifest(identity.unique_id, ppbng_storage::ManifestState::created)) {
      CommandRecord record{payload, false, identity.unique_id, session_directory_,
        "initial manifest write failed; no device request was sent"};
      history_.emplace(request->request_id, record);
      fill_response(record, false, *response);
      return;
    }
    auto result = workflow_.begin(identity.unique_id);
    CommandRecord record{payload, result.accepted, identity.unique_id, session_directory_, result.detail};
    history_.emplace(request->request_id, record);
    fill_response(record, false, *response);
    process_result(std::move(result));
  }

  bool write_manifest(const std::string & session_id, const ppbng_storage::ManifestState state)
  {
    const auto found = sessions_.find(session_id);
    if (found == sessions_.end()) {return false;}
    auto & value = found->second;
    value.manifest.state = state;
    if (state == ppbng_storage::ManifestState::faulted ||
      state == ppbng_storage::ManifestState::finalized)
    {
      if (value.manifest.finalized_utc.empty()) {
        value.manifest.finalized_utc = ppbng_storage::make_session_identity_now().utc_timestamp;
      }
      std::uint64_t available{};
      std::string capacity_error;
      value.manifest.storage.final_available_valid =
        read_available_space(available, capacity_error);
      value.manifest.storage.final_available_bytes = available;
      std::uint64_t dataset_bytes{};
      value.manifest.storage.actual_dataset_bytes_valid =
        directory_size(value.paths.session_directory, dataset_bytes);
      value.manifest.storage.actual_dataset_bytes = dataset_bytes;
    }
    value.manifest.warnings = warnings_;
    const auto serialized = ppbng_storage::serialize_manifest_json(value.manifest);
    if (!serialized.valid) {return false;}
    return ppbng_storage::atomic_write_text(value.paths.session_directory, "manifest.json",
      serialized.json, "production_manifest_" + std::to_string(value.revision++)).ok();
  }

  static bool directory_size(const std::filesystem::path & directory,
    std::uint64_t & total) noexcept
  {
    total = 0U;
    std::error_code error;
    std::filesystem::recursive_directory_iterator iterator(
      directory, std::filesystem::directory_options::skip_permission_denied, error);
    const std::filesystem::recursive_directory_iterator end;
    if (error) return false;
    for (; iterator != end; iterator.increment(error)) {
      if (error) return false;
      const auto status = iterator->symlink_status(error);
      if (error) return false;
      if (!std::filesystem::is_regular_file(status)) continue;
      const auto size = iterator->file_size(error);
      if (error || total > (std::numeric_limits<std::uint64_t>::max)() - size) return false;
      total += size;
    }
    return true;
  }

  void process_result(WorkflowResult result)
  {
    if (!result.batch.actions.empty()) {dispatch(result.batch);}
    if (workflow_.state() == ProductionWorkflowState::recording) {
      if (!write_manifest(workflow_.session_id(), ppbng_storage::ManifestState::recording)) {
        upsert_warning("manager:recording_manifest", "recording manifest update failed");
        process_result(workflow_.report_global_fault("recording manifest update failed"));
        return;
      }
    } else if (workflow_.state() == ProductionWorkflowState::inert) {
      const auto found = sessions_.find(last_session_id_);
      if (found != sessions_.end()) found->second.manifest.storage.stop_reason = result.detail;
      (void)write_manifest(last_session_id_, ppbng_storage::ManifestState::finalized);
    } else if (workflow_.state() == ProductionWorkflowState::fault && result.batch.actions.empty()) {
      const auto found = sessions_.find(workflow_.session_id());
      if (found != sessions_.end()) found->second.manifest.storage.stop_reason = result.detail;
      if (!write_manifest(workflow_.session_id(), ppbng_storage::ManifestState::faulted)) {
        upsert_warning("manager:fault_manifest", "fault manifest update failed");
      }
    }
    if (!workflow_.session_id().empty()) {last_session_id_ = workflow_.session_id();}
    publish_status(result.detail);
  }

  void dispatch(const WorkflowBatch & batch)
  {
    if (active_batch_.expected != 0U) {
      RCLCPP_ERROR(get_logger(), "refusing overlapping workflow batches");
      return;
    }
    active_batch_ = {batch.id, batch.actions.size(), std::chrono::steady_clock::now() +
      batch_timeout_, {}};
    for (const auto & action : batch.actions) {
      if (action.operation == DeviceOperation::prepare) {dispatch_prepare(batch.id, action);}
      else {dispatch_trigger(batch.id, action);}
    }
  }

  void dispatch_prepare(const std::uint64_t batch_id, const DeviceAction action)
  {
    auto & client = prepare_clients_[action.device];
    if (!client) {client = create_client<ppbng_interfaces::srv::PrepareDevice>(
        endpoint(action), rmw_qos_profile_services_default, callback_group_);}
    auto request = std::make_shared<ppbng_interfaces::srv::PrepareDevice::Request>();
    request->request_id = workflow_.session_id() + ":prepare:" + action.device;
    request->session_id = workflow_.session_id();
    request->session_directory = session_directory_;
    client->async_send_request(request,
      [this, batch_id, action](
        rclcpp::Client<ppbng_interfaces::srv::PrepareDevice>::SharedFutureWithRequest future) {
      const auto response = future.get().second;
      receive_outcome(batch_id, {action.device, action.operation, response->accepted,
        response->message});
    });
  }

  void dispatch_trigger(const std::uint64_t batch_id, const DeviceAction action)
  {
    const auto key = endpoint(action);
    auto & client = trigger_clients_[key];
    if (!client) {client = create_client<std_srvs::srv::Trigger>(
        key, rmw_qos_profile_services_default, callback_group_);}
    client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this, batch_id, action](
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFutureWithRequest future) {
        const auto response = future.get().second;
        receive_outcome(batch_id, {action.device, action.operation, response->success,
          response->message});
      });
  }

  void receive_outcome(const std::uint64_t batch_id, ActionOutcome outcome)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (active_batch_.id != batch_id || active_batch_.expected == 0U) {return;}
    active_batch_.outcomes.push_back(std::move(outcome));
    if (active_batch_.outcomes.size() != active_batch_.expected) {return;}
    const auto outcomes = std::move(active_batch_.outcomes);
    active_batch_ = {};
    process_result(workflow_.complete_batch(batch_id, outcomes));
  }

  void check_timeout()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (active_batch_.expected == 0U || std::chrono::steady_clock::now() < active_batch_.deadline) {
      return;
    }
    const auto batch_id = active_batch_.id;
    active_batch_ = {};
    process_result(workflow_.timeout_pending_batch(batch_id,
      "service response timeout; device may have acted"));
  }

  bool read_available_space(std::uint64_t & available, std::string & detail) const
  {
    std::error_code error;
    const auto information = std::filesystem::space(output_root_, error);
    if (error) {
      detail = error.message();
      return false;
    }
    available = information.available;
    return true;
  }

  bool throughput_qualification_valid(std::string & detail) const
  {
    if (!throughput_qualified_) {
      detail = "no approved qualification is recorded";
      return false;
    }
    StorageCapacityPolicy one_second = storage_policy_;
    one_second.planned_duration_seconds = 1U;
    one_second.minimum_reserve_bytes = 0U;
    const auto required = evaluate_storage_capacity(one_second,
      (std::numeric_limits<std::uint64_t>::max)(), 0U);
    if (!required.valid || qualified_durable_bytes_per_second_ <
      required.required_available_bytes)
    {
      detail = "qualified durable rate is below estimated rate plus headroom";
      return false;
    }
    std::error_code error;
    const auto configured = std::filesystem::weakly_canonical(output_root_, error);
    if (error || throughput_qualification_output_root_.empty()) {
      detail = "qualification output root is missing or invalid";
      return false;
    }
    const auto qualified = std::filesystem::weakly_canonical(
      throughput_qualification_output_root_, error);
    if (error || configured != qualified) {
      detail = "qualification was not performed on the configured output volume/root";
      return false;
    }
    if (throughput_qualification_detail_.empty()) {
      detail = "qualification detail is empty";
      return false;
    }
    return true;
  }

  void check_disk_capacity()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const auto state = workflow_.state();
    if (disk_fault_reported_ || state == ProductionWorkflowState::inert ||
      state == ProductionWorkflowState::fault || state == ProductionWorkflowState::stopping)
    {
      return;
    }
    std::uint64_t available{};
    std::string error;
    if (!read_available_space(available, error)) {
      disk_fault_reported_ = true;
      upsert_warning("manager:disk_capacity", "disk capacity query failed: " + error);
      active_batch_ = {};
      process_result(workflow_.report_global_fault("disk capacity query failed: " + error));
      return;
    }
    const auto elapsed = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::steady_clock::now() - task_started_at_).count());
    const auto capacity = evaluate_storage_capacity(storage_policy_, available, elapsed);
    if (capacity.valid && capacity.sufficient) return;
    disk_fault_reported_ = true;
    std::ostringstream detail;
    detail << capacity.detail << "; available=" << available <<
      "; required=" << capacity.required_available_bytes;
    upsert_warning("manager:disk_capacity", detail.str());
    active_batch_ = {};
    process_result(workflow_.report_global_fault(detail.str()));
  }

  void observe_device_status(const ppbng_interfaces::msg::DeviceStatus & message)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (message.session_id.empty()) return;
    const auto session = sessions_.find(message.session_id);
    if (session == sessions_.end() ||
      (message.session_id != workflow_.session_id() && message.session_id != last_session_id_))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "ignored device status for an unknown/non-current session");
      return;
    }
    const auto role = status_role_by_device_id_.find(message.device_id);
    if (role == status_role_by_device_id_.end()) {
      upsert_warning("manager:unknown_device_status:" + message.device_id,
        "unmapped DeviceStatus device_id: " + message.device_id);
      return;
    }
    auto & devices = session->second.manifest.devices;
    const auto device = std::find_if(devices.begin(), devices.end(),
      [&role](const auto & item) {return item.role == role->second;});
    if (device == devices.end()) return;
    ppbng_storage::DeviceRuntimeUpdate update;
    update.status_host_monotonic_ns = message.status_host_monotonic_ns;
    update.lifecycle_state = message.lifecycle_state;
    update.health = message.health;
    update.segment_id = message.segment_id;
    update.last_sample_valid = message.last_sample_valid;
    update.last_sample_sequence = message.last_sample_sequence;
    update.samples_received = message.samples_received;
    update.samples_incomplete = message.samples_incomplete;
    update.samples_lost = message.samples_lost;
    update.samples_dropped = message.samples_dropped;
    update.reconnect_attempts = message.reconnect_attempts;
    update.active_config_hash = message.active_config_hash;
    update.detail = message.detail;
    if (message.actual_setting_keys.size() != message.actual_setting_values.size()) {
      upsert_warning("manager:invalid_actual_settings:" + role->second,
        role->second + " published mismatched actual-setting key/value arrays");
    } else {
      for (std::size_t index = 0U; index < message.actual_setting_keys.size(); ++index) {
        update.actual_settings.emplace_back(
          message.actual_setting_keys[index], message.actual_setting_values[index]);
      }
    }
    const auto updated = ppbng_storage::apply_device_runtime_update(*device, update);
    if (!updated.actual_settings_valid) {
      upsert_warning("manager:invalid_actual_settings:" + role->second,
        role->second + " published invalid actual settings: " + updated.detail);
    }
    const auto descriptor = std::find_if(device_descriptors_.begin(), device_descriptors_.end(),
      [&role](const auto & item) {return item.role == role->second;});
    const bool camera_stop_evidence = (role->second == "rgb" || role->second == "thermal") &&
      message.health != ppbng_interfaces::msg::DeviceStatus::HEALTH_UNKNOWN &&
      message.detail.find("acquisition stopped") != std::string::npos &&
      !message.actual_setting_keys.empty();
    if (descriptor != device_descriptors_.end() && !descriptor->expected_identity.empty() &&
      !message.actual_setting_keys.empty() &&
      (message.lifecycle_state >= ppbng_interfaces::msg::DeviceStatus::LIFECYCLE_OPEN ||
      camera_stop_evidence))
    {
      device->identity = descriptor->expected_identity;
      upsert_setting(device->actual_settings, "identity_evidence",
        "backend accepted exact configured identity before OPEN status");
    }
    if (message.health == ppbng_interfaces::msg::DeviceStatus::HEALTH_DEGRADED ||
      message.health == ppbng_interfaces::msg::DeviceStatus::HEALTH_FAULT)
    {
      upsert_warning("device:" + message.device_id, message.device_id + ": " + message.detail);
      publish_status("degraded/faulted device continues unless a global-stop fault is published: " +
        message.device_id);
    }
    if (message.detail == "dark_complete" &&
      (message.device_id == "fx10e" || message.device_id == "swir"))
    {
      process_result(workflow_.observe_dark_complete(message.device_id));
    }
    if (session->second.manifest.state == ppbng_storage::ManifestState::finalized ||
      session->second.manifest.state == ppbng_storage::ManifestState::faulted)
    {
      (void)write_manifest(message.session_id, session->second.manifest.state);
    }
  }

  void observe_fault(const ppbng_interfaces::msg::FaultEvent & message)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (message.session_id.empty()) return;
    if (message.session_id != workflow_.session_id()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "ignored fault event for a non-active session");
      return;
    }
    upsert_warning("fault:" + message.source_id + ":" + message.fault_code,
      message.source_id + ":" + message.fault_code + ": " + message.detail);
    if (message.causes_global_stop) {
      // A second fault while an ordered cleanup batch is outstanding is
      // evidence to retain, not a reason to discard the only timeout/reply
      // tracking for that batch.  Clearing active_batch_ here would leave the
      // workflow permanently stuck in STOPPING.
      if (workflow_.state() == ProductionWorkflowState::stopping) {
        publish_status("additional global fault recorded while ordered cleanup continues");
        return;
      }
      active_batch_ = {};
      process_result(workflow_.report_global_fault(
        message.source_id + ":" + message.fault_code + ": " + message.detail));
    } else {
      publish_status("non-global device fault recorded; acquisition continues");
    }
  }

  void observe_trigger()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (workflow_.state() == ProductionWorkflowState::waiting_for_sample_pps) {
      process_result(workflow_.observe_sample_pps());
    }
  }

  void publish_status(const std::string & detail)
  {
    ppbng_interfaces::msg::AcquisitionStatus status;
    status.header.stamp = now();
    status.state = status_state(workflow_.state());
    status.degraded = !warnings_.empty();
    status.session_id = workflow_.session_id();
    status.session_directory = session_directory_;
    status.warnings = warnings_;
    status.detail = detail;
    status_publisher_->publish(status);
    if (workflow_.state() == ProductionWorkflowState::fault) {
      RCLCPP_ERROR(get_logger(), "OPERATOR STATUS: %s", detail.c_str());
    } else if (!warnings_.empty()) {
      RCLCPP_WARN(get_logger(), "OPERATOR STATUS (DEGRADED): %s", detail.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "OPERATOR STATUS: %s", detail.c_str());
    }
  }

  void upsert_warning(const std::string & key, std::string value)
  {
    const auto found = warning_indices_.find(key);
    if (found != warning_indices_.end()) {
      warnings_[found->second] = std::move(value);
      return;
    }
    warning_indices_.emplace(key, warnings_.size());
    warnings_.push_back(std::move(value));
  }

  std::recursive_mutex mutex_;
  ProductionWorkflow workflow_;
  std::filesystem::path output_root_;
  std::filesystem::path machine_config_path_;
  std::string machine_id_;
  std::string machine_config_snapshot_;
  std::chrono::milliseconds batch_timeout_{5000};
  StorageCapacityPolicy storage_policy_;
  std::chrono::steady_clock::time_point task_started_at_{};
  bool disk_fault_reported_{false};
  bool throughput_qualified_{false};
  std::uint64_t qualified_durable_bytes_per_second_{};
  std::string throughput_qualification_detail_;
  std::filesystem::path throughput_qualification_output_root_;
  std::string session_directory_;
  std::string last_session_id_;
  std::vector<std::string> warnings_;
  std::unordered_map<std::string, std::size_t> warning_indices_;
  std::unordered_map<std::string, CommandRecord> history_;
  std::unordered_map<std::string, SessionRecord> sessions_;
  std::vector<DeviceDescriptor> device_descriptors_;
  std::unordered_map<std::string, std::string> status_role_by_device_id_;
  ActiveBatch active_batch_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::unordered_map<std::string,
    rclcpp::Client<ppbng_interfaces::srv::PrepareDevice>::SharedPtr> prepare_clients_;
  std::unordered_map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> trigger_clients_;
  rclcpp::Publisher<ppbng_interfaces::msg::AcquisitionStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<ppbng_interfaces::msg::DeviceStatus>::SharedPtr device_status_subscription_;
  rclcpp::Subscription<ppbng_interfaces::msg::FaultEvent>::SharedPtr fault_subscription_;
  rclcpp::Subscription<ppbng_interfaces::msg::TriggerEvent>::SharedPtr trigger_subscription_;
  rclcpp::Service<ppbng_interfaces::srv::StartAcquisition>::SharedPtr start_service_;
  rclcpp::Service<ppbng_interfaces::srv::ConfirmDarkReady>::SharedPtr dark_service_;
  rclcpp::Service<ppbng_interfaces::srv::ConfirmSampleReady>::SharedPtr sample_service_;
  rclcpp::Service<ppbng_interfaces::srv::StopAcquisition>::SharedPtr stop_service_;
  rclcpp::Service<ppbng_interfaces::srv::AbortAcquisition>::SharedPtr abort_service_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  rclcpp::TimerBase::SharedPtr disk_timer_;
};
}  // namespace ppbng_runtime

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<ppbng_runtime::ProductionAcquisitionManagerNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4U);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("production_acquisition_manager"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
