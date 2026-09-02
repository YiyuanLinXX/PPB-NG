#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <ppbng_interfaces/msg/acquisition_status.hpp>
#include <ppbng_interfaces/srv/abort_acquisition.hpp>
#include <ppbng_interfaces/srv/confirm_dark_ready.hpp>
#include <ppbng_interfaces/srv/confirm_sample_ready.hpp>
#include <ppbng_interfaces/srv/start_acquisition.hpp>
#include <ppbng_interfaces/srv/stop_acquisition.hpp>
#include <ppbng_orchestrator/acquisition_state_machine.hpp>
#include <ppbng_storage/dataset_session.hpp>
#include <ppbng_storage/session_manifest.hpp>

namespace ppbng_runtime
{
using namespace std::chrono_literals;

class AcquisitionManagerNode final : public rclcpp::Node
{
public:
  AcquisitionManagerNode()
  : Node("acquisition_manager"),
    machine_([this]() {return create_session_for_pending_start();})
  {
    simulation_mode_ = declare_parameter<bool>("simulation_mode", true);
    output_root_ = std::filesystem::u8path(declare_parameter<std::string>("output_root", ""));
    if (!simulation_mode_) {
      throw std::runtime_error(
              "hardware runtime is intentionally unavailable until approved device backends exist");
    }

    status_publisher_ = create_publisher<ppbng_interfaces::msg::AcquisitionStatus>(
      "acquisition/status", rclcpp::QoS(1).reliable().transient_local());

    start_service_ = create_service<ppbng_interfaces::srv::StartAcquisition>(
      "acquisition/start",
      [this](
        const std::shared_ptr<ppbng_interfaces::srv::StartAcquisition::Request> request,
        std::shared_ptr<ppbng_interfaces::srv::StartAcquisition::Response> response) {
        handle_start(*request, *response);
      });
    dark_service_ = create_service<ppbng_interfaces::srv::ConfirmDarkReady>(
      "acquisition/confirm_dark_ready",
      [this](
        const std::shared_ptr<ppbng_interfaces::srv::ConfirmDarkReady::Request> request,
        std::shared_ptr<ppbng_interfaces::srv::ConfirmDarkReady::Response> response) {
        handle_dark(*request, *response);
      });
    sample_service_ = create_service<ppbng_interfaces::srv::ConfirmSampleReady>(
      "acquisition/confirm_sample_ready",
      [this](
        const std::shared_ptr<ppbng_interfaces::srv::ConfirmSampleReady::Request> request,
        std::shared_ptr<ppbng_interfaces::srv::ConfirmSampleReady::Response> response) {
        handle_sample(*request, *response);
      });
    stop_service_ = create_service<ppbng_interfaces::srv::StopAcquisition>(
      "acquisition/stop",
      [this](
        const std::shared_ptr<ppbng_interfaces::srv::StopAcquisition::Request> request,
        std::shared_ptr<ppbng_interfaces::srv::StopAcquisition::Response> response) {
        handle_stop(*request, *response);
      });
    abort_service_ = create_service<ppbng_interfaces::srv::AbortAcquisition>(
      "acquisition/abort",
      [this](
        const std::shared_ptr<ppbng_interfaces::srv::AbortAcquisition::Request> request,
        std::shared_ptr<ppbng_interfaces::srv::AbortAcquisition::Response> response) {
        handle_abort(*request, *response);
      });

    simulation_pps_timer_ = create_wall_timer(100ms, [this]() {
      if (machine_.state() == ppbng_orchestrator::AcquisitionState::waiting_for_pps) {
        const auto started = machine_.reach_start_pps();
        if (started.accepted &&
          !update_manifest_state(machine_.session_id(), ppbng_storage::ManifestState::recording))
        {
          machine_.report_global_fault(
            ppbng_orchestrator::GlobalFault::disk_write_failure,
            "recording manifest update failed");
          machine_.begin_fault_stop();
          machine_.complete_stop(false, "recording manifest update failed");
          publish_status("storage fault: recording manifest update failed");
          return;
        }
        publish_status("simulation PPS boundary reached");
      }
    });
    publish_status("simulation runtime ready; no hardware backend loaded");
  }

private:
  std::string create_session_for_pending_start()
  {
    pending_session_error_.clear();
    if (output_root_.empty()) {
      pending_session_error_ = "output_root parameter is empty";
      return {};
    }
    const auto identity = ppbng_storage::make_session_identity_now();
    const auto created = ppbng_storage::create_dataset_session(
      output_root_, pending_dataset_name_, identity);
    if (!created.ok()) {
      pending_session_error_ = created.detail.empty() ? "dataset session creation failed" : created.detail;
      return {};
    }
    const auto session_id = identity.unique_id;
    SessionRecord record;
    record.paths = created.paths;
    record.manifest.session_id = session_id;
    record.manifest.user_dataset_name = pending_dataset_name_;
    record.manifest.created_utc = identity.utc_timestamp;
    record.manifest.state = ppbng_storage::ManifestState::created;
    record.manifest.simulation = true;
    record.manifest.hardware_enabled = false;
    for (const auto & role : {"fx10e", "swir", "rgb", "thermal", "gnss", "rsm400", "timing"}) {
      ppbng_storage::DeviceManifest device;
      device.role = role;
      device.backend = "simulation";
      if (device.role == "thermal") {
        device.model = "A6701";
        device.requested_settings = {
          {"transport_geometry", "640x513"}, {"image_geometry", "640x512"},
          {"pixel_format", "Mono16"}, {"payload_bytes", "656640"}};
      } else if (device.role == "rgb") {
        device.requested_settings = {{"payload", "raw Bayer"}, {"trigger_rate_hz", "2"}};
      }
      record.manifest.devices.push_back(std::move(device));
    }
    const auto serialized = ppbng_storage::serialize_manifest_json(record.manifest);
    if (!serialized.valid) {
      pending_session_error_ = "manifest validation failed: " + serialized.detail;
      return {};
    }
    const auto written = ppbng_storage::atomic_write_text(
      record.paths.session_directory, "manifest.json", serialized.json, "manifest_0");
    if (!written.ok()) {
      pending_session_error_ = "initial manifest write failed: " + written.detail;
      return {};
    }
    session_directories_[session_id] = created.paths.session_directory.u8string();
    sessions_.emplace(session_id, std::move(record));
    return session_id;
  }

  bool update_manifest_state(
    const std::string & session_id, const ppbng_storage::ManifestState state)
  {
    const auto found = sessions_.find(session_id);
    if (found == sessions_.end()) {
      return false;
    }
    auto & record = found->second;
    record.manifest.state = state;
    if (state == ppbng_storage::ManifestState::finalized ||
      state == ppbng_storage::ManifestState::faulted)
    {
      record.manifest.finalized_utc = ppbng_storage::make_session_identity_now().utc_timestamp;
    }
    const auto serialized = ppbng_storage::serialize_manifest_json(record.manifest);
    if (!serialized.valid) {
      RCLCPP_ERROR(get_logger(), "manifest validation failed: %s", serialized.detail.c_str());
      return false;
    }
    ++record.manifest_revision;
    const auto operation = "manifest_" + std::to_string(record.manifest_revision);
    const auto written = ppbng_storage::atomic_write_text(
      record.paths.session_directory, "manifest.json", serialized.json, operation);
    if (!written.ok()) {
      RCLCPP_ERROR(get_logger(), "manifest update failed: %s", written.detail.c_str());
      return false;
    }
    return true;
  }

  template<typename ResponseT>
  void fill_workflow_response(
    const ppbng_orchestrator::CommandResult & result, ResponseT & response)
  {
    response.accepted = result.accepted;
    response.duplicate_request = result.duplicate_request;
    response.session_id = result.session_id;
    response.resulting_state = static_cast<std::uint8_t>(machine_.state());
    response.message = result.message;
  }

  void handle_start(
    const ppbng_interfaces::srv::StartAcquisition::Request & request,
    ppbng_interfaces::srv::StartAcquisition::Response & response)
  {
    pending_dataset_name_ = request.dataset_name;
    const auto result = machine_.start(
      request.request_id, request.dataset_name, request.force_degraded);
    response.accepted = result.accepted;
    response.duplicate_request = result.duplicate_request;
    response.session_id = result.session_id;
    const auto directory = session_directories_.find(result.session_id);
    response.session_directory = directory == session_directories_.end() ? "" : directory->second;
    response.message = !result.accepted && !pending_session_error_.empty() ?
      pending_session_error_ : result.message;
    if (result.accepted && !result.duplicate_request) {
      machine_.complete_preflight(true, false, "simulation preflight passed");
    }
    publish_status(response.message);
  }

  void handle_dark(
    const ppbng_interfaces::srv::ConfirmDarkReady::Request & request,
    ppbng_interfaces::srv::ConfirmDarkReady::Response & response)
  {
    const auto result = machine_.confirm_dark_ready(request.request_id);
    if (result.accepted && !result.duplicate_request) {
      machine_.complete_dark_capture(true, "simulation dark capture completed");
    }
    fill_workflow_response(result, response);
    publish_status(result.message);
  }

  void handle_sample(
    const ppbng_interfaces::srv::ConfirmSampleReady::Request & request,
    ppbng_interfaces::srv::ConfirmSampleReady::Response & response)
  {
    const auto result = machine_.confirm_sample_ready(request.request_id);
    fill_workflow_response(result, response);
    publish_status(result.message);
  }

  void handle_stop(
    const ppbng_interfaces::srv::StopAcquisition::Request & request,
    ppbng_interfaces::srv::StopAcquisition::Response & response)
  {
    const auto result = machine_.stop(request.request_id, request.reason);
    if (result.accepted && !result.duplicate_request) {
      const bool manifest_ok =
        update_manifest_state(result.session_id, ppbng_storage::ManifestState::finalized);
      machine_.complete_stop(
        manifest_ok, manifest_ok ? "simulation finalization completed" :
        "final manifest write failed");
      if (!manifest_ok) {
        update_manifest_state(result.session_id, ppbng_storage::ManifestState::faulted);
      }
    }
    response.accepted = result.accepted;
    response.duplicate_request = result.duplicate_request;
    response.session_id = result.session_id;
    response.message = machine_.state() == ppbng_orchestrator::AcquisitionState::fault ?
      result.message + "; final manifest write failed" : result.message;
    publish_status(response.message);
  }

  void handle_abort(
    const ppbng_interfaces::srv::AbortAcquisition::Request & request,
    ppbng_interfaces::srv::AbortAcquisition::Response & response)
  {
    const auto result = machine_.abort(request.request_id, request.reason);
    if (result.accepted && !result.duplicate_request) {
      const bool manifest_ok =
        update_manifest_state(result.session_id, ppbng_storage::ManifestState::faulted);
      machine_.complete_stop(
        manifest_ok, manifest_ok ? "simulation abort finalized" :
        "fault manifest write failed");
    }
    fill_workflow_response(result, response);
    if (machine_.state() == ppbng_orchestrator::AcquisitionState::fault) {
      response.message = result.message + "; fault manifest write failed";
    }
    publish_status(response.message);
  }

  void publish_status(const std::string & detail)
  {
    ppbng_interfaces::msg::AcquisitionStatus status;
    status.header.stamp = now();
    status.state = static_cast<std::uint8_t>(machine_.state());
    status.degraded = machine_.degraded();
    status.session_id = machine_.session_id();
    const auto directory = session_directories_.find(status.session_id);
    status.session_directory = directory == session_directories_.end() ? "" : directory->second;
    status.active_devices = {"sim_fx10e", "sim_swir", "sim_rgb", "sim_thermal", "sim_gnss",
      "sim_rsm400", "sim_timing"};
    if (!machine_.health_detail().empty() &&
      machine_.health() != ppbng_orchestrator::Health::ok)
    {
      status.warnings.push_back(machine_.health_detail());
    }
    status.detail = detail;
    status_publisher_->publish(status);
  }

  bool simulation_mode_{true};
  std::filesystem::path output_root_;
  std::string pending_dataset_name_;
  std::string pending_session_error_;
  struct SessionRecord
  {
    ppbng_storage::DatasetSessionPaths paths;
    ppbng_storage::SessionManifest manifest;
    std::uint64_t manifest_revision{0};
  };
  std::unordered_map<std::string, std::string> session_directories_;
  std::unordered_map<std::string, SessionRecord> sessions_;
  ppbng_orchestrator::AcquisitionStateMachine machine_;

  rclcpp::Publisher<ppbng_interfaces::msg::AcquisitionStatus>::SharedPtr status_publisher_;
  rclcpp::Service<ppbng_interfaces::srv::StartAcquisition>::SharedPtr start_service_;
  rclcpp::Service<ppbng_interfaces::srv::ConfirmDarkReady>::SharedPtr dark_service_;
  rclcpp::Service<ppbng_interfaces::srv::ConfirmSampleReady>::SharedPtr sample_service_;
  rclcpp::Service<ppbng_interfaces::srv::StopAcquisition>::SharedPtr stop_service_;
  rclcpp::Service<ppbng_interfaces::srv::AbortAcquisition>::SharedPtr abort_service_;
  rclcpp::TimerBase::SharedPtr simulation_pps_timer_;
};
}  // namespace ppbng_runtime

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ppbng_runtime::AcquisitionManagerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("ppbng_runtime"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
