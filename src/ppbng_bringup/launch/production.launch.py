from pathlib import Path
import ctypes
import json
import math
import os

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_bool(value, name):
    normalized = value.strip().lower()
    if normalized in ("true", "1", "yes", "on"):
        return True
    if normalized in ("false", "0", "no", "off"):
        return False
    raise RuntimeError(f"{name} must be true or false")


def _placeholder_paths(value, path=""):
    if isinstance(value, dict):
        for key, child in value.items():
            yield from _placeholder_paths(child, f"{path}.{key}" if path else str(key))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            yield from _placeholder_paths(child, f"{path}[{index}]")
    elif isinstance(value, str) and "REQUIRED" in value:
        yield path


def _positive_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeError(f"production launch refused: {name} must be numeric")
    converted = float(value)
    if not math.isfinite(converted) or converted <= 0.0:
        raise RuntimeError(f"production launch refused: {name} must be finite and positive")
    return converted


def _positive_integer(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise RuntimeError(f"production launch refused: {name} must be a positive integer")
    return value


def _canonical_path_key(path):
    return os.path.normcase(str(Path(path).resolve(strict=True)))


def _read_utf8_exact(path):
    # read_text() performs universal-newline conversion. Preserve CRLF bytes so
    # the Windows manager can compare the launch-bound snapshot byte-for-byte.
    return Path(path).read_bytes().decode("utf-8")


def _windows_volume_identity(path):
    if os.name != "nt":
        raise RuntimeError("production launch refused: Windows volume identity is unavailable")
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    volume_root = ctypes.create_unicode_buffer(32768)
    if not kernel32.GetVolumePathNameW(str(path), volume_root, len(volume_root)):
        raise OSError(ctypes.get_last_error(), "GetVolumePathNameW failed")
    serial = ctypes.c_ulong()
    filesystem = ctypes.create_unicode_buffer(256)
    if not kernel32.GetVolumeInformationW(
        volume_root.value, None, 0, ctypes.byref(serial), None, None,
        filesystem, len(filesystem)
    ):
        raise OSError(ctypes.get_last_error(), "GetVolumeInformationW failed")
    return f"{serial.value:08X}", filesystem.value, volume_root.value


def _load_throughput_evidence(session, output_root, volume_identity=None):
    evidence_text = session.get("throughput_evidence_path", "")
    if not isinstance(evidence_text, str) or not evidence_text.strip():
        raise RuntimeError(
            "production launch refused: session.throughput_evidence_path is required"
        )
    try:
        root_key = _canonical_path_key(output_root)
        evidence_path = Path(evidence_text).resolve(strict=True)
        if not evidence_path.is_file() or evidence_path.stat().st_size == 0:
            raise RuntimeError("evidence is missing or empty")
        if _canonical_path_key(evidence_path.parent) != root_key:
            raise RuntimeError("evidence must be directly beneath the configured output root")
        evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError, RuntimeError) as error:
        raise RuntimeError(
            f"production launch refused: invalid throughput evidence: {error}"
        ) from error
    if not isinstance(evidence, dict) or evidence.get("schema_version") != 1 or evidence.get(
        "tool"
    ) != "ppbng_storage_durable_write_qualification":
        raise RuntimeError("production launch refused: unrecognized throughput evidence schema")
    try:
        if _canonical_path_key(evidence.get("normalized_output_root", "")) != root_key:
            raise RuntimeError("evidence output root does not match session.output_root")
    except (OSError, RuntimeError) as error:
        raise RuntimeError(f"production launch refused: {error}") from error
    parameters = evidence.get("parameters")
    if not isinstance(parameters, dict) or parameters.get("write_through") is not True or \
            parameters.get("flush_file_buffers") is not True:
        raise RuntimeError("production launch refused: durable flush evidence is absent")
    mib = 1024 * 1024
    requested_duration = parameters.get("requested_duration_seconds")
    block_bytes = parameters.get("block_bytes")
    maximum_test_bytes = parameters.get("maximum_test_bytes")
    reserve_bytes = parameters.get("reserve_bytes")
    if (isinstance(requested_duration, bool) or not isinstance(requested_duration, int) or
            not 1 <= requested_duration <= 60):
        raise RuntimeError("production launch refused: evidence requested duration is invalid")
    if (isinstance(block_bytes, bool) or not isinstance(block_bytes, int) or
            block_bytes % mib != 0 or not mib <= block_bytes <= 64 * mib):
        raise RuntimeError("production launch refused: evidence block size is invalid")
    if (isinstance(maximum_test_bytes, bool) or not isinstance(maximum_test_bytes, int) or
            maximum_test_bytes % mib != 0 or
            not block_bytes <= maximum_test_bytes <= 16_384 * mib):
        raise RuntimeError("production launch refused: evidence maximum test size is invalid")
    if reserve_bytes != 100 * 1024 ** 3:
        raise RuntimeError("production launch refused: evidence reserve is invalid")
    bytes_written = evidence.get("bytes_written")
    duration = evidence.get("duration_seconds")
    rate = evidence.get("bytes_per_second")
    if isinstance(bytes_written, bool) or not isinstance(bytes_written, int) or bytes_written <= 0:
        raise RuntimeError("production launch refused: evidence bytes_written is invalid")
    if bytes_written > maximum_test_bytes or bytes_written % block_bytes != 0:
        raise RuntimeError("production launch refused: evidence byte count violates probe bounds")
    duration = _positive_number(duration, "evidence.duration_seconds")
    rate = _positive_number(rate, "evidence.bytes_per_second")
    measured = bytes_written / duration
    if not math.isclose(rate, measured, rel_tol=1e-6, abs_tol=1.0):
        raise RuntimeError("production launch refused: evidence rate is internally inconsistent")
    resolver = volume_identity or _windows_volume_identity
    try:
        current_serial, current_filesystem, current_root = resolver(Path(output_root))
    except (OSError, RuntimeError) as error:
        raise RuntimeError(
            f"production launch refused: cannot verify output volume identity: {error}"
        ) from error
    if str(evidence.get("volume_serial_hex", "")).upper() != str(current_serial).upper():
        raise RuntimeError("production launch refused: output volume serial has changed")
    if str(evidence.get("filesystem", "")).casefold() != str(current_filesystem).casefold():
        raise RuntimeError("production launch refused: output filesystem has changed")
    if os.path.normcase(str(evidence.get("volume_root", ""))) != os.path.normcase(
        str(current_root)
    ):
        raise RuntimeError("production launch refused: output volume root has changed")
    return {
        "throughput_qualified": True,
        "qualified_durable_bytes_per_second": int(math.floor(rate)),
        "throughput_qualification_detail": (
            f"verified evidence {evidence_path.name}; volume {current_serial}"
        ),
        "throughput_qualification_output_root": str(Path(output_root).resolve(strict=True)),
    }


def _channel_rate(timing, channel):
    values = timing.get("channels", {}).get(channel)
    if not isinstance(values, dict) or values.get("enabled") is not True:
        raise RuntimeError(f"production launch refused: timing channel {channel} must be enabled")
    numerator = _positive_integer(values.get("rate_numerator_hz"),
                                  f"timing.channels.{channel}.rate_numerator_hz")
    denominator = _positive_integer(values.get("rate_denominator"),
                                    f"timing.channels.{channel}.rate_denominator")
    return numerator / denominator


def _estimated_payload_rate(document):
    timing = document["timing"]
    hsi = document["hsi"]
    # HSI cameras run continuously from their internal line clocks. Their
    # rates therefore come from the camera configuration, not trigger outputs.
    fx_rate = _positive_number(hsi["fx10e"]["line_rate_hz"],
                               "hsi.fx10e.line_rate_hz")
    swir_rate = _positive_number(hsi["swir"]["line_rate_hz"],
                                 "hsi.swir.line_rate_hz")
    rgb_rate = _channel_rate(timing, "rgb")
    thermal_rate = _channel_rate(timing, "thermal")
    if not math.isclose(rgb_rate, thermal_rate, rel_tol=0.0, abs_tol=0.0):
        raise RuntimeError(
            "production launch refused: RGB and thermal logical channels must share one rate"
        )
    fx_bytes = (_positive_integer(hsi["fx10e"]["spatial_samples"],
                                 "hsi.fx10e.spatial_samples") *
                _positive_integer(hsi["fx10e"]["spectral_bands"],
                                 "hsi.fx10e.spectral_bands") * 2.0 * fx_rate)
    swir_bytes = (_positive_integer(hsi["swir"]["spatial_samples"],
                                   "hsi.swir.spatial_samples") *
                  _positive_integer(hsi["swir"]["spectral_bands"],
                                   "hsi.swir.spectral_bands") * 2.0 * swir_rate)
    rgb_payload = _positive_integer(
        document["rgb"].get("acquisition", {}).get("payload_bytes"),
        "rgb.acquisition.payload_bytes")
    # The installed A6701 transport contract is exactly 640x513 Mono16.
    thermal_payload = 640.0 * 513.0 * 2.0
    total = fx_bytes + swir_bytes + rgb_payload * rgb_rate + thermal_payload * thermal_rate
    if not math.isfinite(total) or total > 9_223_372_036_854_775_807:
        raise RuntimeError("production launch refused: estimated payload rate is not representable")
    return math.ceil(total)


def _validate_hsi_queue_bounds(document):
    hsi = document["hsi"]
    for camera_name in ("fx10e", "swir"):
        camera = hsi[camera_name]
        callback_capacity = _positive_integer(
            camera.get("callback_queue_capacity", 128),
            f"hsi.{camera_name}.callback_queue_capacity",
        )
        pending_capacity = _positive_integer(
            camera.get("pending_trigger_capacity", 128),
            f"hsi.{camera_name}.pending_trigger_capacity",
        )
        _positive_integer(
            camera.get("trigger_match_timeout_ms", 500),
            f"hsi.{camera_name}.trigger_match_timeout_ms",
        )
        if callback_capacity > 512:
            raise RuntimeError(
                "production launch refused: "
                f"hsi.{camera_name}.callback_queue_capacity exceeds 512"
            )
        line_bytes = (
            _positive_integer(camera["spatial_samples"],
                              f"hsi.{camera_name}.spatial_samples") *
            _positive_integer(camera["spectral_bands"],
                              f"hsi.{camera_name}.spectral_bands") * 2
        )
        if line_bytes > 64 * 1024 * 1024 or callback_capacity * line_bytes > 1024 ** 3:
            raise RuntimeError(
                "production launch refused: "
                f"hsi.{camera_name} callback storage exceeds safety bounds"
            )
        if pending_capacity < callback_capacity:
            raise RuntimeError(
                "production launch refused: "
                f"hsi.{camera_name}.pending_trigger_capacity must be at least "
                "callback_queue_capacity"
            )


def _validate_hsi_continuous_mode(document):
    """Validate the two-camera deployment contract before any process opens hardware."""
    hsi = document.get("hsi")
    timing = document.get("timing", {})
    if not isinstance(hsi, dict) or hsi.get("enabled") is not True:
        raise RuntimeError("production launch refused: hsi.enabled must be true")
    for camera_name in ("fx10e", "swir"):
        camera = hsi.get(camera_name)
        if not isinstance(camera, dict) or camera.get("enabled") is not True:
            raise RuntimeError(
                f"production launch refused: hsi.{camera_name}.enabled must be true"
            )
        if camera.get("trigger_mode") != "Internal":
            raise RuntimeError(
                "production launch refused: both line-scan HSI cameras must use "
                "Internal continuous timing"
            )
        channel = timing.get("channels", {}).get(camera_name)
        if not isinstance(channel, dict) or channel.get("enabled") is not False:
            raise RuntimeError(
                "production launch refused: external timing outputs for internally "
                f"timed HSI camera {camera_name} must be disabled"
            )
    fx_channel = str(hsi["fx10e"].get("grabber_channel", "")).strip()
    if not fx_channel or fx_channel.casefold() == "ui":
        raise RuntimeError(
            "production launch refused: FX10e requires a fixed headless grabber_channel"
        )
    initialization_timeout_ms = _positive_integer(
        hsi.get("initialization_timeout_ms", 5000), "hsi.initialization_timeout_ms"
    )
    batch_timeout_ms = _positive_integer(
        document.get("runtime", {}).get("batch_timeout_ms", 5000),
        "runtime.batch_timeout_ms",
    )
    if batch_timeout_ms <= initialization_timeout_ms:
        raise RuntimeError(
            "production launch refused: runtime.batch_timeout_ms must exceed the HSI "
            "SDK initialization timeout"
        )
    _positive_integer(
        document.get("session", {}).get("hsi_flush_every_lines", 120),
        "session.hsi_flush_every_lines",
    )


def _validate_safety_policy(document):
    if document.get("schema_version") != 1:
        raise RuntimeError("production launch refused: machine config schema_version must be 1")
    safety = document.get("safety")
    if not isinstance(safety, dict) or safety.get("configured") is not True:
        raise RuntimeError("production launch refused: safety.configured must be true")
    if safety.get("hardware_enabled") is not True:
        raise RuntimeError(
            "production launch refused: safety.hardware_enabled must also be true"
        )
    for key in (
        "allow_device_reset",
        "allow_network_reconfiguration",
        "allow_firmware_update",
    ):
        if safety.get(key) is not False:
            raise RuntimeError(
                f"production launch refused: safety.{key} must be explicitly false"
            )
    if document.get("gnss", {}).get("receive_only") is not True:
        raise RuntimeError("production launch refused: gnss.receive_only must be true")
    if document.get("rsm400", {}).get("observe_only_on_launch") is not True:
        raise RuntimeError(
            "production launch refused: rsm400.observe_only_on_launch must be true"
        )
    pps_required = document.get("timing", {}).get("pps_required")
    if not isinstance(pps_required, bool):
        raise RuntimeError(
            "production launch refused: timing.pps_required must be explicitly true or false"
        )


def _hsi_parameters(hsi, session, camera, output_root, dark_seconds):
    values = hsi[camera]
    parameters = {
        "hardware_enabled": True,
        "allowed_output_root": output_root,
        "camera_kind": camera,
        "device_id": camera,
        "device_index": values.get("device_index", -1),
        "expected_sensor_serial": values["expected_identity"],
        "expected_profile_name": values["expected_profile_name"],
        "calibration_pack_path": values["calibration_pack_path"],
        "license_path": hsi.get("license_path", ""),
        "initialization_timeout_ms": hsi.get("initialization_timeout_ms", 5000),
        "trigger_channel": camera,
        "trigger_mode": values.get("trigger_mode", "Internal"),
        "spatial_samples": values["spatial_samples"],
        "spectral_bands": values["spectral_bands"],
        "line_rate_hz": values["line_rate_hz"],
        "exposure_us": values["exposure_us"],
        "dark_line_count": int(math.ceil(dark_seconds * values["line_rate_hz"])),
        "maximum_segment_bytes": session.get("maximum_segment_bytes", 8589934592),
        "flush_every_lines": session.get("hsi_flush_every_lines", 120),
        "maximum_frame_bytes": values["spatial_samples"] * values["spectral_bands"] * 2,
        "callback_queue_capacity": values.get("callback_queue_capacity", 128),
        "pending_trigger_capacity": values.get("pending_trigger_capacity", 128),
        "trigger_match_timeout_ms": values.get("trigger_match_timeout_ms", 500),
        "internal_first_frame_timeout_ms": values.get(
            "internal_first_frame_timeout_ms", 10000),
        "fail_fast_on_sample_fault": values.get(
            "fail_fast_on_sample_fault", True),
        "association_evidence_confirmed": values.get(
            "association_evidence_confirmed", False),
        **values.get("recovery", {}),
    }
    if camera == "fx10e":
        parameters.update({
            "grabber_channel": values["grabber_channel"],
            "pleora_packet_size": values["pleora_packet_size"],
        })
    else:
        parameters.update({
            "ni_grabber_channel": values["ni_grabber_channel"],
            "ni_camera_file_path": values["ni_camera_file_path"],
            "ni_camera_serial_port": values.get("ni_camera_serial_port", ""),
        })
    return parameters


def _ros_scalar(value):
    if isinstance(value, (dict, list)) or value is None:
        raise RuntimeError("dual HSI process accepts scalar ROS parameters only")
    return json.dumps(value, ensure_ascii=True, separators=(",", ":"))


def _dual_hsi_arguments(fx10e_parameters, swir_parameters, remappings):
    arguments = [
        "--fx10e-namespace", "/fx10e", "--fx10e-name", "camera",
        "--swir-namespace", "/swir", "--swir-name", "camera",
    ]
    for camera, parameters in (("fx10e", fx10e_parameters), ("swir", swir_parameters)):
        for name, value in sorted(parameters.items()):
            arguments.extend([f"--{camera}-param", f"{name}:={_ros_scalar(value)}"])
        for source, destination in remappings:
            arguments.extend([f"--{camera}-remap", f"{source}:={destination}"])
    return arguments


def _validate_and_create_nodes(context):
    enabled = _parse_bool(
        LaunchConfiguration("hardware_enabled").perform(context), "hardware_enabled"
    )
    config_text = LaunchConfiguration("machine_config").perform(context).strip()
    machine_id = LaunchConfiguration("machine_id").perform(context).strip()

    if not enabled:
        raise RuntimeError(
            "production launch refused: pass hardware_enabled:=true explicitly; "
            "the safe default is false"
        )
    if not config_text:
        raise RuntimeError("production launch refused: machine_config is empty")
    if not machine_id or machine_id == "REQUIRED":
        raise RuntimeError("production launch refused: machine_id is empty or a placeholder")

    config_path = Path(config_text)
    if not config_path.is_file() or config_path.stat().st_size == 0:
        raise RuntimeError("production launch refused: machine_config is missing or empty")
    try:
        config_source_text = _read_utf8_exact(config_path)
        document = yaml.safe_load(config_source_text)
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise RuntimeError(f"production launch refused: invalid machine YAML: {error}") from error
    if not isinstance(document, dict):
        raise RuntimeError("production launch refused: machine config root must be a mapping")
    _validate_safety_policy(document)
    configured_machine_id = document.get("machine_id")
    if configured_machine_id != machine_id:
        raise RuntimeError(
            "production launch refused: launch machine_id does not match machine config"
        )

    required_values = {
        "session.output_root": document.get("session", {}).get("output_root"),
        "timing.port": document.get("timing", {}).get("port"),
        "gnss.port": document.get("gnss", {}).get("port"),
        "rsm400.port": document.get("rsm400", {}).get("port"),
        # Spinnaker discovery currently matches the GenICam TLDevice DeviceID
        # exactly. MAC/IP are inventory evidence only until their node-map keys
        # have been verified on the installed A6701.
        "thermal.expected_device_id": document.get("thermal", {}).get(
            "expected_device_id"
        ),
        "rgb.expected_serial": document.get("rgb", {}).get("expected_serial"),
        "hsi.fx10e.expected_identity": document.get("hsi", {}).get("fx10e", {}).get(
            "expected_identity"
        ),
        "hsi.swir.expected_identity": document.get("hsi", {}).get("swir", {}).get(
            "expected_identity"
        ),
    }
    for name, value in required_values.items():
        if not isinstance(value, str) or not value.strip() or "REQUIRED" in value:
            raise RuntimeError(
                f"production launch refused: {name} is empty or still a placeholder"
            )
    rsm400 = document["rsm400"]
    if rsm400.get("require_ready_telemetry") is not True:
        raise RuntimeError(
            "production launch refused: rsm400.require_ready_telemetry must be explicitly true"
        )
    if rsm400.get("stab_motion_status_confirmed") is not True:
        raise RuntimeError(
            "production launch refused: the Stage 5 STAB motion-status value is unconfirmed"
        )
    expected_stab = rsm400.get("expected_stab_motion_status")
    maximum_ready_error = rsm400.get("maximum_ready_error_level")
    for name, value in (("expected_stab_motion_status", expected_stab),
                        ("maximum_ready_error_level", maximum_ready_error)):
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 9:
            raise RuntimeError(
                f"production launch refused: rsm400.{name} must be a confirmed digit 0..9"
            )
    ready_timeout_ms = _positive_integer(
        rsm400.get("ready_timeout_ms"), "rsm400.ready_timeout_ms")
    if ready_timeout_ms > 10000:
        raise RuntimeError(
            "production launch refused: rsm400.ready_timeout_ms exceeds the 10000 ms hard limit"
        )
    batch_timeout_ms = _positive_integer(
        document.get("runtime", {}).get("batch_timeout_ms", 5000),
        "runtime.batch_timeout_ms")
    if ready_timeout_ms >= batch_timeout_ms:
        raise RuntimeError(
            "production launch refused: RSM readiness timeout must be shorter than batch timeout"
        )
    remaining_placeholders = list(_placeholder_paths(document))
    if remaining_placeholders:
        raise RuntimeError(
            "production launch refused: unresolved REQUIRED placeholders: "
            + ", ".join(remaining_placeholders)
        )

    output_root = str(required_values["session.output_root"])
    session = document["session"]
    throughput_evidence = _load_throughput_evidence(session, output_root)
    common_parameters = {
        "hardware_enabled": True,
        "launch_authorized": True,
        "machine_config_path": str(config_path.resolve()),
        # Bind the manager's persisted snapshot to the exact bytes used above to
        # construct every node parameter. The manager also checks that the file
        # has not changed during launch handoff.
        "machine_config_snapshot": config_source_text,
        "machine_id": machine_id,
    }
    hsi = document["hsi"]
    timing = document["timing"]
    dark_seconds = _positive_number(
        session.get("hsi_dark_duration_seconds", 5.0),
        "session.hsi_dark_duration_seconds")
    _validate_hsi_queue_bounds(document)
    _validate_hsi_continuous_mode(document)
    estimated_payload_rate = _estimated_payload_rate(document)
    sdk_bin = str(Path(hsi["specsensor_sdk_root"]) / "bin" / "x64")
    hsi_environment = {"PATH": sdk_bin + os.pathsep + os.environ.get("PATH", "")}
    hsi_remappings = [
        ("trigger", "/acquisition/trigger"),
        ("sample_stamp", "/acquisition/sample_stamp"),
        ("status", "/acquisition/device_status"),
        ("fault_event", "/acquisition/fault_event"),
    ]
    fx10e_parameters = _hsi_parameters(
        hsi, session, "fx10e", output_root, dark_seconds)
    swir_parameters = _hsi_parameters(
        hsi, session, "swir", output_root, dark_seconds)
    nodes = [
        Node(
            package="ppbng_runtime",
            executable="production_acquisition_manager_node",
            name="production_acquisition_manager",
            output="screen",
            parameters=[common_parameters, {
                "output_root": output_root,
                "batch_timeout_ms": document.get("runtime", {}).get("batch_timeout_ms", 5000),
                "estimated_bytes_per_second": estimated_payload_rate,
                "planned_duration_seconds": session.get("planned_duration_seconds", 7200),
                "capacity_headroom_basis_points": session.get(
                    "capacity_headroom_basis_points", 2500
                ),
                "minimum_reserve_bytes": session.get(
                    "minimum_reserve_bytes", 107374182400
                ),
                "disk_check_interval_ms": session.get("disk_check_interval_ms", 1000),
                "pps_required": timing["pps_required"],
                "fx10e_expected_identity": hsi["fx10e"]["expected_identity"],
                "swir_expected_identity": hsi["swir"]["expected_identity"],
                "rgb_expected_identity": document["rgb"]["expected_serial"],
                "rgb_status_device_id": document["rgb"]["expected_serial"],
                "thermal_expected_identity": document["thermal"]["expected_device_id"],
                "thermal_status_device_id": document["thermal"]["expected_device_id"],
                **throughput_evidence,
            }],
        ),
        Node(
            package="ppbng_core", executable="time_authority_node",
            name="time_authority", output="screen", parameters=[{
                "pps_topic": "/timing/pps_anchor",
                "raw_trigger_topic": "/timing/trigger_event",
                "gnss_topic": "/gnss/observation",
                "output_topic": "/acquisition/trigger",
                # This must remain false until the serial-output-delay/PPS pairing
                # has been established with an approved bench validation.
                "pps_gnss_evidence_confirmed": False,
                **document.get("time_authority", {}),
                "pps_required": timing["pps_required"],
            }],
        ),
        Node(
            package="ppbng_core", executable="frame_context_node", name="context",
            output="screen", parameters=[{
                "allowed_output_root": output_root,
                "rgb_frame_topic": "/rgb/frame_metadata",
                "thermal_frame_topic": "/thermal/frame_metadata",
                **document.get("frame_context", {}),
            }], remappings=[("/context/fault_event", "/acquisition/fault_event")],
        ),
        # Keep Pleora FX10e and NI SWIR callbacks in separate address spaces.
        # Their production start transactions are serialized across processes
        # by hsi_production_node's Windows named mutex.
        Node(
            package="ppbng_hsi", executable="hsi_production_node",
            namespace="fx10e", name="camera", output="screen",
            additional_env=hsi_environment, parameters=[fx10e_parameters],
            remappings=hsi_remappings,
        ),
        Node(
            package="ppbng_hsi", executable="hsi_production_node",
            namespace="swir", name="camera", output="screen",
            additional_env=hsi_environment, parameters=[swir_parameters],
            remappings=hsi_remappings,
        ),
        Node(
            package="ppbng_rgb", executable="ppbng_rgb_camera_node", name="rgb",
            output="screen", parameters=[{
                "allow_hardware_access": True, "allowed_output_root": output_root,
                "device_id": document["rgb"]["expected_serial"], "stream_name": "rgb",
                "trigger_channel": "rgb", "trigger_topic": "/acquisition/trigger",
                **document["rgb"].get("acquisition", {}),
            }],
            remappings=[("/rgb/device_status", "/acquisition/device_status"),
                        ("/rgb/fault_event", "/acquisition/fault_event")],
        ),
        Node(
            package="ppbng_thermal", executable="ppbng_thermal_camera_node", name="thermal",
            output="screen",
            # Spinnaker 4.4 and the ROS pixi runtime ship incompatible DLLs
            # named libiomp5md.dll. Scope vendor-first resolution to this one
            # process so the camera node can load without changing global PATH.
            additional_env={
                "PATH": (
                    "C:/Program Files/Teledyne/Spinnaker/bin64/vs2015;"
                    + os.environ.get("PATH", "")
                )
            },
            parameters=[{
                "allow_hardware_access": True, "allowed_output_root": output_root,
                "device_id": document["thermal"]["expected_device_id"],
                "expected_model": document["thermal"].get("expected_model", "A6701"),
                "stream_name": "thermal", "trigger_channel": "thermal",
                "trigger_topic": "/acquisition/trigger",
                **document["thermal"].get("acquisition", {}),
            }],
            remappings=[("/thermal/device_status", "/acquisition/device_status"),
                        ("/thermal/fault_event", "/acquisition/fault_event")],
        ),
        Node(
            package="ppbng_gnss", executable="um982_production_node", namespace="gnss",
            name="receiver", output="screen", parameters=[{
                "hardware_enabled": True, "allowed_output_root": output_root,
                "com_path": document["gnss"]["port"],
                "baud_rate": document["gnss"].get("baud_rate", 115200),
                "device_id": "gnss",
                **document["gnss"].get("recovery", {}),
            }], remappings=[("status", "/acquisition/device_status"),
                            ("fault_event", "/acquisition/fault_event")],
        ),
        Node(
            package="ppbng_rsm400", executable="rsm400_production_node", name="rsm400",
            output="screen", parameters=[{
                "hardware_enabled": True, "com_port": document["rsm400"]["port"],
                "allowed_output_root": output_root,
                "storage_stem": document["rsm400"].get("storage_stem", "rsm400"),
                "flush_every_records": document["rsm400"].get("flush_every_records", 10),
                "device_id": "rsm400",
                "allow_control": rsm400.get("allow_control", False),
                "features_confirmed": document["rsm400"].get("features_confirmed", False),
                "of002_available": document["rsm400"].get("of002_available", False),
                "of005_available": document["rsm400"].get("of005_available", False),
                "require_ready_telemetry": True,
                "stab_motion_status_confirmed": True,
                "expected_stab_motion_status": expected_stab,
                "maximum_ready_error_level": maximum_ready_error,
                "ready_timeout_ms": ready_timeout_ms,
            }], remappings=[("/rsm400/status", "/acquisition/device_status"),
                            ("/rsm400/fault_event", "/acquisition/fault_event")],
        ),
        Node(
            package="ppbng_timing", executable="timing_production_node", namespace="timing",
            name="controller", output="screen", parameters=[{
                "hardware_enabled": True, "allowed_output_root": output_root,
                "trusted_usb_identity_mapping": timing.get("trusted_usb_identity_mapping", False),
                "com_path": timing["port"], "baud_rate": timing.get("baud_rate", 115200),
                "usb_vid": timing.get("usb_vid", 0), "usb_pid": timing.get("usb_pid", 0),
                "usb_serial": timing.get("usb_serial", ""), "device_id": "timing",
                "schedule_id": timing.get("schedule_id", 1),
                "ticks_per_second": timing["ticks_per_second"],
                **{f"{channel}.{key}": value for channel, values in timing["channels"].items()
                   for key, value in values.items()},
            }], remappings=[("device_status", "/acquisition/device_status"),
                            ("fault_event", "/acquisition/fault_event")],
        ),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_enabled",
                default_value="false",
                description="Must be passed explicitly as true for production hardware nodes",
            ),
            DeclareLaunchArgument(
                "machine_config",
                default_value="",
                description="Nonempty, locally verified machine configuration file",
            ),
            DeclareLaunchArgument(
                "machine_id",
                default_value="",
                description="Non-placeholder identity for this industrial PC",
            ),
            OpaqueFunction(function=_validate_and_create_nodes),
        ]
    )
