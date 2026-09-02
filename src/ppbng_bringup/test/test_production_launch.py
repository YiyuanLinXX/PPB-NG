import copy
import importlib.util
import json
from pathlib import Path
import sys

import pytest
import yaml


SOURCE_ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "ppbng_production_launch", SOURCE_ROOT / "launch" / "production.launch.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

TOOLS_ROOT = SOURCE_ROOT.parents[1] / "tools"
sys.path.insert(0, str(TOOLS_ROOT))
import storage_qualification_plan as STORAGE_PLAN


def example():
    return yaml.safe_load((SOURCE_ROOT / "config" / "machine.example.yaml").read_text("utf-8"))


def unified_config():
    return yaml.safe_load((SOURCE_ROOT / "config" / "ppbng_config.yaml").read_text("utf-8"))


def test_unified_config_is_headless_and_uses_internal_continuous_hsi():
    document = unified_config()
    assert document["hsi"]["fx10e"]["grabber_channel"] == "192.168.10.2"
    assert document["hsi"]["fx10e"]["grabber_channel"].lower() != "ui"
    assert document["hsi"]["fx10e"]["trigger_mode"] == "Internal"
    assert document["hsi"]["swir"]["trigger_mode"] == "Internal"
    assert document["hsi"]["fx10e"]["fail_fast_on_sample_fault"] is True
    assert document["hsi"]["swir"]["fail_fast_on_sample_fault"] is True
    assert document["timing"]["channels"]["fx10e"]["enabled"] is False
    assert document["timing"]["channels"]["swir"]["enabled"] is False
    MODULE._validate_hsi_continuous_mode(document)


def test_four_camera_workflow_matches_validated_uno_r4_firmware():
    document = unified_config()
    assert document["timing"]["ticks_per_second"] == 1_000_000
    for camera in ("rgb", "thermal"):
        channel = document["timing"]["channels"][camera]
        assert channel == {
            "enabled": True,
            "rate_numerator_hz": 2,
            "rate_denominator": 1,
            "pulse_width_ticks": 1000,
            "phase_ticks": 0,
        }
    assert document["thermal"]["acquisition"]["frame_sync_evidence_confirmed"] is True


def test_four_camera_launch_is_modular_and_arms_ascii_trigger_last():
    launch_source = (SOURCE_ROOT / "launch" / "four_camera.launch.py").read_text("utf-8")
    workflow_source = (SOURCE_ROOT.parents[1] / "tools" /
                       "run_four_camera_test.ps1").read_text("utf-8")
    assert 'executable="hsi_production_node"' in launch_source
    assert 'executable="ppbng_rgb_camera_node"' in launch_source
    assert 'executable="ppbng_thermal_camera_node"' in launch_source
    assert 'executable="uno_r4_ascii_trigger_node"' in launch_source
    assert workflow_source.index("foreach ($camera in @('rgb', 'thermal'))") < \
        workflow_source.rindex("'/timing/arm_immediate'")
    assert workflow_source.rindex("'/timing/disarm_keep_config'") < \
        workflow_source.rindex("foreach ($camera in @('rgb', 'thermal', 'fx10e', 'swir'))")


@pytest.mark.parametrize("camera", ["fx10e", "swir"])
def test_production_rejects_external_hsi_trigger_or_enabled_hsi_timing_output(camera):
    document = unified_config()
    document["hsi"][camera]["trigger_mode"] = "External"
    with pytest.raises(RuntimeError, match="Internal continuous timing"):
        MODULE._validate_hsi_continuous_mode(document)

    document = unified_config()
    document["timing"]["channels"][camera]["enabled"] = True
    with pytest.raises(RuntimeError, match="must be disabled"):
        MODULE._validate_hsi_continuous_mode(document)


def test_production_rejects_fx10e_ui_channel_and_invalid_shared_writer_settings():
    document = unified_config()
    document["hsi"]["fx10e"]["grabber_channel"] = "ui"
    with pytest.raises(RuntimeError, match="headless grabber_channel"):
        MODULE._validate_hsi_continuous_mode(document)

    document = unified_config()
    document["session"]["hsi_flush_every_lines"] = 0
    with pytest.raises(RuntimeError, match="hsi_flush_every_lines"):
        MODULE._validate_hsi_continuous_mode(document)


def test_production_batch_timeout_exceeds_hsi_sdk_initialization_timeout():
    document = unified_config()
    assert document["runtime"]["batch_timeout_ms"] > document["hsi"]["initialization_timeout_ms"]
    document["runtime"]["batch_timeout_ms"] = document["hsi"]["initialization_timeout_ms"]
    with pytest.raises(RuntimeError, match="must exceed"):
        MODULE._validate_hsi_continuous_mode(document)


def test_exact_config_reader_preserves_windows_crlf(tmp_path):
    path = tmp_path / "machine.yaml"
    path.write_bytes(b"schema_version: 1\r\nsafety:\r\n  configured: true\r\n")
    assert MODULE._read_utf8_exact(path).count("\r\n") == 3


def test_example_payload_rate_is_exact_and_includes_thermal_header_row():
    assert MODULE._estimated_payload_rate(example()) == 162_531_840


def test_unified_storage_plan_matches_production_rate_and_current_threshold():
    document = unified_config()
    assert MODULE._estimated_payload_rate(document) == 76_318_659
    assert STORAGE_PLAN.estimated_payload_rate(document) == 76_318_659
    plan = STORAGE_PLAN.make_plan(document, available_bytes=2_000_000_000_000)
    assert plan["required_durable_bytes_per_second"] == 95_398_324
    assert plan["planned_task_required_available_bytes"] == 794_242_113_400


def test_hsi_payload_rate_is_independent_of_disabled_external_trigger_channels():
    document = copy.deepcopy(example())
    baseline = MODULE._estimated_payload_rate(document)
    document["timing"]["channels"]["fx10e"]["enabled"] = False
    document["timing"]["channels"]["fx10e"]["rate_numerator_hz"] = 119.5
    assert MODULE._estimated_payload_rate(document) == baseline


def test_disabled_or_invalid_timing_channel_is_rejected():
    document = copy.deepcopy(example())
    document["timing"]["channels"]["thermal"]["enabled"] = False
    with pytest.raises(RuntimeError, match="must be enabled"):
        MODULE._estimated_payload_rate(document)


def test_fractional_payload_dimensions_are_rejected():
    document = copy.deepcopy(example())
    document["hsi"]["fx10e"]["spatial_samples"] = 1024.5
    with pytest.raises(RuntimeError, match="positive integer"):
        MODULE._estimated_payload_rate(document)


def test_nonpositive_hsi_internal_line_rate_is_rejected():
    document = copy.deepcopy(example())
    document["hsi"]["fx10e"]["line_rate_hz"] = 0.0
    with pytest.raises(RuntimeError, match="finite and positive"):
        MODULE._estimated_payload_rate(document)


def test_hsi_queue_bounds_accept_documented_two_camera_profile():
    MODULE._validate_hsi_queue_bounds(example())


def test_hsi_queue_bounds_reject_oversize_or_trigger_underprovisioning():
    document = copy.deepcopy(example())
    document["hsi"]["fx10e"]["callback_queue_capacity"] = 513
    with pytest.raises(RuntimeError, match="exceeds 512"):
        MODULE._validate_hsi_queue_bounds(document)

    document = copy.deepcopy(example())
    document["hsi"]["swir"]["pending_trigger_capacity"] = 127
    with pytest.raises(RuntimeError, match="must be at least"):
        MODULE._validate_hsi_queue_bounds(document)


def test_dual_hsi_arguments_keep_camera_parameters_and_remaps_separate():
    arguments = MODULE._dual_hsi_arguments(
        {"camera_kind": "fx10e", "device_index": 3, "hardware_enabled": True},
        {"camera_kind": "swir", "device_index": 8, "hardware_enabled": True},
        [("status", "/acquisition/device_status")],
    )
    fx_values = [arguments[index + 1] for index, value in enumerate(arguments[:-1])
                 if value == "--fx10e-param"]
    swir_values = [arguments[index + 1] for index, value in enumerate(arguments[:-1])
                   if value == "--swir-param"]
    assert "camera_kind:=\"fx10e\"" in fx_values
    assert "camera_kind:=\"swir\"" not in fx_values
    assert "camera_kind:=\"swir\"" in swir_values
    assert "camera_kind:=\"fx10e\"" not in swir_values
    assert arguments.count("--fx10e-remap") == 1
    assert arguments.count("--swir-remap") == 1


def test_production_launch_isolates_pleora_and_ni_hsi_processes():
    source = (SOURCE_ROOT / "launch" / "production.launch.py").read_text("utf-8")
    assert 'executable="hsi_dual_production_node"' not in source
    assert 'executable="hsi_production_node",' in source
    assert 'namespace="fx10e", name="camera"' in source
    assert 'namespace="swir", name="camera"' in source

def test_example_declares_fail_closed_production_safety_policy():
    document = example()
    document["safety"]["configured"] = True
    document["safety"]["hardware_enabled"] = True
    MODULE._validate_safety_policy(document)


def test_pps_requirement_is_an_explicit_boolean_and_defaults_optional():
    document = example()
    assert document["timing"]["pps_required"] is False
    document["safety"]["configured"] = True
    document["safety"]["hardware_enabled"] = True
    MODULE._validate_safety_policy(document)
    for invalid in (None, 0, "false"):
        candidate = copy.deepcopy(document)
        candidate["timing"]["pps_required"] = invalid
        with pytest.raises(RuntimeError, match="pps_required"):
            MODULE._validate_safety_policy(candidate)


def test_safety_policy_rejects_schema_or_privileged_operations():
    for path in (
        ("schema_version",),
        ("safety", "allow_device_reset"),
        ("safety", "allow_network_reconfiguration"),
        ("safety", "allow_firmware_update"),
    ):
        document = example()
        document["safety"]["configured"] = True
        document["safety"]["hardware_enabled"] = True
        target = document
        for key in path[:-1]:
            target = target[key]
        target[path[-1]] = 2 if path == ("schema_version",) else True
        with pytest.raises(RuntimeError):
            MODULE._validate_safety_policy(document)


def test_safety_policy_requires_receive_only_gnss_and_observe_only_rsm_startup():
    for section, key in (("gnss", "receive_only"),
                         ("rsm400", "observe_only_on_launch")):
        document = example()
        document["safety"]["configured"] = True
        document["safety"]["hardware_enabled"] = True
        document[section][key] = False
        with pytest.raises(RuntimeError):
            MODULE._validate_safety_policy(document)

def _write_evidence(root, **updates):
    evidence = {
        "schema_version": 1,
        "tool": "ppbng_storage_durable_write_qualification",
        "normalized_output_root": str(root.resolve()),
        "volume_root": "D:/",
        "volume_serial_hex": "A1B2C3D4",
        "filesystem": "NTFS",
        "bytes_written": 1_048_576_000,
        "duration_seconds": 5.0,
        "bytes_per_second": 209_715_200.0,
        "parameters": {
            "requested_duration_seconds": 5,
            "block_bytes": 8 * 1024 ** 2,
            "maximum_test_bytes": 1024 ** 3,
            "reserve_bytes": 100 * 1024 ** 3,
            "write_through": True,
            "flush_file_buffers": True,
        },
    }
    evidence.update(updates)
    path = root / "ppbng_durable_evidence_test.json"
    path.write_text(json.dumps(evidence), encoding="utf-8")
    return path


def test_throughput_evidence_is_derived_not_manually_asserted(tmp_path):
    evidence = _write_evidence(tmp_path)
    result = MODULE._load_throughput_evidence(
        {"throughput_evidence_path": str(evidence)}, tmp_path,
        volume_identity=lambda _path: ("A1B2C3D4", "NTFS", "D:/"),
    )
    assert result["throughput_qualified"] is True
    assert result["qualified_durable_bytes_per_second"] == 209_715_200


def test_throughput_evidence_rejects_wrong_volume_or_missing_flush(tmp_path):
    evidence = _write_evidence(tmp_path)
    with pytest.raises(RuntimeError, match="serial has changed"):
        MODULE._load_throughput_evidence(
            {"throughput_evidence_path": str(evidence)}, tmp_path,
            volume_identity=lambda _path: ("00000000", "NTFS", "D:/"),
        )
    _write_evidence(
        tmp_path, parameters={
            "requested_duration_seconds": 5,
            "block_bytes": 8 * 1024 ** 2,
            "maximum_test_bytes": 1024 ** 3,
            "reserve_bytes": 100 * 1024 ** 3,
            "write_through": True,
            "flush_file_buffers": False,
        }
    )
    with pytest.raises(RuntimeError, match="durable flush evidence"):
        MODULE._load_throughput_evidence(
            {"throughput_evidence_path": str(evidence)}, tmp_path,
            volume_identity=lambda _path: ("A1B2C3D4", "NTFS", "D:/"),
        )


def test_throughput_evidence_requires_tool_probe_bounds(tmp_path):
    evidence = _write_evidence(tmp_path)
    document = json.loads(evidence.read_text(encoding="utf-8"))
    del document["parameters"]["maximum_test_bytes"]
    evidence.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(RuntimeError, match="maximum test size"):
        MODULE._load_throughput_evidence(
            {"throughput_evidence_path": str(evidence)}, tmp_path,
            volume_identity=lambda _path: ("A1B2C3D4", "NTFS", "D:/"),
        )
