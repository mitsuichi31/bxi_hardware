from pathlib import Path
import sys

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from validate_config import validate  # noqa: E402


def run(config: Path, tmp_path: Path, text: str):
    config.write_text(text)
    return validate(config, ROOT / "config/bxi_hardware.schema.json", ROOT / "config/specs")


def test_valid_fixture():
    assert validate(
        ROOT / "test/config/valid.yaml",
        ROOT / "config/bxi_hardware.schema.json",
        ROOT / "config/specs",
    ) == []


def test_rejects_duplicate_can_id(tmp_path):
    errors = run(tmp_path / "duplicate.yaml", tmp_path, """
active_can_buses: [can0]
defaults: {control_timeout_ms: 3, maximum_consecutive_timeouts: 3, can_hz: 200}
joints:
  a: {enabled: true, can_bus: can0, can_id: 1, motor_dir: 1, motor_type: MOTOR_70, lower: -1, upper: 1}
  b: {enabled: true, can_bus: can0, can_id: 1, motor_dir: 1, motor_type: MOTOR_70, lower: -1, upper: 1}
""")
    assert any("duplicate CAN ID" in error for error in errors)


def test_rejects_unknown_bus_limit_conflict_and_missing_spec(tmp_path):
    errors = run(tmp_path / "invalid.yaml", tmp_path, """
active_can_buses: [can0]
defaults: {control_timeout_ms: 3, maximum_consecutive_timeouts: 3, can_hz: 200}
joints:
  a: {enabled: true, can_bus: can9, can_id: 1, motor_dir: 1, motor_type: TBD, lower: 2, upper: 1}
""")
    assert any("unknown or inactive bus" in error for error in errors)
    assert any("lower must" in error for error in errors)
    assert any("missing motor spec" in error for error in errors)


def test_accepts_per_bus_frame_format(tmp_path):
    errors = run(tmp_path / "fd.yaml", tmp_path, """
active_can_buses: [can0, can1]
can_frame_format: {can0: fd_brs, can1: classic}
defaults: {control_timeout_ms: 3, maximum_consecutive_timeouts: 3, can_hz: 200,
           temperature_sensor_fault_below_c: -20.0}
joints:
  a: {enabled: true, can_bus: can0, can_id: 1, motor_dir: 1, motor_type: MOTOR_70, lower: -1, upper: 1}
""")
    assert errors == []


def test_rejects_unknown_frame_format_and_bus(tmp_path):
    errors = run(tmp_path / "bad_fd.yaml", tmp_path, """
active_can_buses: [can0]
can_frame_format: {can0: fd_fast, can9: fd}
defaults: {control_timeout_ms: 3, maximum_consecutive_timeouts: 3, can_hz: 200}
joints:
  a: {enabled: false}
""")
    assert any("is not one of ['classic', 'fd', 'fd_brs']" in error for error in errors)
    errors = run(tmp_path / "bad_bus.yaml", tmp_path, """
active_can_buses: [can0]
can_frame_format: {can9: fd}
defaults: {control_timeout_ms: 3, maximum_consecutive_timeouts: 3, can_hz: 200}
joints:
  a: {enabled: false}
""")
    assert any("can_frame_format: unknown or inactive bus 'can9'" in error for error in errors)


def test_rejects_invalid_motor_direction(tmp_path):
    errors = run(tmp_path / "invalid_direction.yaml", tmp_path, """
active_can_buses: [can0]
defaults: {control_timeout_ms: 3, maximum_consecutive_timeouts: 3, can_hz: 200}
joints:
  a: {enabled: true, can_bus: can0, can_id: 1, motor_dir: 0, motor_type: MOTOR_70, lower: -1, upper: 1}
""")
    assert any("not one of [-1, 1]" in error for error in errors)
