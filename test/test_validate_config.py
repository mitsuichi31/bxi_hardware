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


def test_rejects_motor_can_id_16():
    errors = validate(
        ROOT / "test/config/invalid_can_id16.yaml",
        ROOT / "config/bxi_hardware.schema.json",
        ROOT / "config/specs",
    )
    assert any("greater than the maximum of 15" in error for error in errors)


def test_accepts_motor_can_id_15(tmp_path):
    valid = (ROOT / "test/config/valid.yaml").read_text()
    config = tmp_path / "id15.yaml"
    config.write_text(valid.replace("can_id: 1\n", "can_id: 15\n", 1))
    assert validate(config, ROOT / "config/bxi_hardware.schema.json", ROOT / "config/specs") == []


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


TRANSPORT_BASE = """
active_can_buses: [can0]
can_frame_format: {{can0: {fmt}}}
defaults: {{control_timeout_ms: 3, maximum_consecutive_timeouts: 3, can_hz: {hz}}}
can_transport:
  can0: {transport}
joints:
{joints}
"""
JOINT = "  j{id}: {{enabled: true, can_bus: can0, can_id: {id}, motor_dir: 1, motor_type: MOTOR_70, lower: -1, upper: 1}}\n"


def transport_errors(tmp_path, transport, ids=(1, 2, 3), fmt="fd", hz=1000):
    joints = "".join(JOINT.format(id=can_id) for can_id in ids)
    return run(tmp_path / "transport.yaml", tmp_path,
               TRANSPORT_BASE.format(fmt=fmt, hz=hz, transport=transport, joints=joints))


def test_accepts_streamed_broadcast_and_unicast(tmp_path):
    assert transport_errors(
        tmp_path, "{rx_mode: separate, tx_frame: broadcast, stream_feedback_max_age_ms: 3}") == []
    assert transport_errors(
        tmp_path, "{rx_mode: separate, stream_feedback_max_age_ms: 15}", ids=(1, 5), hz=200) == []


def test_streamed_bus_requires_max_age_of_at_least_two_periods(tmp_path):
    for transport in ("{rx_mode: separate}",
                      "{rx_mode: separate, stream_feedback_max_age_ms: 1.5}"):
        errors = transport_errors(tmp_path, transport)
        assert any("at least two periods" in error for error in errors), transport
    errors = transport_errors(tmp_path, "{rx_mode: separate, stream_feedback_max_age_ms: 60}")
    assert any("greater than the maximum of 50" in error for error in errors)


def test_command_max_age_is_optional_and_at_least_two_periods(tmp_path):
    base = "{rx_mode: separate, stream_feedback_max_age_ms: 3"
    assert transport_errors(tmp_path, base + "}") == []
    assert transport_errors(tmp_path, base + ", stream_command_max_age_ms: 5}") == []
    errors = transport_errors(tmp_path, base + ", stream_command_max_age_ms: 1.5}")
    assert any("stream_command_max_age_ms must be at least two periods" in e for e in errors)
    errors = transport_errors(tmp_path, base + ", stream_command_max_age_ms: 60}")
    assert any("greater than the maximum of 50" in error for error in errors)
    errors = transport_errors(tmp_path, "{stream_command_max_age_ms: 5}")
    assert any("require rx_mode: separate" in error for error in errors)


def test_stream_options_require_separate_rx(tmp_path):
    errors = transport_errors(tmp_path, "{tx_frame: broadcast}")
    assert any("require rx_mode: separate" in error for error in errors)
    errors = transport_errors(tmp_path, "{send_thread_priority: 60}")
    assert any("require rx_mode: separate" in error for error in errors)


def test_thread_settings_are_range_checked(tmp_path):
    base = "{rx_mode: separate, stream_feedback_max_age_ms: 3, "
    assert transport_errors(
        tmp_path, base + "send_thread_priority: 60, receive_thread_priority: 61, "
        "thread_cpus: [2, 3]}") == []
    errors = transport_errors(tmp_path, base + "send_thread_priority: 99}")
    assert any("greater than the maximum of 98" in error for error in errors)
    errors = transport_errors(tmp_path, base + "thread_cpus: []}")
    assert any("[] is too short" in error for error in errors)


def test_broadcast_needs_fd_and_ids_one_to_n(tmp_path):
    base = "{rx_mode: separate, tx_frame: broadcast, stream_feedback_max_age_ms: 3}"
    assert any("needs CAN FD" in error for error in transport_errors(tmp_path, base, fmt="classic"))
    assert any("IDs 1..N" in error for error in transport_errors(tmp_path, base, ids=(1, 3)))
    assert any("IDs 1..N" in error for error in transport_errors(tmp_path, base, ids=(1, 2, 3, 4, 5)))


def test_isolated_single_motor_broadcast_is_limited_to_can0_id1_fd(tmp_path):
    isolated = ("{rx_mode: separate, tx_frame: broadcast, stream_feedback_max_age_ms: 3, "
                "hil_isolated_id1_broadcast_blocks: 3}")
    assert transport_errors(tmp_path, isolated, ids=(1,)) == []
    assert any("hil_isolated" in error for error in transport_errors(tmp_path, isolated, ids=(1, 2)))
    assert any("hil_isolated" in error
               for error in transport_errors(tmp_path, isolated, ids=(1,), fmt="fd_brs"))
    errors = transport_errors(tmp_path, isolated.replace("blocks: 3", "blocks: 4"), ids=(1,))
    assert any("3 was expected" in error for error in errors)
