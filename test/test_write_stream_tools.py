from pathlib import Path
import json
import subprocess
import sys

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from analyze_write_stream import analyze


def test_temporal_envelope_does_not_count_missing_reply_as_success(tmp_path):
    (tmp_path / "sample.complete").write_text(json.dumps({
        "cycles_file": "sample_cycles.csv", "rx_file": "sample_rx.csv", "fault": "none",
        "dropped_cycles": 0, "dropped_rx": 0}))
    (tmp_path / "sample_cycles.csv").write_text(
        "#period_ns=1000,can_ids=1;2,tx_trigger=write,direct_feedback=1\n"
        "send_kind,send_success,send_before_steady_ns,command_published_ns,command_sequence\n"
        "2,1,110,100,1\n")
    (tmp_path / "sample_control.csv").write_text(
        "#dropped_control_cycles=0\n"
        "cycle,joint_index,read_start_ns,read_end_ns,acquired_ns,write_end_ns,command_published_ns,"
        "rx_sequence,rx_published_ns,attempts,consecutive_failures\n"
        "1,0,50,60,55,105,100,0,0,2,1\n"
        "1,1,50,60,55,105,100,0,0,2,1\n"
        "2,0,1050,1060,1055,1105,1100,1,200,1,0\n"
        "2,1,1050,1060,1055,1105,1100,0,0,2,1\n")
    (tmp_path / "sample_rx.csv").write_text(
        "#format_version=3\ncan_id,published_steady_ns,rx_sequence\n1,200,1\n2,1200,1\n")
    result = analyze(tmp_path)[0]
    assert result["next_read_temporal_envelope_eligible"] == 2
    assert result["next_read_temporal_envelope_misses"] == 1
    assert result["axes"]["1"]["read_latest_misses"] == 0
    assert result["axes"]["2"]["fallbacks"] == 2


def test_config_generator_requires_margin_and_never_overwrites(tmp_path):
    command = [sys.executable, str(ROOT / "scripts/make_write_stream_config.py"),
               str(ROOT / "hil/single_joint/bxi_hardware_stream_1khz.yaml")]
    invalid = tmp_path / "invalid.yaml"
    assert subprocess.run(command + [str(invalid), "--grace-ms", "2"], capture_output=True).returncode != 0
    assert not invalid.exists()
    valid = tmp_path / "valid.yaml"
    assert subprocess.run(command + [str(valid), "--grace-ms", "0.5"], capture_output=True).returncode == 0
    contents = valid.read_text()
    assert "tx_trigger: write" in contents
    assert subprocess.run(command + [str(valid), "--grace-ms", "0.4"], capture_output=True).returncode != 0
    assert valid.read_text() == contents
