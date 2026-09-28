from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "hil/single_joint"))

from compare_stream_scheduling import write_config  # noqa: E402

sys.path.insert(0, str(ROOT / "scripts"))
from validate_config import validate  # noqa: E402

TEMPLATE = ROOT / "hil/single_joint/bxi_hardware_stream_1khz.yaml"


def generated(tmp_path, send, receive, cpu):
    path = tmp_path / "config.yaml"
    write_config(TEMPLATE, path, "vcan0", send, receive, cpu)
    errors = validate(path, ROOT / "config/bxi_hardware.schema.json", ROOT / "config/specs")
    assert errors == []
    return yaml.safe_load(path.read_text())


def test_moves_the_template_to_vcan_and_keeps_sched_other_by_default(tmp_path):
    config = generated(tmp_path, 0, 0, None)
    assert config["active_can_buses"] == ["vcan0"]
    assert config["joints"]["joint1"]["can_bus"] == "vcan0"
    transport = config["can_transport"]["vcan0"]
    assert transport["rx_mode"] == "separate"
    assert not {"send_thread_priority", "receive_thread_priority", "thread_cpus"} & set(transport)


def test_adds_the_requested_thread_settings(tmp_path):
    transport = generated(tmp_path, 60, 61, 3)["can_transport"]["vcan0"]
    assert transport["send_thread_priority"] == 60
    assert transport["receive_thread_priority"] == 61
    assert transport["thread_cpus"] == [3]


def test_summary_separates_outcomes_and_reports_dropped_replies(tmp_path):
    from compare_stream_scheduling import summarize_run

    run_dir = tmp_path / "other_cpu_r5"
    run_dir.mkdir()
    metadata = ("#period_ns=1000000,max_feedback_age_ns=3000000,broadcast=1,can_ids=1,"
                "dropped_cycles=0,dropped_rx=9,tx_thread=SCHED_OTHER_priority=0_cpus=0,"
                "rx_thread=SCHED_OTHER_priority=0_cpus=0\n")
    (run_dir / "bxi_stream_vcan0_1_cycles.csv").write_text(metadata + "\n".join([
        "cycle,outcome,wake_lateness_ns,hook_ns,send_lateness_ns,frames,age_ns_1,fresh_1",
        "0,sent,10000,1000,20000,1,-1,0",
        "1,sent,10000,1000,30000,1,600000,1",
        "2,skipped_late,1200000,0,-1,0,1800000,0",
        "3,stopped,2500000,0,-1,0,3100000,1",
    ]) + "\n")
    (run_dir / "bxi_stream_vcan0_1_rx.csv").write_text(metadata + "\n".join([
        "can_id,received_ns,rx_kernel_realtime_ns,rx_userspace_realtime_ns,rx_hardware_raw_ns",
        "1,400000,100,5100,0",
    ]) + "\n")
    (run_dir / "launch.log").write_text("[WARN] Overrun detected! ...\n[WARN] Overrun detected! ...\n")

    row = summarize_run(run_dir)
    assert (row["sent"], row["skipped_late"], row["stopped"]) == (2, 1, 1)
    assert row["latest_reply_age_max_ms"] == 3.1  # recorded on the stopped slot too
    assert row["latest_reply_age_over_1period"] == 2
    assert row["slots_without_new_reply"] == 1  # from fresh_1, not from the age
    assert abs(row["reply_to_send_max_ms"] - 0.62) < 1e-9  # 0.60 + (0.03 - 0.01)
    assert (row["replies_recorded"], row["dropped_rx"]) == (1, 9)
    assert row["cm_overrun_warnings"] == 2
