from pathlib import Path
import sys

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "hil/single_joint"))

from analyze_stream_timing import summarize  # noqa: E402

METADATA = ("#period_ns=1000000,max_feedback_age_ns=3000000,broadcast=1,can_ids=1;2,"
            "dropped_cycles=0,dropped_rx=0,tx_thread=SCHED_FIFO_priority=60_cpus=2,"
            "rx_thread=SCHED_OTHER_priority=0_cpus=0-3\n")


def write_capture(tmp_path):
    stem = tmp_path / "bxi_stream_vcan0_1"
    cycles = [
        "cycle,outcome,wake_lateness_ns,hook_ns,send_lateness_ns,frames,"
        "age_ns_1,fresh_1,age_ns_2,fresh_2",
        # Before the first send the ages are -1.
        "0,sent,50000,10000,60000,1,-1,0,-1,0",
        "1,sent,40000,10000,50000,1,500000,1,700000,1",
        # Motor 2 has no new reply here, so its age grows past one period.
        "2,sent,300000,10000,320000,1,450000,1,1700000,0",
        "3,skipped_late,1100000,0,-1,0,600000,1,2700000,1",
    ]
    (Path(f"{stem}_cycles.csv")).write_text(METADATA + "\n".join(cycles) + "\n")
    rx = [
        "can_id,received_ns,rx_kernel_realtime_ns,rx_userspace_realtime_ns,rx_hardware_raw_ns",
        "1,500000,100000,110000,0",
        "2,700000,200000,230000,0",
        "1,1550000,300000,305000,0",
        "1,2600000,400000,420000,0",
    ]
    (Path(f"{stem}_rx.csv")).write_text(METADATA + "\n".join(rx) + "\n")
    return stem


def test_summarizes_outcomes_lateness_and_per_motor_age(tmp_path):
    lines = summarize(write_capture(tmp_path))
    text = "\n".join(lines)
    assert "motors=[1, 2]" in text
    assert "send thread: SCHED_FIFO_priority=60_cpus=2" in text
    assert "outcomes: sent=3, skipped_late=1" in text
    # 60, 50, 320 us: one send over the 0.1 ms tolerance.
    assert "send lateness from slot start:" in text and "over 0.1 ms=1/3" in text
    # Send times 0.06, 1.05, 2.32 ms: the second interval (1.27 ms) exceeds period + 0.1 ms.
    assert "over period+0.1 ms=1/2" in next(line for line in lines if line.startswith("send interval"))
    motor2 = next(line for line in lines if line.startswith("motor 2 feedback age"))
    assert "max=2.7000" in motor2 and "over one period=2/3" in motor2
    assert "motor 2: over max age=0, cycles without a new reply=1/3" in text
    assert "motor 1: kernel timestamps=3/3, hardware timestamps=0/3" in text
    kernel_to_user = next(line for line in lines if line.startswith("motor 1 kernel RX"))
    assert "max=0.0200" in kernel_to_user


def test_tolerance_changes_the_late_counts(tmp_path):
    lines = summarize(write_capture(tmp_path), tolerance_ms=0.5)
    assert "over 0.5 ms=0/3" in next(line for line in lines if line.startswith("send lateness"))
