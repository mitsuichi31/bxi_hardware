#!/usr/bin/env python3
"""Compare stream-thread scheduling on vcan with controller_manager at 1 kHz (FIFO/50).

For each condition this writes a vcan copy of bxi_hardware_stream_1khz.yaml with the thread
settings, starts bxi_vcan_motor_emulator, runs single_joint_hil.launch.py with
controllers_1khz.yaml and BXI_WORKER_TIMING_DIR for a fixed time, stops everything with SIGINT,
and writes the CSVs, logs, and a summary table under the output directory.

Run inside the dev container on a CAN FD vcan (mtu 72) that nothing else uses. vcan only: this
never touches a real CAN interface.
"""

import argparse
import csv
import datetime
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).parent))
from analyze_stream_timing import (  # noqa: E402
    distribution, latest_reply_ages, read_csv, reply_to_send_times, slots_without_new_reply)

OUTCOMES = ("sent", "skipped_late", "skipped_burst", "expired_after_hook", "partial", "stopped")
RUN_NAME = re.compile(r"^(?P<condition>.+)_(?P<load>idle|cpu)_r(?P<replies>\d+)$")

# name -> (send_thread_priority, receive_thread_priority, pin to one CPU). 0 = SCHED_OTHER.
# controller_manager runs its update thread at SCHED_FIFO 50.
CONDITIONS = {
    "other": (0, 0, False),
    "fifo_below_cm": (45, 46, False),
    "fifo_above_cm": (60, 61, False),
    "fifo_above_cm_pinned": (60, 61, True),
    "rx_high": (45, 61, False),
}


def ros_prefix():
    return Path(subprocess.check_output(["ros2", "pkg", "prefix", "bxi_hardware"], text=True).strip())


def write_config(template, path, iface, send, receive, cpu):
    text = re.sub(r"\bcan0\b", iface, template.read_text())
    # The template carries provisional thread settings; each condition sets its own.
    text = re.sub(r"(?m)^\s*(send_thread_priority|receive_thread_priority|thread_cpus):.*\n", "",
                  text)
    settings = []
    if send:
        settings.append(f"    send_thread_priority: {send}")
    if receive:
        settings.append(f"    receive_thread_priority: {receive}")
    if cpu is not None:
        settings.append(f"    thread_cpus: [{cpu}]")
    if settings:
        text = text.replace("    stream_feedback_max_age_ms: 3.0\n",
                            "    stream_feedback_max_age_ms: 3.0\n" + "\n".join(settings) + "\n")
    path.write_text(text)


def start_cpu_hogs(count):
    # One busy loop per CPU at normal priority: SCHED_OTHER threads compete with them,
    # SCHED_FIFO threads preempt them.
    return [subprocess.Popen(["sh", "-c", "while :; do :; done"]) for _ in range(count)]


def stop(process, timeout=20.0):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def summarize_run(run_dir):
    """One summary row from a run directory's CSVs and launch log."""
    stems = sorted(run_dir.glob("bxi_stream_*_cycles.csv"))
    if not stems:
        return {"error": "no stream CSV"}
    stem = str(stems[-1])[:-len("_cycles.csv")]
    metadata, cycles = read_csv(Path(stem + "_cycles.csv"))
    _, replies = read_csv(Path(stem + "_rx.csv"))
    can_id = metadata["can_ids"].split(";")[0]
    period_ns = int(metadata["period_ns"])
    row = {"slots": len(cycles)}
    # Every slot outcome separately: a skipped slot sent nothing, and the final "stopped" row is
    # the slot in which a fault ended the stream.
    for outcome in OUTCOMES:
        row[outcome] = sum(cycle["outcome"] == outcome for cycle in cycles)
    wake = distribution([int(cycle["wake_lateness_ns"]) for cycle in cycles]) or {}
    sent = [int(cycle["send_lateness_ns"]) for cycle in cycles if int(cycle["send_lateness_ns"]) >= 0]
    send = distribution(sent) or {}
    ages = latest_reply_ages(cycles, can_id)
    age = distribution(ages) or {}
    to_send = distribution(reply_to_send_times(cycles, can_id)) or {}
    stamped = [int(reply["rx_userspace_realtime_ns"]) - int(reply["rx_kernel_realtime_ns"])
               for reply in replies if int(reply["rx_kernel_realtime_ns"]) > 0]
    rx = distribution(stamped) or {}
    launch_log = (run_dir / "launch.log").read_text(errors="replace")
    row.update({
        "wake_p99_ms": wake.get("p99"), "wake_max_ms": wake.get("max"),
        "send_p99_ms": send.get("p99"), "send_max_ms": send.get("max"),
        "send_over_0.1ms": sum(value > 100_000 for value in sent),
        # Stop decision value: age of the latest reply when the slot starts, sent or not.
        "latest_reply_age_max_ms": age.get("max"),
        "latest_reply_age_over_1period": sum(value > period_ns for value in ages),
        "slots_without_new_reply": slots_without_new_reply(cycles, can_id),
        # Sent slots only: latest reply to the actual send.
        "reply_to_send_p99_ms": to_send.get("p99"), "reply_to_send_max_ms": to_send.get("max"),
        "replies_recorded": len(replies),
        # When non-zero, the rx statistics cover only the recorded (earliest) replies.
        "dropped_rx": int(metadata["dropped_rx"]),
        "rx_read_p99_ms": rx.get("p99"), "rx_read_max_ms": rx.get("max"),
        # controller_manager "Overrun detected" warnings in the launch log.
        "cm_overrun_warnings": launch_log.count("Overrun detected"),
        "tx_thread": metadata["tx_thread"], "rx_thread": metadata["rx_thread"],
    })
    return row


def write_summary(out, rows):
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with (out / "summary.tsv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fields, delimiter="\t")
        writer.writeheader()
        writer.writerows(rows)
    print(f"summary: {out / 'summary.tsv'}")


def resummarize(out):
    """Rebuild summary.tsv from the run directories under out (no new measurement)."""
    rows = []
    for run_dir in sorted(path for path in out.iterdir() if path.is_dir()):
        match = RUN_NAME.match(run_dir.name)
        if not match:
            continue
        row = {"condition": match["condition"], "load": match["load"],
               "replies_per_command": int(match["replies"])}
        row.update(summarize_run(run_dir))
        rows.append(row)
    write_summary(out, rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iface", default="vcan0")
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--conditions", default=",".join(CONDITIONS))
    parser.add_argument("--replies", default="1,5", help="replies per command, comma separated")
    parser.add_argument("--loads", default="idle,cpu", help="idle and/or cpu (one busy loop per CPU)")
    parser.add_argument("--pin-cpu", type=int, default=os.cpu_count() - 1)
    # A real motor sits outside the host, so host load must not delay its replies. Without this
    # the CPU hogs starve the emulator and the stream stops for stale feedback instead.
    parser.add_argument("--emulator-priority", type=int, default=80,
                        help="SCHED_FIFO priority of the emulator (0 = SCHED_OTHER)")
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--resummarize", type=Path, default=None,
                        help="rebuild summary.tsv from an earlier output directory and exit")
    args = parser.parse_args()
    if args.resummarize:
        resummarize(args.resummarize)
        return
    if not args.iface.startswith("vcan"):
        parser.error("vcan interfaces only")
    if args.seconds * 1000 > 18000:
        parser.error("the stream trace holds 20,000 cycles; use at most 18 seconds")

    prefix = ros_prefix()
    share = prefix / "share/bxi_hardware/hil/single_joint"
    emulator = prefix / "lib/bxi_hardware/bxi_vcan_motor_emulator"
    out = args.out or Path("/ws/log") / (
        "stream_sched_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for name in args.conditions.split(","):
        send, receive, pinned = CONDITIONS[name]
        for load in args.loads.split(","):
            for replies in (int(value) for value in args.replies.split(",")):
                label = f"{name}_{load}_r{replies}"
                run_dir = out / label
                run_dir.mkdir()
                config = run_dir / "config.yaml"
                write_config(share / "bxi_hardware_stream_1khz.yaml", config, args.iface, send,
                             receive, args.pin_cpu if pinned else None)
                print(f"== {label}", flush=True)
                hogs = start_cpu_hogs(os.cpu_count()) if load == "cpu" else []
                with (run_dir / "emulator.log").open("w") as emulator_log, \
                        (run_dir / "launch.log").open("w") as launch_log:
                    realtime = (["chrt", "-f", str(args.emulator_priority)]
                                if args.emulator_priority > 0 else [])
                    motor = subprocess.Popen(
                        realtime + [str(emulator), "--iface", args.iface, "--ids", "1",
                                    "--replies-per-command", str(replies)],
                        stdout=emulator_log, stderr=subprocess.STDOUT)
                    time.sleep(0.5)
                    env = dict(os.environ, BXI_WORKER_TIMING_DIR=str(run_dir))
                    launch = subprocess.Popen(
                        ["ros2", "launch", "bxi_hardware", "single_joint_hil.launch.py",
                         f"config_file:={config}",
                         f"controllers_file:={share / 'controllers_1khz.yaml'}"],
                        stdout=launch_log, stderr=subprocess.STDOUT, env=env,
                        start_new_session=True)
                    # Startup (enable, spawners) takes a few seconds before the stream runs.
                    time.sleep(args.seconds + 3.0)
                    stop(launch)
                    stop(motor)
                for hog in hogs:
                    hog.kill()
                    hog.wait()
                row = {"condition": name, "load": load, "replies_per_command": replies}
                row.update(summarize_run(run_dir))
                rows.append(row)
                print("   " + ", ".join(f"{key}={value}" for key, value in row.items()
                                        if key not in ("tx_thread", "rx_thread")), flush=True)
    write_summary(out, rows)


if __name__ == "__main__":
    main()
