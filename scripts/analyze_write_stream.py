#!/usr/bin/env python3
"""Summarize control/TX/RX traces. Temporal association is not a MIT command echo."""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path


def rows(path):
    with path.open() as source:
        metadata = source.readline().strip()
        if not metadata.startswith("#"):
            raise ValueError(f"missing metadata: {path}")
        return list(csv.DictReader(source)), dict(
            item.split("=", 1) for item in metadata[1:].split(",") if "=" in item)


def distribution(values):
    values = sorted(values)
    if not values:
        return {"count": 0}
    return {"count": len(values), "min_ns": values[0], "max_ns": values[-1],
            **{f"p{p}_ns": values[max(0, math.ceil(len(values) * p / 100) - 1)]
               for p in (50, 99, 99.9)}}


def analyze(directory):
    summaries = []
    for complete in sorted(directory.rglob("*.complete")):
        info = json.loads(complete.read_text())
        cycle_path = complete.parent / info["cycles_file"]
        controls_path = cycle_path.with_name(cycle_path.name.replace("_cycles.csv", "_control.csv"))
        if not controls_path.exists():
            continue
        cycles, meta = rows(cycle_path)
        controls, control_meta = rows(controls_path)
        replies, _ = rows(complete.parent / info["rx_file"])
        period = int(meta["period_ns"])
        axes = sorted({int(c["joint_index"]) for c in controls})
        groups = {axis: [c for c in controls if int(c["joint_index"]) == axis
                         and int(c["write_end_ns"]) > 0] for axis in axes}
        first = groups[axes[0]]
        intervals = [int(b["command_published_ns"]) - int(a["command_published_ns"])
                     for a, b in zip(first, first[1:])]
        publication = {int(c["cycle"]): c for c in first}
        latencies = [int(c["send_before_steady_ns"]) - int(c["command_published_ns"])
                     for c in cycles if c.get("send_kind") == "2"
                     and c["send_success"] == "1" and int(c["command_published_ns"]) > 0]
        receive_by_axis = {}
        for reply in replies:
            if "published_steady_ns" in reply:
                receive_by_axis.setdefault(int(reply["can_id"]), []).append(
                    (int(reply["published_steady_ns"]), int(reply["rx_sequence"])))
        can_ids = list(map(int, meta["can_ids"].split(";")))
        per_axis = {}
        for axis, can_id in zip(axes, can_ids):
            samples = groups[axis]
            published = sorted(receive_by_axis.get(can_id, []))
            times = [p[0] for p in published]
            missed_latest = eligible = 0
            for c in samples:
                at = bisect.bisect_right(times, int(c["read_start_ns"])) - 1
                if at >= 0:
                    eligible += 1
                    missed_latest += int(c["rx_sequence"]) < published[at][1]
            per_axis[str(can_id)] = {
                "read_latest_eligible": eligible, "read_latest_misses": missed_latest,
                "fallbacks": sum(int(c["consecutive_failures"]) > 0 for c in samples),
                "read_duration": distribution([int(c["read_end_ns"]) - int(c["read_start_ns"])
                                               for c in samples if int(c["read_start_ns"]) > 0]),
                "read_feedback_age": distribution([int(c["acquired_ns"]) - int(c["rx_published_ns"])
                                                   for c in samples if int(c["rx_published_ns"]) > 0])}
        # A receive after send is a temporal envelope only: replies carry no command sequence.
        next_read_misses = next_read_eligible = 0
        # Sort once per axis; re-sorting per cycle made 1 kHz runs of ~50k cycles effectively hang.
        sorted_times = [[p[0] for p in sorted(receive_by_axis.get(i, []))] for i in can_ids]
        for c in cycles:
            if c.get("send_kind") != "2" or c["send_success"] != "1":
                continue
            if int(c["command_sequence"]) == 0:
                continue
            following = publication.get(int(c["command_sequence"]) + 1)
            if not following:
                continue
            deadline = int(following["read_start_ns"])
            if deadline <= 0:
                continue
            for times in sorted_times:
                next_read_eligible += 1
                index = bisect.bisect_left(times, int(c["send_before_steady_ns"]))
                next_read_misses += index == len(times) or times[index] >= deadline
        summaries.append({"trace": str(complete), "mode": meta["tx_trigger"],
                          "direct_feedback": meta["direct_feedback"], "fault": info["fault"],
                          "period_ns": period, "write_interval": distribution(intervals),
                          "write_interval_excess": distribution([max(0, d-period) for d in intervals]),
                          "publication_to_send": distribution(latencies),
                          "publication_to_send_over_period": sum(d > period for d in latencies),
                          "resends": info.get("resends", 0),
                          "writes_after_resend": info.get("writes_after_resend", 0),
                          "minimum_interval_holds": info.get("minimum_interval_holds", 0),
                          "resend_command_holds": info.get("resend_command_holds", 0),
                          "new_command_holds": info.get("new_command_holds", 0),
                          "bootstrap_command_holds": info.get("bootstrap_command_holds", 0),
                          "max_hold_ns": info.get("max_hold_ns", 0),
                          "command_snapshot_failures": info.get("command_snapshot_failures", 0),
                          "dropped_control_cycles": int(control_meta["dropped_control_cycles"]),
                          "dropped_cycles": info["dropped_cycles"], "dropped_rx": info["dropped_rx"],
                          "next_read_temporal_envelope_eligible": next_read_eligible,
                          "next_read_temporal_envelope_misses": next_read_misses,
                          "axes": per_axis})
    if not summaries:
        raise ValueError("no completed traces with control records")
    return summaries


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze(args.directory), indent=2))
