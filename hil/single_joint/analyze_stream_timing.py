#!/usr/bin/env python3
"""Summarize opt-in BXI streamed-bus timing CSVs (rx_mode: separate; durations in ms).

Reads <stem>_cycles.csv and <stem>_rx.csv written when BXI_WORKER_TIMING_DIR is set.
"""

import argparse
import csv
import math
from collections import Counter
from pathlib import Path


def read_csv(path):
    """Return the metadata dict from the '#key=value,...' first line and the rows."""
    with path.open(newline="") as stream:
        first = stream.readline().strip()
        if not first.startswith("#period_ns="):
            raise ValueError(f"missing stream timing metadata: {path}")
        metadata = dict(item.split("=", 1) for item in first[1:].split(","))
        return metadata, list(csv.DictReader(stream))


def nearest_rank(sorted_values, fraction):
    return sorted_values[max(0, math.ceil(fraction * len(sorted_values)) - 1)]


def distribution(values_ns):
    """median/p99/p99.9/max in milliseconds, or None without samples."""
    values = sorted(values_ns)
    if not values:
        return None
    return {name: nearest_rank(values, fraction) / 1e6
            for name, fraction in (("median", 0.5), ("p99", 0.99), ("p99.9", 0.999), ("max", 1.0))}


def format_distribution(label, values_ns, over_ns=None, over_label=""):
    stats = distribution(values_ns)
    if stats is None:
        return f"{label}: no samples"
    text = f"{label}: " + " ".join(f"{name}={value:.4f}" for name, value in stats.items()) + " ms"
    if over_ns is not None:
        over = sum(value > over_ns for value in values_ns)
        text += f", {over_label}={over}/{len(values_ns)}"
    return text


def intervals(values):
    return [later - earlier for earlier, later in zip(values, values[1:])]


def summarize(stem, tolerance_ms=0.1):
    """Return the report lines for one stream timing capture."""
    metadata, cycles = read_csv(Path(f"{stem}_cycles.csv"))
    _, replies = read_csv(Path(f"{stem}_rx.csv"))
    period_ns = int(metadata["period_ns"])
    max_age_ns = int(metadata["max_feedback_age_ns"])
    tolerance_ns = int(tolerance_ms * 1e6)
    can_ids = [int(value) for value in metadata["can_ids"].split(";") if value]

    lines = [
        f"period={period_ns / 1e6:.3f} ms, max feedback age={max_age_ns / 1e6:.3f} ms, "
        f"broadcast={metadata['broadcast']}, motors={can_ids}",
        f"send thread: {metadata['tx_thread']}; receive thread: {metadata['rx_thread']}",
        f"cycles={len(cycles)} (dropped {metadata['dropped_cycles']}), "
        f"replies={len(replies)} (dropped {metadata['dropped_rx']})",
    ]
    outcomes = Counter(row["outcome"] for row in cycles)
    lines.append("outcomes: " + ", ".join(f"{name}={count}" for name, count in sorted(outcomes.items())))

    lines.append(format_distribution(
        "wake lateness", [int(row["wake_lateness_ns"]) for row in cycles],
        tolerance_ns, f"over {tolerance_ms:g} ms"))
    sent = [row for row in cycles if int(row["send_lateness_ns"]) >= 0]
    lines.append(format_distribution(
        "send lateness from slot start", [int(row["send_lateness_ns"]) for row in sent],
        tolerance_ns, f"over {tolerance_ms:g} ms"))
    lines.append(format_distribution(
        "hook", [int(row["hook_ns"]) for row in cycles if row["outcome"] != "skipped_late"
                 and row["outcome"] != "skipped_burst"]))
    send_times = [int(row["cycle"]) * period_ns + int(row["send_lateness_ns"]) for row in sent]
    lines.append(format_distribution(
        "send interval", intervals(send_times), period_ns + tolerance_ns,
        f"over period+{tolerance_ms:g} ms"))

    for can_id in can_ids:
        ages = [int(row[f"age_ns_{can_id}"]) for row in cycles if int(row[f"age_ns_{can_id}"]) >= 0]
        lines.append(format_distribution(
            f"motor {can_id} feedback age at cycle start", ages, period_ns, "over one period"))
        over_limit = sum(age > max_age_ns for age in ages)
        # A cycle counts as without a new reply only after the first send (age is then >= 0).
        without_new = sum(1 for row in cycles
                          if int(row[f"age_ns_{can_id}"]) >= 0 and row[f"fresh_{can_id}"] == "0")
        lines.append(f"motor {can_id}: over max age={over_limit}, "
                     f"cycles without a new reply={without_new}/{len(ages)}")

        rows = [row for row in replies if int(row["can_id"]) == can_id]
        lines.append(format_distribution(
            f"motor {can_id} reply interval (userspace)",
            intervals([int(row["received_ns"]) for row in rows]), period_ns + tolerance_ns,
            f"over period+{tolerance_ms:g} ms"))
        stamped = [row for row in rows
                   if int(row["rx_kernel_realtime_ns"]) > 0 and int(row["rx_userspace_realtime_ns"]) > 0]
        # Includes the receive thread's wake-up, the recvmsg() call, and any wait behind earlier
        # frames, so it bounds the wake-up delay from above rather than isolating it.
        lines.append(format_distribution(
            f"motor {can_id} kernel RX to userspace read (upper bound of receive wake-up)",
            [int(row["rx_userspace_realtime_ns"]) - int(row["rx_kernel_realtime_ns"]) for row in stamped]))
        hardware = [int(row["rx_hardware_raw_ns"]) for row in rows if int(row["rx_hardware_raw_ns"]) > 0]
        lines.append(f"motor {can_id}: kernel timestamps={len(stamped)}/{len(rows)}, "
                     f"hardware timestamps={len(hardware)}/{len(rows)}")
        if hardware:
            lines.append(format_distribution(
                f"motor {can_id} hardware RX interval", intervals(hardware),
                period_ns + tolerance_ns, f"over period+{tolerance_ms:g} ms"))
    return lines


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stem", type=Path, help="CSV path without the _cycles.csv / _rx.csv suffix")
    parser.add_argument("--tolerance-ms", type=float, default=0.1,
                        help="allowed lateness before a cycle or interval is counted (default 0.1)")
    args = parser.parse_args()
    for line in summarize(args.stem, args.tolerance_ms):
        print(line)


if __name__ == "__main__":
    main()
