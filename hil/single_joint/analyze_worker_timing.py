#!/usr/bin/env python3
"""Summarize opt-in BXI CAN worker timing CSVs (durations in milliseconds)."""

import argparse
import csv
import math
from pathlib import Path


def load_cycles(path):
    with path.open(newline="") as stream:
        metadata = stream.readline().strip()
        if not metadata.startswith("#period_ns="):
            raise ValueError(f"missing timing metadata: {path}")
        values = dict(item.split("=", 1) for item in metadata[1:].split(","))
        return {key: int(value) for key, value in values.items()}, list(csv.DictReader(stream))


def load_transactions(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def nearest_rank(sorted_values, fraction):
    return sorted_values[max(0, math.ceil(fraction * len(sorted_values)) - 1)] / 1_000_000


def show_metric(label, rows, key):
    values = sorted(int(row[key]) for row in rows)
    if not values:
        print(f"{label}: no samples")
        return
    print(
        f"{label}: median={nearest_rank(values, 0.5):.4f} "
        f"p99={nearest_rank(values, 0.99):.4f} "
        f"p99.9={nearest_rank(values, 0.999):.4f} "
        f"max={nearest_rank(values, 1.0):.4f} ms"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stem", type=Path, help="CSV path without _cycles.csv suffix")
    args = parser.parse_args()
    stem = str(args.stem)
    metadata, cycles = load_cycles(Path(stem + "_cycles.csv"))
    transactions = load_transactions(Path(stem + "_transactions.csv"))
    period_ns = metadata["period_ns"]

    print(f"period={period_ns / 1_000_000:.3f} ms, cycles={len(cycles)}, "
          f"transactions={len(transactions)}")
    print(f"dropped cycles={metadata['dropped_cycles']}, "
          f"dropped transactions={metadata['dropped_transactions']}, "
          f"invalid responses={sum(int(row['invalid_responses']) for row in cycles)}")
    print(f"work over period={sum(int(row['work_ns']) > period_ns for row in cycles)}, "
          f"interval over period={sum(int(row['interval_ns']) > period_ns for row in cycles[1:])}")
    for label, key in (
        ("cycle interval", "interval_ns"),
        ("start lateness", "start_lateness_ns"),
        ("allocation", "allocation_ns"),
        ("target lock wait", "target_lock_wait_ns"),
        ("target lock hold", "target_lock_hold_ns"),
        ("CAN section", "can_ns"),
        ("feedback lock wait", "feedback_lock_wait_ns"),
        ("feedback lock hold", "feedback_lock_hold_ns"),
        ("work", "work_ns"),
    ):
        # Cycle 0 has no preceding cycle, so both fields contain placeholder zeroes.
        samples = cycles[1:] if key in ("interval_ns", "start_lateness_ns") else cycles
        show_metric(label, samples, key)
    show_metric("CAN transaction", transactions, "duration_ns")


if __name__ == "__main__":
    main()
