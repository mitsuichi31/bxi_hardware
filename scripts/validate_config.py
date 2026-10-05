#!/usr/bin/env python3
"""Validate BXI YAML structure and cross-field safety constraints."""

import argparse
import math
from pathlib import Path
import sys

import json
import jsonschema
import yaml


def validate(config_path: Path, schema_path: Path, spec_dir: Path) -> list[str]:
    config = yaml.safe_load(config_path.read_text())
    schema = json.loads(schema_path.read_text())
    errors = [error.message for error in jsonschema.Draft202012Validator(schema).iter_errors(config)]
    if errors:
        return errors

    active_buses = set(config["active_can_buses"])
    for bus in config.get("can_frame_format", {}):
        if bus not in active_buses:
            errors.append(f"can_frame_format: unknown or inactive bus {bus!r}")
    used_ids: set[tuple[str, int]] = set()
    for name, joint in config["joints"].items():
        if not joint["enabled"]:
            continue
        required = ("can_bus", "can_id", "motor_dir", "motor_type", "lower", "upper")
        missing = [key for key in required if key not in joint]
        if missing:
            errors.append(f"{name}: missing enabled-joint keys: {', '.join(missing)}")
            continue
        bus = joint["can_bus"]
        can_id = joint["can_id"]
        if bus not in active_buses:
            errors.append(f"{name}: unknown or inactive bus {bus!r}")
        if (bus, can_id) in used_ids:
            errors.append(f"{name}: duplicate CAN ID {can_id} on {bus}")
        used_ids.add((bus, can_id))
        if not all(math.isfinite(joint[key]) for key in ("lower", "upper")) or joint["lower"] >= joint["upper"]:
            errors.append(f"{name}: lower must be finite and less than upper")
        motor_type = joint["motor_type"]
        if motor_type == "TBD" or not (spec_dir / f"{motor_type}.yaml").is_file():
            errors.append(f"{name}: missing motor spec {motor_type!r}")
    errors.extend(validate_transport(config, active_buses, used_ids))
    return errors


# Valid CAN FD payload lengths for a broadcast of N 8-byte blocks.
BROADCAST_MOTOR_COUNTS = {1, 2, 3, 4, 6, 8}


def validate_transport(config: dict, active_buses: set[str], used_ids: set[tuple[str, int]]) -> list[str]:
    """Mirror BxiSystemInterface::parseConfig() for can_transport."""
    errors: list[str] = []
    period_ms = 1000.0 / config["defaults"]["can_hz"]
    formats = config.get("can_frame_format", {})
    for bus, transport in config.get("can_transport", {}).items():
        if bus not in active_buses:
            errors.append(f"can_transport: unknown or inactive bus {bus!r}")
            continue
        separate = transport.get("rx_mode", "sequential") == "separate"
        broadcast = transport.get("tx_frame", "unicast") == "broadcast"
        isolated = "hil_isolated_id1_broadcast_blocks" in transport
        max_age = transport.get("stream_feedback_max_age_ms")
        command_max_age = transport.get("stream_command_max_age_ms")
        write_trigger = transport.get("tx_trigger", "periodic") == "write"
        direct = transport.get("direct_feedback", False)
        grace = transport.get("stream_write_grace_ms")
        thread_setting = any(key in transport for key in (
            "send_thread_priority", "receive_thread_priority", "thread_cpus"))
        if not separate:
            if (broadcast or max_age is not None or command_max_age is not None or isolated
                    or thread_setting or write_trigger or direct or grace is not None
                    or "stream_first_write_timeout_ms" in transport):
                errors.append(f"can_transport.{bus}: broadcast, stream_feedback_max_age_ms, "
                              "stream_command_max_age_ms, hil_isolated_id1_broadcast_blocks and "
                              "thread settings require rx_mode: separate")
            continue
        if max_age is None or not math.isfinite(max_age) or max_age < 2 * period_ms:
            errors.append(f"can_transport.{bus}.stream_feedback_max_age_ms is required and must be "
                          f"at least two periods ({2 * period_ms:.3f} ms)")
        if command_max_age is not None and (
                not math.isfinite(command_max_age) or command_max_age < 2 * period_ms):
            errors.append(f"can_transport.{bus}.stream_command_max_age_ms must be at least two "
                          f"periods ({2 * period_ms:.3f} ms)")
        if write_trigger:
            first_write_timeout = transport.get("stream_first_write_timeout_ms", 20)
            if not math.isfinite(first_write_timeout) or not 0 < first_write_timeout <= 1000:
                errors.append(f"can_transport.{bus}: first-write timeout must be finite in (0,1000] ms")
            effective_command_age = command_max_age if command_max_age is not None else max(5, 2 * period_ms)
            if (grace is None or not math.isfinite(grace) or grace <= 0
                    or max_age is None or period_ms + grace >= max_age
                    or period_ms + grace >= effective_command_age):
                errors.append(f"can_transport.{bus}: write trigger needs explicit positive grace "
                              "and freshness margin")
        elif grace is not None or "stream_first_write_timeout_ms" in transport:
            errors.append(f"can_transport.{bus}: write grace / first-write timeout require tx_trigger: write")
        ids = sorted(can_id for joint_bus, can_id in used_ids if joint_bus == bus)
        bus_format = formats.get(bus, "classic")
        if isolated:
            if not broadcast or bus != "can0" or ids != [1] or bus_format != "fd":
                errors.append(f"can_transport.{bus}.hil_isolated_id1_broadcast_blocks needs can0, "
                              "only motor ID 1, can_frame_format: fd and tx_frame: broadcast")
        elif broadcast:
            if bus_format == "classic":
                errors.append(f"can_transport.{bus}: broadcast needs CAN FD")
            if ids != list(range(1, len(ids) + 1)) or len(ids) not in BROADCAST_MOTOR_COUNTS:
                errors.append(f"can_transport.{bus}: broadcast needs enabled motor IDs 1..N "
                              f"with N in {sorted(BROADCAST_MOTOR_COUNTS)}")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("config", type=Path)
    parser.add_argument("--schema", type=Path, default=Path("config/bxi_hardware.schema.json"))
    parser.add_argument("--spec-dir", type=Path, default=Path("config/specs"))
    args = parser.parse_args()
    errors = validate(args.config, args.schema, args.spec_dir)
    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        return 1
    print(f"validated BXI config: {args.config}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
