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
