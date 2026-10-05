#!/usr/bin/env python3
"""Generate an explicit opt-in HIL config with a measured grace, preserving the baseline."""
import argparse
import math
from pathlib import Path
import yaml
from validate_config import validate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--grace-ms", type=float)
    parser.add_argument("--direct-only", action="store_true")
    args = parser.parse_args()
    if not args.direct_only and args.grace_ms is None:
        parser.error("write trigger requires an explicit --grace-ms")
    if args.grace_ms is not None and (not math.isfinite(args.grace_ms) or args.grace_ms <= 0):
        parser.error("grace must be finite and positive; measure it before using a physical bus")
    config = yaml.safe_load(args.baseline.read_text())
    for transport in config.get("can_transport", {}).values():
        if transport.get("rx_mode") != "separate":
            parser.error("baseline must use separate RX")
        transport["direct_feedback"] = True
        if args.direct_only:
            transport["tx_trigger"] = "periodic"
            transport.pop("stream_write_grace_ms", None)
            transport.pop("stream_first_write_timeout_ms", None)
        else:
            transport["tx_trigger"] = "write"
            transport["stream_write_grace_ms"] = args.grace_ms
            transport["stream_first_write_timeout_ms"] = 20.0
    root = Path(__file__).resolve().parents[1]
    if not (root / "config/bxi_hardware.schema.json").exists():
        root = Path(__file__).resolve().parents[2] / "share/bxi_hardware"
    # Never replace an existing config, including the baseline.
    with args.output.open("x") as out:
        out.write("# Generated opt-in config; grace must be verified on independent evaluation data.\n")
        yaml.safe_dump(config, out, sort_keys=False)
    errors = validate(args.output, root / "config/bxi_hardware.schema.json", root / "config/specs")
    if errors:
        args.output.unlink()
        parser.error("; ".join(errors))


if __name__ == "__main__":
    main()
