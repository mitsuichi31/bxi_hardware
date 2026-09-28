#!/usr/bin/env python3
"""Minimal FD reply emulator for an isolated bxi_probe --stream vcan smoke test.

Run only on a vcan interface. This does not emulate motor state or timing.
"""

import argparse
import socket
import struct
import time


REPLY = bytes.fromhex("0180487ff7ff5351")
ZERO_COMMAND = bytes.fromhex("7fff7ff0000007ff")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--iface", default="vcan0")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--broadcast-replies", type=int, default=-1)
    parser.add_argument("--broadcast-reply-ids", default="1")
    parser.add_argument("--expect-broadcast-bytes", type=int, choices=(8, 24), default=8)
    args = parser.parse_args()
    if not args.iface.startswith("vcan"):
        parser.error("only vcan interfaces are allowed")
    try:
        reply_ids = tuple(int(value) for value in args.broadcast_reply_ids.split(","))
    except ValueError:
        parser.error("broadcast reply IDs must be integers")
    if not reply_ids or len(reply_ids) != len(set(reply_ids)) or any(
        motor_id not in (1, 2, 3) for motor_id in reply_ids
    ):
        parser.error("broadcast reply IDs must be unique and within 1-3")

    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
        bus.setsockopt(socket.SOL_CAN_RAW, 5, 1)  # CAN_RAW_FD_FRAMES
        bus.bind((args.iface,))
        bus.settimeout(0.2)
        end = time.monotonic() + args.seconds
        broadcast_replies = 0
        broadcast_frames = 0
        invalid_broadcast = 0
        broadcast_replies_by_motor = {1: 0, 2: 0, 3: 0}
        disable_replies = 0
        while time.monotonic() < end:
            try:
                raw = bus.recv(72)
            except socket.timeout:
                continue
            if len(raw) != 72:
                continue
            can_id, length, flags = struct.unpack_from("=IBB", raw)
            if can_id not in (0x001, 0x7FF):
                continue
            data = raw[8:8 + length]
            if can_id == 0x001 and data != b"\xff" * 7 + b"\xfd":
                continue
            if can_id == 0x7FF:
                if length != args.expect_broadcast_bytes or data != ZERO_COMMAND * (length // 8):
                    invalid_broadcast += 1
                    continue
                broadcast_frames += 1
                if 0 <= args.broadcast_replies <= broadcast_replies:
                    continue
                broadcast_replies += 1
            motor_ids = reply_ids if can_id == 0x7FF else (1,)
            for motor_id in motor_ids:
                reply = bytes((motor_id,)) + REPLY[1:]
                response = struct.pack(
                    "=IBB2x64s", 0x010 | motor_id, 8, flags & 1, reply.ljust(64, b"\0")
                )
                bus.send(response)
                if can_id == 0x7FF:
                    broadcast_replies_by_motor[motor_id] += 1
                else:
                    disable_replies += 1
        print(f"broadcast_frames={broadcast_frames} invalid_broadcast={invalid_broadcast}", flush=True)
        print(
            "broadcast_replies="
            + ",".join(
                f"{motor_id}:{count}" for motor_id, count in broadcast_replies_by_motor.items()
            )
            + f" disable_replies={disable_replies}",
            flush=True,
        )
        if broadcast_frames == 0 or invalid_broadcast:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
