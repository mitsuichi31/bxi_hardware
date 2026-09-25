# BXI MIT protocol and safety decisions

## Confirmed protocol

Source: BXI CAN Communication Guide summarized in the parent workspace's `BXI_Actuators_Motor_Info.md`.

- Classic CAN uses an 8-byte standard frame addressed to the motor `can_id`.
- Command fields are big-endian bit streams: position 16 bit; velocity, Kp, Kd and torque 12 bit each.
- Feedback uses byte 0 motor ID, then position 16 bit, velocity/torque 12 bit, MOS and motor temperatures.
- Default response ID is `can_id | 0x010`.
- Enable, disable and save-zero are seven `0xFF` bytes followed by `0xFC`, `0xFD` and `0xFE` respectively.

## Confirmed on hardware (2026-09-25)

One BXI7010-19 (FW code 0.2.2 / config 0.2, `can_id` 1, `master_id` 0x11) on PCAN-M.2 `can0`
via SocketCAN, without bxi_usb2can. Details: `minibexkai_docs/research/20260925_bxi_motor_pcan_socketcan_verification.md`.

- Reply ID is `0x010 + can_id` (0x011) and byte 0 of the reply is `can_id` (0x01), not `master_id`.
- One reply per MIT command (148/150 at 50 Hz classic, 1000/1000 with FD+BRS).
- The motor replies in the format it received: classic, CAN FD, or CAN FD with BRS.
  1 Mbps / 5 Mbps FD+BRS ran 1000 frames with zero bus errors.
- MIT replies are also returned in the MENU state, so a reply alone does not prove that the
  motor is in Motor Mode.
- Temperature bytes 6 and 7 are raw values: degC = raw * 180 / 255 - 30 (range -30 to 150).
  Reply `55 02` matched the motor's own readout NTC1 30.5 degC / NTC2 FAIL (-27.8 degC).
  An earlier revision read the raw byte as degC, which reported 85 degC and would have tripped
  the 80 degC over-temperature stop on the first reply.
- The MIT position is the output-shaft angle in rad relative to the user zero point.
- The motor also exposes a text console: input on `0x7E0 + can_id`, output on `0x7F0 + can_id`
  (ASCII, 8 bytes per frame). The driver does not use it; `bxi_probe` reads the `STATE` lines.
- The motor's `can_timeout` register is 0: it keeps applying the last command if traffic stops.

Measured with `tools/bxi_probe` (2026-09-25, classic and FD+BRS, 500 zero-gain commands each at
200 Hz, user-space send-to-receive time in the Dev Container):

| | enable (FC) | Motor Mode after FC | disable (FD) | reply latency min / median / 99 % / max [us] |
|---|---|---|---|---|
| classic | reply (1433 us) | `STATE : 02 MOTOR` | reply, `STATE : 01 MENU` | MENU 1206 / 1284 / 1416 / 1423, enabled 1235 / 1292 / 1424 / 1438 |
| FD+BRS | reply (1299 us) | `STATE : 02 MOTOR` | reply, `STATE : 01 MENU` | MENU 1133 / 1235 / 1347 / 1424, enabled 1153 / 1232 / 1360 / 1593 |

- Enable and disable special frames each produce one MIT reply and switch between MENU and
  Motor Mode, which is what `BxiActuatorDriver::enable()` / `disable()` assume.
- No timeouts in 2000 commands. The worst reply (1.6 ms) is inside the default
  `control_timeout_ms` of 3 ms.
- The reply takes about 1.2-1.3 ms although a classic frame lasts about 0.13 ms at 1 Mbps, and
  FD+BRS only saves about 0.05 ms. The time is dominated by the motor's processing, not the bus.
  Because the worker talks to the motors on one bus one after another, a bus can serve at most
  about 3 motors at 200 Hz (3 x 1.3 ms < 5 ms); 1 kHz needs a different scheme (C-08).

## Temperature sensor faults

A reading below `defaults.temperature_sensor_fault_below_c` (default -20 degC) is treated as a
disconnected or failed NTC. That sensor is excluded from the reported temperature and a warning
is logged once; the remaining sensor keeps driving the 80 degC over-temperature stop. If both
sensors are faulted the reported temperature is NaN and no over-temperature stop can trigger.
This follows the project decision to keep operating the HIL motor whose NTC2 reads about -29 degC
while BXI is investigating it.

## CAN frame format

`can_frame_format` selects `classic`, `fd` or `fd_brs` per bus (default `classic`). Every frame
transmitted on that bus uses the selected format; FD requires the interface to be configured with
`fd on` (MTU 72) or the bus refuses to open. Received classic and FD frames are both accepted.

## Input policy

- The pure codec rejects NaN, infinity, inconsistent limits and out-of-range physical values.
- The actuator boundary rejects non-finite commands and saturates finite runtime commands to both joint and motor limits before calling the strict codec.
- Configuration validation rejects duplicate CAN IDs on a bus, inactive/unknown buses, invalid joint limits, invalid direction, missing motor type and missing spec.

## Lifecycle and safety

The lifecycle is `disabled -> enabled -> disabled`. Consecutive response timeouts latch `safe-stopped`; once latched, new command frames are not transmitted. Recovery requires lifecycle teardown and reconstruction rather than an implicit command retry. MOS temperature over 80 degrees also stops the SystemInterface and sends disable.

## Deliberately unresolved

The acquired guide does not define a separate error frame or a run-mode selection frame. No guessed CAN frame is emitted for either. `setRunMode()` only confirms that the documented enable transition succeeded. Fault coverage in Stage 1 is therefore response timeout, malformed/mismatched feedback and reported temperature. Vendor confirmation and HIL error injection remain prerequisites for the physical safety gate.
