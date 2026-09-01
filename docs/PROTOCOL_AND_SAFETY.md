# BXI MIT protocol and safety decisions

## Confirmed protocol

Source: BXI CAN Communication Guide summarized in the parent workspace's `BXI_Actuators_Motor_Info.md`.

- Classic CAN uses an 8-byte standard frame addressed to the motor `can_id`.
- Command fields are big-endian bit streams: position 16 bit; velocity, Kp, Kd and torque 12 bit each.
- Feedback uses byte 0 motor ID, then position 16 bit, velocity/torque 12 bit, MOS and motor temperatures.
- Default response ID is `can_id | 0x010`.
- Enable, disable and save-zero are seven `0xFF` bytes followed by `0xFC`, `0xFD` and `0xFE` respectively.

## Input policy

- The pure codec rejects NaN, infinity, inconsistent limits and out-of-range physical values.
- The actuator boundary rejects non-finite commands and saturates finite runtime commands to both joint and motor limits before calling the strict codec.
- Configuration validation rejects duplicate CAN IDs on a bus, inactive/unknown buses, invalid joint limits, invalid direction, missing motor type and missing spec.

## Lifecycle and safety

The lifecycle is `disabled -> enabled -> disabled`. Consecutive response timeouts latch `safe-stopped`; once latched, new command frames are not transmitted. Recovery requires lifecycle teardown and reconstruction rather than an implicit command retry. MOS temperature over 80 degrees also stops the SystemInterface and sends disable.

## Deliberately unresolved

The acquired guide does not define a separate error frame or a run-mode selection frame. No guessed CAN frame is emitted for either. `setRunMode()` only confirms that the documented enable transition succeeded. Fault coverage in Stage 1 is therefore response timeout, malformed/mismatched feedback and reported temperature. Vendor confirmation and HIL error injection remain prerequisites for the physical safety gate.
