#!/usr/bin/env python3
"""Reference values of the single-joint HIL trajectories, computed by the Python scripts.

    python3 test/generate_trajectory_reference.py > test/data/hil_trajectory_reference.csv

test_hil_trajectory.cpp checks the C++ port (bxi_hardware::hil) against this table to 1e-12, and
test_trajectory_reference.py checks that the table still matches the Python scripts. Regenerate
the table only together with an intended change of the Python formulas.

Columns: kind,p0,p1,p2,p3,p4,t,out0,out1
  sine     p0..p4 = center, amplitude, frequency, cycles, ramp_cycles; out = position, velocity
  vbound   p0..p4 = as sine; out0 = Trajectory.velocity_bound()
  ramp     p0..p2 = start, target, speed; out = position, velocity
  friction p0..p4 = target velocity, kinetic, friction velocity, static (-1: same as kinetic),
           Stribeck velocity; out0 = torque
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "hil" / "single_joint"))

from ramp_command import ramp_setpoint  # noqa: E402
from sine_command import Trajectory, friction_torque  # noqa: E402

SINES = [
    (0.05, 0.2, 0.5, 3.0, 1.0),
    (0.0452, 0.1, 0.5, 5.0, 1.0),     # HIL stage 4, first condition
    (-0.1, 0.3, 0.2, 5.0, 1.0),
    (0.0, 0.45, 0.6, 1.0, 0.01),      # short ramp: the envelope derivative dominates
    (0.0, 0.1, 2.0, 0.0, 0.5),        # no constant-amplitude part
    (0.0, 0.1, 0.5, 2.0, 0.0),        # no ramp
]
RAMPS = [
    (0.0277, 0.2, 0.3),               # HIL stage 3, operation B
    (0.1646, 0.0, 0.3),               # HIL stage 3, operation C
    (0.1, 0.1, 0.3),                  # no distance
    (-0.2, 0.3, 2.0),
]
FRICTIONS = [
    (0.6, 0.05, None, 0.2),
    (0.6, 0.02, 1.0, 0.2),
    (0.0, 0.05, None, 0.2),
    (0.9, 0.05, None, 0.2),
    (0.8, 0.02, 1.0, 0.1),
]
VELOCITIES = [-1.0, -0.3, -0.05, -0.01, 0.0, 0.01, 0.05, 0.3, 1.0]


def times(boundaries, end):
    """A grid over [0, end] plus each boundary and its +-1e-9 s neighbours, and outside values."""
    values = {-0.1, 0.0, end, end + 0.5}
    values.update(end * k / 97.0 for k in range(98))
    for b in boundaries:
        values.update((b - 1e-9, b, b + 1e-9))
    return sorted(values)


def rows():
    for params in SINES:
        traj = Trajectory(*params)
        for t in times((traj.ramp, traj.ramp + traj.flat, traj.duration), traj.duration):
            p, v = traj.at(t)
            yield ("sine", *params, t, p, v)
        yield ("vbound", *params, 0.0, traj.velocity_bound(), 0.0)
    for start, target, speed in RAMPS:
        ramp_time = abs(target - start) / speed
        for t in times((ramp_time,), ramp_time + 0.5):
            if t < 0.0:
                continue  # the controller never evaluates before the start
            p, v = ramp_setpoint(start, target, speed, t)
            yield ("ramp", start, target, speed, 0.0, 0.0, t, p, v)
    for kinetic, friction_velocity, static, stribeck in FRICTIONS:
        for v in VELOCITIES:
            torque = friction_torque(v, kinetic, friction_velocity, static, stribeck)
            yield ("friction", v, kinetic, friction_velocity, -1.0 if static is None else static,
                   stribeck, 0.0, torque, 0.0)


def table():
    lines = ["kind,p0,p1,p2,p3,p4,t,out0,out1"]
    for row in rows():
        lines.append(",".join([row[0]] + [repr(float(x)) for x in row[1:]]))
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    sys.stdout.write(table())
