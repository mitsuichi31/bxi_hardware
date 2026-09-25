from pathlib import Path
import math
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / "hil" / "single_joint"))

from sine_command import Trajectory, friction_torque  # noqa: E402


@pytest.fixture
def traj():
    return Trajectory(center=0.05, amplitude=0.2, frequency=0.5, cycles=3, ramp_cycles=1)


def test_starts_and_ends_at_center_at_rest(traj):
    for t in (0.0, traj.duration, traj.duration + 1.0):
        p, v = traj.at(t)
        assert p == pytest.approx(0.05, abs=1e-12)
        assert v == pytest.approx(0.0, abs=1e-12)


def test_velocity_is_the_derivative_of_position(traj):
    h = 1e-6
    t = 0.0
    while t < traj.duration:
        p1, _ = traj.at(t - h) if t > h else traj.at(t)
        p2, _ = traj.at(t + h)
        numeric = (p2 - p1) / ((t + h) - (t - h if t > h else t))
        assert traj.at(t)[1] == pytest.approx(numeric, abs=1e-4)
        t += 0.01


def test_position_and_velocity_are_continuous(traj):
    previous = traj.at(0.0)
    t = 0.0
    while t < traj.duration:
        t += 0.001
        current = traj.at(t)
        assert abs(current[0] - previous[0]) < 0.002
        assert abs(current[1] - previous[1]) < 0.01
        previous = current


def test_full_amplitude_between_the_ramps(traj):
    peak = max(traj.at(traj.ramp + k * 0.001)[0] for k in range(int(traj.flat / 0.001)))
    assert peak == pytest.approx(0.25, abs=1e-4)
    assert max(abs(traj.at(k * 0.001)[1]) for k in range(int(traj.duration / 0.001))) <= \
        0.2 * 2 * math.pi * 0.5 + 1e-9


def test_stribeck_is_static_at_low_speed_and_kinetic_at_high_speed():
    # 低速（tanh はほぼ 1）では静止摩擦に近く、高速では動摩擦に近づく
    low = friction_torque(0.02, 0.6, 0.005, static=1.0, stribeck_velocity=0.2)
    high = friction_torque(2.0, 0.6, 0.005, static=1.0, stribeck_velocity=0.2)
    assert 0.95 < low < 1.0
    assert high == pytest.approx(0.6, abs=1e-4)
    assert friction_torque(-2.0, 0.6, 0.005, static=1.0) == pytest.approx(-0.6, abs=1e-4)
    # 大きさは速度とともに単調に減る（tanh が飽和した範囲）
    values = [friction_torque(v, 0.6, 0.005, static=1.0) for v in (0.05, 0.1, 0.2, 0.4)]
    assert values == sorted(values, reverse=True)


def test_stribeck_without_static_is_the_constant_model():
    for v in (-0.3, -0.01, 0.0, 0.02, 0.5):
        assert friction_torque(v, 0.8, 0.05, static=None) == friction_torque(v, 0.8, 0.05, static=0.8)
        assert friction_torque(v, 0.8, 0.05, static=None) == pytest.approx(0.8 * math.tanh(v / 0.05))


def test_friction_torque_follows_the_target_velocity_direction():
    assert friction_torque(0.3, 0.8, 0.05) == pytest.approx(0.8, abs=1e-4)
    assert friction_torque(-0.3, 0.8, 0.05) == pytest.approx(-0.8, abs=1e-4)
    assert friction_torque(0.0, 0.8, 0.05) == 0.0
    assert friction_torque(0.3, 0.0, 0.05) == 0.0
    # 折り返し点付近では小さく、なめらかに向きを変える
    assert abs(friction_torque(0.005, 0.8, 0.05)) < 0.1
