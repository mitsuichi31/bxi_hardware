"""The committed reference table still matches the Python scripts, and their velocity checks."""
import math
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "test"))
sys.path.insert(0, str(ROOT / "hil" / "single_joint"))

from generate_trajectory_reference import table  # noqa: E402
from ramp_command import ramp_setpoint  # noqa: E402
from sine_command import Trajectory  # noqa: E402


def test_committed_table_matches_the_python_scripts():
    committed = (ROOT / "test" / "data" / "hil_trajectory_reference.csv").read_text()
    assert committed == table(), "regenerate with test/generate_trajectory_reference.py"


def test_velocity_bound_includes_the_envelope_derivative():
    short_ramp = Trajectory(0.0, 0.45, 0.6, 1.0, 0.01)
    assert 0.45 * short_ramp.omega < 2.0
    assert short_ramp.velocity_bound() > 2.0
    # A ramp shorter than any sampling step: about 78,540 rad/s in the middle of the ramp-down.
    tiny = Trajectory(0.0, 0.1, 0.5, 0.25, 0.000001)
    peak = abs(tiny.at(tiny.ramp + tiny.flat + tiny.ramp / 2)[1])
    assert peak > 78000.0
    assert tiny.velocity_bound() >= peak
    stage4 = Trajectory(0.0452, 0.1, 0.5, 5.0, 1.0)
    assert stage4.velocity_bound() == pytest.approx(0.1 * math.hypot(math.pi, math.pi / 4), abs=1e-12)
    # Without a ramp the amplitude never changes.
    assert Trajectory(0.0, 0.1, 0.5, 2.0, 0.0).velocity_bound() == pytest.approx(0.1 * math.pi)


@pytest.mark.parametrize("ramp_cycles", ["0.000001", "0.01"])
def test_sine_command_rejects_a_ramp_too_fast_for_the_velocity_limit(ramp_cycles):
    """Rejected before ROS starts, so no launch is needed."""
    frequency = "0.5" if ramp_cycles == "0.000001" else "0.6"
    amplitude = "0.1" if ramp_cycles == "0.000001" else "0.45"
    cycles = "0.25" if ramp_cycles == "0.000001" else "1"
    result = subprocess.run(
        [sys.executable, str(ROOT / "hil" / "single_joint" / "sine_command.py"),
         "--amplitude", amplitude, "--frequency", frequency, "--cycles", cycles,
         "--ramp-cycles", ramp_cycles],
        capture_output=True, text=True, timeout=30)
    assert result.returncode == 1
    assert "目標速度の上界" in result.stderr


def test_ramp_setpoint_moves_at_speed_then_holds_the_target():
    assert ramp_setpoint(0.0277, 0.2, 0.3, 0.0) == (0.0277, 0.3)
    ramp_time = (0.2 - 0.0277) / 0.3
    assert ramp_setpoint(0.0277, 0.2, 0.3, ramp_time) == (0.2, 0.0)
    assert ramp_setpoint(0.1646, 0.0, 0.3, 0.1)[1] == -0.3
    assert ramp_setpoint(0.1, 0.1, 0.3, 0.0) == (0.1, 0.0)


def run_script(name, *arguments):
    return subprocess.run(
        [sys.executable, str(ROOT / "hil" / "single_joint" / name), *arguments],
        capture_output=True, text=True, timeout=30)


@pytest.mark.parametrize("arguments, named", [
    (["--max-velocity", "nan"], "--max-velocity"),   # would make the velocity check always pass
    (["--amplitude", "inf"], "--amplitude"),
    (["--frequency", "nan"], "--frequency"),
    (["--ramp-cycles", "nan"], "--ramp-cycles"),
    (["--hold", "nan"], "--hold"),
    (["--max-torque", "nan"], "--max-torque"),
    (["--friction-ff", "nan"], "--friction-ff"),
])
def test_sine_command_rejects_non_finite_numbers(arguments, named):
    result = run_script("sine_command.py", *arguments)
    assert result.returncode == 1
    assert "有限でない値" in result.stderr and named in result.stderr


@pytest.mark.parametrize("arguments, named", [
    (["0.1", "--max-torque", "nan"], "--max-torque"),  # would never abort on the torque check
    (["nan"], "--target"),
    (["0.1", "--speed", "inf"], "--speed"),
    (["0.1", "--hold", "nan"], "--hold"),
])
def test_ramp_command_rejects_non_finite_numbers(arguments, named):
    result = run_script("ramp_command.py", *arguments)
    assert result.returncode == 1
    assert "有限でない値" in result.stderr and named in result.stderr
