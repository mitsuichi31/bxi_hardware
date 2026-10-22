// Single-joint HIL trajectories: the same formulas as hil/single_joint/ramp_command.py
// (ramp_setpoint) and sine_command.py (Trajectory, friction_torque), for use inside a
// controller's update(). at() and frictionTorque() do not allocate or throw.
// Plan: docs/bxi/worker-hil/plans/2026-09-29-rt-trajectory-controller-plan.md (workspace)
#ifndef BXI_HARDWARE__HIL_TRAJECTORY_HPP_
#define BXI_HARDWARE__HIL_TRAJECTORY_HPP_

#include <optional>

namespace bxi_hardware
{
namespace hil
{

struct Setpoint
{
  double position = 0.0;
  double velocity = 0.0;
};

// Moves the target from start to target at a constant speed, then stays at target.
// Same as ramp_command.py's ramp_setpoint(): the moving phase is elapsed < ramp_time().
class RampTrajectory
{
public:
  // speed > 0 [rad/s]
  RampTrajectory(double start, double target, double speed);
  Setpoint at(double elapsed) const noexcept;
  // |target - start| / speed
  double rampTime() const noexcept {return ramp_time_;}

private:
  double start_;
  double target_;
  double speed_;
  double ramp_time_;
  double direction_;
};

// center + a(t) sin(2 pi f t) with a(t) raised and lowered over ramp_cycles periods by a cosine
// shape (sine_command.py's Trajectory). Time is clamped to [0, duration()].
class SineTrajectory
{
public:
  // frequency > 0 [Hz]; cycles and ramp_cycles >= 0
  SineTrajectory(
    double center, double amplitude, double frequency, double cycles, double ramp_cycles);
  Setpoint at(double t) const noexcept;
  // Analytic acceleration, using the same clamped time and envelope as at(). RT-safe:
  // no allocation, locks, I/O or exceptions. Invalid numeric inputs return NaN.
  double accelerationAt(double t) const noexcept;
  double duration() const noexcept {return duration_;}
  // An upper bound of |target velocity| over the whole trajectory, including the envelope's
  // derivative while the amplitude ramps up and down:
  //   |v| = |da/dt sin(wt) + a w cos(wt)| <= hypot(da/dt, a w)
  //       <= A hypot(w, pi / (2 ramp)), by Cauchy-Schwarz,
  // or A w without a ramp. Never below the true maximum, however short the ramp (sampling the
  // ramps would miss the peak of one shorter than the sampling step).
  double velocityUpperBound() const noexcept;
  // NON-RT ONLY. An outward-rounded upper bound, NOT an estimate of the maximum.
  // Isolates every possible stationary point analytically (no time sampling).
  // Supported: finite center, A > 0, f > 0, ramp_cycles exactly 1 or 2, and integer
  // flat cycles in [0, 20]. nullopt means unsupported/invalid input or overflow.
  // Requires IEEE-754 double arithmetic without fast-math; no controller calls this yet.
  std::optional<double> accelerationUpperBound() const;

private:
  struct Envelope {double a; double da; double dda;};
  Envelope envelope(double t, bool second_derivative) const noexcept;
  double center_;
  double amplitude_;
  double omega_;
  double ramp_;
  double flat_;
  double duration_;
  double frequency_;
  double cycles_;
  double ramp_cycles_;
};

// Torque added in the direction of the target velocity (sine_command.py's friction_torque,
// Stribeck model). static_torque < 0 means "same as kinetic".
double frictionTorque(
  double target_velocity, double kinetic, double friction_velocity, double static_torque,
  double stribeck_velocity) noexcept;

}  // namespace hil
}  // namespace bxi_hardware

#endif  // BXI_HARDWARE__HIL_TRAJECTORY_HPP_
