#include "bxi_hardware/hil_trajectory.hpp"

#include <algorithm>
#include <cmath>

namespace bxi_hardware
{
namespace hil
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

RampTrajectory::RampTrajectory(double start, double target, double speed)
: start_(start), target_(target), speed_(speed),
  ramp_time_(std::abs(target - start) / speed),
  direction_(std::copysign(1.0, target - start))
{
}

Setpoint RampTrajectory::at(double elapsed) const noexcept
{
  if (elapsed < ramp_time_) {
    return {start_ + direction_ * speed_ * elapsed, direction_ * speed_};
  }
  return {target_, 0.0};
}

SineTrajectory::SineTrajectory(
  double center, double amplitude, double frequency, double cycles, double ramp_cycles)
: center_(center), amplitude_(amplitude), omega_(2.0 * kPi * frequency),
  ramp_(ramp_cycles / frequency), flat_(cycles / frequency),
  duration_(2.0 * (ramp_cycles / frequency) + cycles / frequency)
{
}

Setpoint SineTrajectory::at(double t) const noexcept
{
  t = std::min(std::max(t, 0.0), duration_);
  // Envelope a(t) and its derivative, as sine_command.py's Trajectory.envelope().
  double a = amplitude_;
  double da = 0.0;
  if (ramp_ > 0.0) {
    if (t < ramp_) {
      const double x = t / ramp_;
      a = amplitude_ * 0.5 * (1.0 - std::cos(kPi * x));
      da = amplitude_ * 0.5 * kPi / ramp_ * std::sin(kPi * x);
    } else if (t > ramp_ + flat_) {
      const double x = std::min((t - ramp_ - flat_) / ramp_, 1.0);
      a = amplitude_ * 0.5 * (1.0 + std::cos(kPi * x));
      da = -amplitude_ * 0.5 * kPi / ramp_ * std::sin(kPi * x);
    }
  }
  const double s = std::sin(omega_ * t);
  const double c = std::cos(omega_ * t);
  return {center_ + a * s, da * s + a * omega_ * c};
}

double SineTrajectory::velocityUpperBound() const noexcept
{
  // Same as sine_command.py's Trajectory.velocity_bound().
  if (ramp_ <= 0.0) {
    return amplitude_ * omega_;  // the amplitude never changes
  }
  // Cauchy-Schwarz: |da*sin + a*omega*cos| <= hypot(da, a*omega).
  // Bound each component over the entire envelope, including arbitrarily short ramps.
  return amplitude_ * std::hypot(omega_, 0.5 * kPi / ramp_);
}

double frictionTorque(
  double target_velocity, double kinetic, double friction_velocity, double static_torque,
  double stribeck_velocity) noexcept
{
  const double stat = static_torque < 0.0 ? kinetic : static_torque;
  if (kinetic == 0.0 && stat == 0.0) {
    return 0.0;
  }
  const double magnitude =
    kinetic + (stat - kinetic) * std::exp(-std::abs(target_velocity) / stribeck_velocity);
  return magnitude * std::tanh(target_velocity / friction_velocity);
}

}  // namespace hil
}  // namespace bxi_hardware
