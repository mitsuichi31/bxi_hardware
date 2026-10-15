#include "bxi_hardware/hil_trajectory.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace bxi_hardware
{
namespace hil
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

// Non-RT interval arithmetic. Each basic operation is expanded by one representable
// double on both sides. No libm trigonometric root finder or sampled time grid is used.
struct Interval {double lo; double hi;};
double down(double v) {return std::nextafter(v, -std::numeric_limits<double>::infinity());}
double up(double v) {return std::nextafter(v, std::numeric_limits<double>::infinity());}
Interval add(Interval a, Interval b) {return {down(a.lo + b.lo), up(a.hi + b.hi)};}
Interval multiply(Interval a, Interval b)
{
  const double p[] = {a.lo * b.lo, a.lo * b.hi, a.hi * b.lo, a.hi * b.hi};
  return {down(*std::min_element(p, p + 4)), up(*std::max_element(p, p + 4))};
}
double magnitude(Interval a) {return std::max(std::abs(a.lo), std::abs(a.hi));}
Interval polynomial(const std::array<double, 6> & coefficients, Interval x)
{
  Interval y{0.0, 0.0};
  for (auto it = coefficients.rbegin(); it != coefficients.rend(); ++it) {
    y = add(multiply(y, x), {*it, *it});
  }
  return y;
}

// |sin(x) Q(cos(x))|, with cos(x) enclosed by c, x in [0, pi].
double rampMagnitude(const std::array<double, 6> & q, Interval c)
{
  const double min_abs = c.lo <= 0.0 && c.hi >= 0.0 ? 0.0 :
    std::min(std::abs(c.lo), std::abs(c.hi));
  const double min_square = std::max(0.0, down(min_abs * min_abs));
  const double sin_upper = up(std::sqrt(std::max(0.0, up(1.0 - min_square))));
  return up(sin_upper * magnitude(polynomial(q, c)));
}

// Horner interval exclusion encloses ALL roots, including tangencies/multiple roots:
// discard only intervals proven to contain no zero. At depth 36 evaluate the whole
// surviving interval, not an approximate root. Even an unresolved root cannot be lost.
// A work budget makes this bounded work; exhaustion retains a wider, conservative bound.
double stationaryMagnitude(
  const std::array<double, 6> & derivative, const std::array<double, 6> & q,
  Interval c, unsigned depth, unsigned & remaining)
{
  const auto p = polynomial(derivative, c);
  if (p.lo > 0.0 || p.hi < 0.0) {return 0.0;}
  if (depth == 36 || remaining == 0) {return rampMagnitude(q, c);}
  --remaining;
  const double mid = c.lo + (c.hi - c.lo) * 0.5;
  const double left = stationaryMagnitude(derivative, q, {c.lo, mid}, depth + 1, remaining);
  const double right = stationaryMagnitude(derivative, q, {mid, c.hi}, depth + 1, remaining);
  return std::max(left, right);
}
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
  duration_(2.0 * (ramp_cycles / frequency) + cycles / frequency),
  frequency_(frequency), cycles_(cycles), ramp_cycles_(ramp_cycles)
{
}

SineTrajectory::Envelope SineTrajectory::envelope(double t, bool second_derivative) const noexcept
{
  // Preserve the original arithmetic/order for a and da (the 1-axis reference values).
  double a = amplitude_;
  double da = 0.0;
  double dda = 0.0;
  if (ramp_ > 0.0) {
    if (t < ramp_) {
      const double x = t / ramp_;
      a = amplitude_ * 0.5 * (1.0 - std::cos(kPi * x));
      da = amplitude_ * 0.5 * kPi / ramp_ * std::sin(kPi * x);
      if (second_derivative) {
        dda = amplitude_ * 0.5 * (kPi / ramp_) * (kPi / ramp_) * std::cos(kPi * x);
      }
    } else if (t > ramp_ + flat_) {
      const double x = std::min((t - ramp_ - flat_) / ramp_, 1.0);
      a = amplitude_ * 0.5 * (1.0 + std::cos(kPi * x));
      da = -amplitude_ * 0.5 * kPi / ramp_ * std::sin(kPi * x);
      if (second_derivative) {
        dda = -amplitude_ * 0.5 * (kPi / ramp_) * (kPi / ramp_) * std::cos(kPi * x);
      }
    }
  }
  return {a, da, dda};
}

Setpoint SineTrajectory::at(double t) const noexcept
{
  t = std::min(std::max(t, 0.0), duration_);
  const auto e = envelope(t, false);
  const double s = std::sin(omega_ * t);
  const double c = std::cos(omega_ * t);
  return {center_ + e.a * s, e.da * s + e.a * omega_ * c};
}

double SineTrajectory::accelerationAt(double t) const noexcept
{
  // The legacy constructor/at()/velocityUpperBound() retain their input contract.
  // This new API fails closed without throwing on malformed numeric input.
  if (!std::isfinite(t) || !std::isfinite(center_) || !std::isfinite(amplitude_) ||
    amplitude_ <= 0.0 || !std::isfinite(frequency_) || frequency_ <= 0.0 ||
    !std::isfinite(cycles_) || cycles_ < 0.0 || !std::isfinite(ramp_cycles_) ||
    ramp_cycles_ < 0.0 || !std::isfinite(duration_) || !std::isfinite(omega_))
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  t = std::min(std::max(t, 0.0), duration_);
  const auto e = envelope(t, true);
  const double s = std::sin(omega_ * t);
  const double c = std::cos(omega_ * t);
  const double result = e.dda * s + 2.0 * omega_ * e.da * c - omega_ * omega_ * e.a * s;
  return std::isfinite(result) ? result : std::numeric_limits<double>::quiet_NaN();
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

std::optional<double> SineTrajectory::accelerationUpperBound() const
{
#ifdef __FAST_MATH__
  return std::nullopt;  // Outward rounding must not be compiled away.
#else
  static_assert(std::numeric_limits<double>::is_iec559, "IEEE-754 double required");
  if (!std::isfinite(center_) || !std::isfinite(amplitude_) || amplitude_ <= 0.0 ||
    !std::isfinite(frequency_) || frequency_ <= 0.0 || !std::isfinite(duration_) ||
    !std::isfinite(omega_) || !std::isfinite(ramp_) || ramp_ <= 0.0 ||
    !std::isfinite(cycles_) || cycles_ < 0.0 || cycles_ > 20.0 ||
    std::floor(cycles_) != cycles_ || (ramp_cycles_ != 1.0 && ramp_cycles_ != 2.0))
  {
    return std::nullopt;
  }
  // x=pi*t/T, n=2*ramp_cycles. a=A*(pi/T)^2*sin(x)*Q(cos(x))/4.
  // da/dx is proportional to P(c)=-2*n^3*T_n(c)+(n+1)^3*T_(n+1)(c)
  //                                  +(n-1)^3*T_(n-1)(c).
  // Integer flat cycles make ramp-down a time reversal up to sign: same |a| bound.
  const std::array<double, 6> p = ramp_cycles_ == 1.0 ?
    std::array<double, 6>{16, -80, -32, 108, 0, 0} :
    std::array<double, 6>{-128, 544, 1024, -2392, -1024, 2000};
  const std::array<double, 6> q = ramp_cycles_ == 1.0 ?
    std::array<double, 6>{-8, -16, 36, 0, 0, 0} :
    std::array<double, 6>{16, 128, -264, -256, 400, 0};
  unsigned budget = 100000;
  const double ramp_factor = std::max({rampMagnitude(q, {-1.0, -1.0}),
      rampMagnitude(q, {1.0, 1.0}), stationaryMagnitude(p, q, {-1.0, 1.0}, 0, budget)});
  const Interval pi{down(kPi), up(kPi)};
  const Interval frequency{frequency_, frequency_};
  const Interval a{amplitude_, amplitude_};
  const auto k = multiply(multiply(pi, frequency),
      {1.0 / ramp_cycles_, 1.0 / ramp_cycles_});
  const auto omega = multiply(multiply(pi, frequency), {2.0, 2.0});
  const auto ramp_scale = multiply(multiply(a, multiply(k, k)), {0.25, 0.25});
  const double bound = std::max(multiply(a, multiply(omega, omega)).hi,
      up(ramp_scale.hi * ramp_factor));
  // Interval operations above cover the real analytic formula. Additional guard for the
  // separate double at()/accelerationAt() phase/envelope evaluations (cycles <= 20).
  // This is not a claim about arbitrary libm implementations or fast-math builds.
  const double guarded = up(bound + up(1e-10 * std::max(1.0, bound)));
  if (!std::isfinite(guarded)) {return std::nullopt;}
  return guarded;
#endif
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
