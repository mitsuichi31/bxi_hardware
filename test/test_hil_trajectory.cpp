// The C++ trajectories (bxi_hardware::hil) against the table computed by the Python scripts
// (test/generate_trajectory_reference.py -> test/data/hil_trajectory_reference.csv).
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "bxi_hardware/hil_trajectory.hpp"

namespace bxi_hardware
{
namespace hil
{
namespace
{

constexpr double kTolerance = 1e-12;

struct Row
{
  std::string kind;
  double p[5];
  double t;
  double out[2];
};

std::vector<Row> readTable()
{
  std::ifstream file(std::string(BXI_TEST_SOURCE_DIR) + "/test/data/hil_trajectory_reference.csv");
  std::vector<Row> rows;
  std::string line;
  std::getline(file, line);  // header
  while (std::getline(file, line)) {
    std::stringstream stream(line);
    Row row;
    std::string field;
    std::getline(stream, row.kind, ',');
    std::vector<double> values;
    while (std::getline(stream, field, ',')) {
      values.push_back(std::stod(field));
    }
    if (values.size() != 8U) {
      ADD_FAILURE() << "bad row: " << line;
      continue;
    }
    for (size_t i = 0; i < 5; ++i) {
      row.p[i] = values[i];
    }
    row.t = values[5];
    row.out[0] = values[6];
    row.out[1] = values[7];
    rows.push_back(row);
  }
  return rows;
}

TEST(HilTrajectory, MatchesThePythonReferenceTable)
{
  const auto rows = readTable();
  ASSERT_GE(rows.size(), 1000U);
  size_t counts[4] = {0, 0, 0, 0};
  for (const auto & row : rows) {
    const std::string where = row.kind + " t=" + std::to_string(row.t);
    if (row.kind == "sine") {
      const SineTrajectory sine(row.p[0], row.p[1], row.p[2], row.p[3], row.p[4]);
      const auto s = sine.at(row.t);
      EXPECT_NEAR(s.position, row.out[0], kTolerance) << where;
      EXPECT_NEAR(s.velocity, row.out[1], kTolerance) << where;
      ++counts[0];
    } else if (row.kind == "vbound") {
      const SineTrajectory sine(row.p[0], row.p[1], row.p[2], row.p[3], row.p[4]);
      EXPECT_NEAR(sine.velocityUpperBound(), row.out[0], kTolerance) << where;
      ++counts[1];
    } else if (row.kind == "ramp") {
      const RampTrajectory ramp(row.p[0], row.p[1], row.p[2]);
      const auto s = ramp.at(row.t);
      EXPECT_NEAR(s.position, row.out[0], kTolerance) << where;
      EXPECT_NEAR(s.velocity, row.out[1], kTolerance) << where;
      ++counts[2];
    } else if (row.kind == "friction") {
      EXPECT_NEAR(frictionTorque(row.p[0], row.p[1], row.p[2], row.p[3], row.p[4]), row.out[0],
        kTolerance) << where;
      ++counts[3];
    } else {
      ADD_FAILURE() << "unknown kind " << row.kind;
    }
  }
  // Every kind is present, so a truncated table cannot pass silently.
  EXPECT_GE(counts[0], 600U);
  EXPECT_EQ(counts[1], 7U);
  EXPECT_GE(counts[2], 400U);
  EXPECT_EQ(counts[3], 45U);
}

// Largest |velocity| found by sampling the whole trajectory every step seconds.
double sampledMaxVelocity(const SineTrajectory & sine, double step)
{
  double peak = 0.0;
  const long steps = static_cast<long>(std::ceil(sine.duration() / step));
  for (long i = 0; i <= steps; ++i) {
    peak = std::max(peak, std::abs(sine.at(static_cast<double>(i) * step).velocity));
  }
  return peak;
}

TEST(HilTrajectory, VelocityBoundIncludesTheEnvelopeDerivative)
{
  // amplitude * omega is about 1.70 rad/s, but the 0.01-cycle ramp is far faster.
  const SineTrajectory short_ramp(0.0, 0.45, 0.6, 1.0, 0.01);
  EXPECT_LT(0.45 * 2.0 * M_PI * 0.6, 2.0);
  EXPECT_GT(short_ramp.velocityUpperBound(), 2.0);
  // A ramp far shorter than any sampling step: the peak in the middle of the ramp-down is about
  // 78,540 rad/s while amplitude * omega is 0.314 rad/s. The bound must still cover it.
  const SineTrajectory tiny_ramp(0.0, 0.1, 0.5, 0.25, 0.000001);
  const double ramp = 0.000001 / 0.5;
  const double mid_ramp_down = ramp + 0.25 / 0.5 + ramp / 2.0;
  const double peak = std::abs(tiny_ramp.at(mid_ramp_down).velocity);
  EXPECT_GT(peak, 78000.0);
  EXPECT_GE(tiny_ramp.velocityUpperBound(), peak);
  EXPECT_GT(tiny_ramp.velocityUpperBound(), 2.0);
  // HIL stage 4's first condition: hypot(0.1 pi, 0.1 pi / 4), below 2 rad/s.
  const SineTrajectory stage4(0.0452, 0.1, 0.5, 5.0, 1.0);
  EXPECT_NEAR(stage4.velocityUpperBound(), 0.1 * std::hypot(M_PI, M_PI / 4), 1e-12);
}

TEST(HilTrajectory, VelocityBoundIsNeverBelowTheSampledMaximum)
{
  const double sines[][5] = {
    {0.0, 1.5707963267948966, 0.1, 1.0, 2.0},
    {0.05, 0.2, 0.5, 3.0, 1.0}, {0.0452, 0.1, 0.5, 5.0, 1.0}, {-0.1, 0.3, 0.2, 5.0, 1.0},
    {0.0, 0.45, 0.6, 1.0, 0.01}, {0.0, 0.1, 2.0, 0.0, 0.5}, {0.0, 0.1, 0.5, 2.0, 0.0},
    {0.0, 0.5, 5.0, 3.0, 0.5}, {0.0, 0.3, 1.0, 0.0, 5.0},
  };
  for (const auto & p : sines) {
    const SineTrajectory sine(p[0], p[1], p[2], p[3], p[4]);
    // 1 us steps, far finer than any of these ramps.
    EXPECT_GE(sine.velocityUpperBound(), sampledMaxVelocity(sine, 1e-6))
      << p[1] << " rad " << p[2] << " Hz ramp " << p[4];
  }
}

TEST(HilTrajectory, RampReachesTheTargetAndStops)
{
  const RampTrajectory ramp(0.0277, 0.2, 0.3);
  EXPECT_NEAR(ramp.rampTime(), (0.2 - 0.0277) / 0.3, 1e-15);
  EXPECT_DOUBLE_EQ(ramp.at(0.0).position, 0.0277);
  EXPECT_DOUBLE_EQ(ramp.at(0.0).velocity, 0.3);
  EXPECT_DOUBLE_EQ(ramp.at(ramp.rampTime()).position, 0.2);
  EXPECT_DOUBLE_EQ(ramp.at(ramp.rampTime()).velocity, 0.0);
  const RampTrajectory back(0.1646, 0.0, 0.3);
  EXPECT_DOUBLE_EQ(back.at(0.1).velocity, -0.3);
  const RampTrajectory still(0.1, 0.1, 0.3);
  EXPECT_DOUBLE_EQ(still.rampTime(), 0.0);
  EXPECT_DOUBLE_EQ(still.at(0.0).position, 0.1);
  EXPECT_DOUBLE_EQ(still.at(0.0).velocity, 0.0);
}

TEST(HilTrajectory, AccelerationMatchesBothNumericalDerivatives)
{
  for (const double ramp : {1.0, 2.0}) {
    const SineTrajectory sine(0.0, M_PI / 6, 0.2, 5, ramp);
    const double h = 2e-4;
    // Avoid one-sided boundary derivatives. Central position difference has O(h^2)
    // truncation and O(epsilon/h^2) cancellation (about 1e-8 at this amplitude).
    // 1e-7 covers both. The velocity difference has much less cancellation (1e-8).
    for (double t = 0.01; t < sine.duration() - 0.01; t += 0.037) {
      // The joins are C2, not C3. A central stencil across a jerk discontinuity has
      // O(h), not O(h^2), error; join continuity is checked independently below.
      if (std::abs(t - ramp / .2) <= 2 * h || std::abs(t - (ramp + 5) / .2) <= 2 * h) {
        continue;
      }
      const auto left = sine.at(t - h);
      const auto mid = sine.at(t);
      const auto right = sine.at(t + h);
      EXPECT_NEAR(sine.accelerationAt(t),
        (right.position - 2 * mid.position + left.position) / (h * h), 1e-7) << t;
      EXPECT_NEAR(sine.accelerationAt(t), (right.velocity - left.velocity) / (2 * h), 1e-7)
        << t;
    }
  }
}

TEST(HilTrajectory, IntegerProfilesAreC2AtAllBoundaries)
{
  for (const double ramp : {1.0, 2.0}) {
    for (int cycles = 1; cycles <= 5; ++cycles) {
      const SineTrajectory sine(0.0, M_PI / 6, 0.2, cycles, ramp);
      for (const double end : {0.0, sine.duration()}) {
        EXPECT_NEAR(sine.at(end).position, 0.0, 1e-13);
        EXPECT_NEAR(sine.at(end).velocity, 0.0, 1e-13);
        EXPECT_NEAR(sine.accelerationAt(end), 0.0, 1e-13);
      }
      for (const double boundary : {0.0, ramp / 0.2, (ramp + cycles) / 0.2, sine.duration()}) {
        // q/v/a differences are O(epsilon), including the change in jerk at joins.
        const double epsilon = 1e-7;
        for (const double t : {boundary - epsilon, boundary, boundary + epsilon}) {
          EXPECT_NEAR(sine.at(t).position, sine.at(boundary).position, 1e-6);
          EXPECT_NEAR(sine.at(t).velocity, sine.at(boundary).velocity, 1e-6);
          EXPECT_NEAR(sine.accelerationAt(t), sine.accelerationAt(boundary), 1e-6);
        }
      }
    }
  }
}

TEST(HilTrajectory, AnalyticAccelerationBoundCoversIndependentDenseCrossCheck)
{
  for (const double ramp : {1.0, 2.0}) {
    for (int cycles = 1; cycles <= 5; ++cycles) {
      const SineTrajectory sine(0.0, M_PI / 6, 0.2, cycles, ramp);
      const auto bound = sine.accelerationUpperBound();
      ASSERT_TRUE(bound.has_value());
      double sampled = 0.0;
      // 0.1 ms is 10x finer than update. This is only a cross-check: the proof is
      // interval exclusion of the derivative polynomial and endpoint evaluation.
      const auto steps = static_cast<long>(std::ceil(sine.duration() / 1e-4));
      for (long i = 0; i <= steps; ++i) {
        const double t = i * 1e-4;
        sampled = std::max(sampled, std::abs(sine.accelerationAt(t)));
      }
      EXPECT_GE(*bound, sampled);
      EXPECT_LE(*bound, 0.90);
      if (cycles == 5) {
        std::ostringstream value;
        value << std::setprecision(17) << *bound;
        RecordProperty(ramp == 1.0 ? "ramp1_upper_bound" : "ramp2_upper_bound", value.str());
        value.str("");
        value << std::setprecision(17) << sampled;
        RecordProperty(ramp == 1.0 ? "ramp1_sampled_max" : "ramp2_sampled_max", value.str());
      }
      if (ramp == 1.0) {
        EXPECT_NEAR(sampled, 0.8674872373, 1e-7);
        EXPECT_GE(*bound, 0.8674872372888551);
        EXPECT_GT(*bound, 0.83);  // An admission limit of 0.83 MUST reject this profile.
        EXPECT_FALSE(*bound <= 0.83);
      } else {
        EXPECT_NEAR(sampled, 0.8269750706, 1e-7);
        EXPECT_LE(*bound, 0.83);
      }
    }
  }
}

TEST(HilTrajectory, AccelerationBoundScalesAndRejectsUnsupportedInput)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  // center, amplitude, frequency, cycles, ramp_cycles. The old constructor and at()
  // remain permissive; only the new bound API rejects these inputs with nullopt.
  const double invalid[][5] = {
    {nan, .5, .2, 5, 1}, {inf, .5, .2, 5, 1}, {0, nan, .2, 5, 1},
    {0, inf, .2, 5, 1}, {0, -.5, .2, 5, 1}, {0, 0, .2, 5, 1},
    {0, .5, nan, 5, 1}, {0, .5, inf, 5, 1}, {0, .5, -.2, 5, 1},
    {0, .5, 0, 5, 1}, {0, .5, .2, nan, 1}, {0, .5, .2, inf, 1},
    {0, .5, .2, -1, 1}, {0, .5, .2, 1.5, 1}, {0, .5, .2, 21, 1},
    {0, .5, .2, 5, nan}, {0, .5, .2, 5, inf}, {0, .5, .2, 5, -1},
    {0, .5, .2, 5, 0}, {0, .5, .2, 5, .5}, {0, .5, .2, 5, 1.5},
    {0, .5, .2, 5, 3}, {0, 1e300, 1e100, 5, 1}};
  for (const auto & p : invalid) {
    EXPECT_FALSE(SineTrajectory(p[0], p[1], p[2], p[3], p[4]).accelerationUpperBound());
  }
  for (const double ramp : {1.0, 2.0}) {
    const SineTrajectory base(0, .3, .2, 0, ramp);
    const SineTrajectory scaled(2, .6, .4, 20, ramp);
    ASSERT_TRUE(base.accelerationUpperBound());
    ASSERT_TRUE(scaled.accelerationUpperBound());
    EXPECT_NEAR(*scaled.accelerationUpperBound(), 8 * *base.accelerationUpperBound(), 1e-8);
  }
  EXPECT_TRUE(std::isnan(SineTrajectory(0, 0, .2, 5, 1).accelerationAt(1)));
  EXPECT_TRUE(std::isnan(SineTrajectory(0, .5, 0, 5, 1).accelerationAt(1)));
  EXPECT_TRUE(std::isnan(SineTrajectory(0, .5, .2, 5, 1).accelerationAt(nan)));
  EXPECT_TRUE(std::isnan(SineTrajectory(0, .5, .2, 5, 1).accelerationAt(inf)));
  // No-ramp and fractional legacy profiles still have finite acceleration evaluations;
  // the restricted G1-1 upper-bound API does not accept them.
  EXPECT_TRUE(std::isfinite(SineTrajectory(0, .5, .2, 5, 0).accelerationAt(1)));
  EXPECT_TRUE(std::isfinite(SineTrajectory(0, .5, .2, 5, .5).accelerationAt(1)));
}

TEST(HilTrajectory, LegacyPositionVelocityArithmeticAndBoundAreUnchanged)
{
  for (const double ramp_cycles : {0.0, 0.01, 0.5, 1.0, 2.0}) {
    const double ramp = ramp_cycles / .2, flat = 5 / .2;
    const SineTrajectory sine(.123, M_PI / 6, .2, 5, ramp_cycles);
    for (double t = -1; t < sine.duration() + 1; t += .013) {
      // Original at() arithmetic, including noninteger and zero ramps.
      const double ct = std::min(std::max(t, 0.0), sine.duration());
      double a = M_PI / 6, da = 0.0;
      if (ramp > 0.0) {
        if (ct < ramp) {
          const double x = ct / ramp;
          a = (M_PI / 6) * .5 * (1 - std::cos(M_PI * x));
          da = (M_PI / 6) * .5 * M_PI / ramp * std::sin(M_PI * x);
        } else if (ct > ramp + flat) {
          const double x = std::min((ct - ramp - flat) / ramp, 1.0);
          a = (M_PI / 6) * .5 * (1 + std::cos(M_PI * x));
          da = -(M_PI / 6) * .5 * M_PI / ramp * std::sin(M_PI * x);
        }
      }
      const double omega = 2 * M_PI * .2;
      EXPECT_DOUBLE_EQ(sine.at(t).position, .123 + a * std::sin(omega * ct));
      EXPECT_DOUBLE_EQ(sine.at(t).velocity,
        da * std::sin(omega * ct) + a * omega * std::cos(omega * ct));
    }
  }
  const SineTrajectory g1(0, M_PI / 6, .2, 5, 1);
  EXPECT_NEAR(g1.velocityUpperBound(), 0.6782236904792058, 1e-15);
}

}  // namespace
}  // namespace hil
}  // namespace bxi_hardware
