// The C++ trajectories (bxi_hardware::hil) against the table computed by the Python scripts
// (test/generate_trajectory_reference.py -> test/data/hil_trajectory_reference.csv).
#include <algorithm>
#include <cmath>
#include <fstream>
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

}  // namespace
}  // namespace hil
}  // namespace bxi_hardware
