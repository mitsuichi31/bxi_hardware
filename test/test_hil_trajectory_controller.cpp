// HilTrajectoryController unit tests: the controller is created directly with loaned command and
// state interfaces, and update() is called with chosen periods. The steady clock used for the
// lease and arming timeouts is driven by the test.
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <new>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "controller_interface/controller_interface_params.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "bxi_hardware/hil_trajectory.hpp"
#include "bxi_hardware/hil_trajectory_controller.hpp"

// Counts allocations while enabled, to check that update() does not allocate.
namespace
{
std::atomic<bool> g_count_allocations{false};
std::atomic<long> g_allocations{0};
}  // namespace

void * operator new(std::size_t size)
{
  if (g_count_allocations.load(std::memory_order_relaxed)) {
    g_allocations.fetch_add(1, std::memory_order_relaxed);
  }
  if (void * p = std::malloc(size == 0 ? 1 : size)) {
    return p;
  }
  throw std::bad_alloc();
}
void operator delete(void * p) noexcept {std::free(p);}
void operator delete(void * p, std::size_t) noexcept {std::free(p);}

namespace bxi_hardware
{
namespace
{

using Request = srv::StartHilTrajectory::Request;
using State = HilTrajectoryController::State;
using EndReason = HilTrajectoryController::EndReason;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kDt = 0.001;

class TestController : public HilTrajectoryController
{
public:
  // Atomic: the non-real-time calls read it from another thread in the race test.
  std::atomic<int64_t> now_ns{1'000'000'000};
  // The next fail_count writes to command interface fail_index are refused.
  int fail_index = -1;
  int fail_count = 0;
  using HilTrajectoryController::holdSnapshotLockForTest;

protected:
  int64_t steadyNowNs() const override {return now_ns.load();}
  bool writeCommand(size_t index, double value) override
  {
    if (static_cast<int>(index) == fail_index && fail_count > 0) {
      --fail_count;
      return false;
    }
    return HilTrajectoryController::writeCommand(index, value);
  }
};

class HilTrajectoryControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase() {rclcpp::init(0, nullptr);}
  static void TearDownTestCase() {rclcpp::shutdown();}

  virtual std::vector<rclcpp::Parameter> extraParameters() {return {};}

  void SetUp() override
  {
    out_dir_ = std::filesystem::path(::testing::TempDir()) /
      ("hil_trajectory_" + std::to_string(serial_++));
    std::filesystem::remove_all(out_dir_);
    std::filesystem::create_directories(out_dir_);
    controller_ = std::make_unique<TestController>();
    rclcpp::NodeOptions options;
    auto overrides = extraParameters();
    overrides.insert(overrides.end(), {
        rclcpp::Parameter("joint", "joint1"),
        rclcpp::Parameter("output_dir", output_dir_override_.value_or(out_dir_.string())),
        rclcpp::Parameter("record_seconds", record_seconds_),
      });
    options.parameter_overrides(overrides);
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "hil_trajectory";
    params.update_rate = 1000;
    params.node_options = options;
    ASSERT_EQ(controller_->init(params), controller_interface::return_type::OK);
    cmd_values_.assign(5, 0.0);
    const char * names[] = {"position", "velocity", "kp", "kd", "effort"};
    cmd_itfs_.reserve(5);
    std::vector<hardware_interface::LoanedCommandInterface> cmds;
    for (size_t i = 0; i < 5; ++i) {
      cmd_itfs_.emplace_back(hardware_interface::CommandInterface("joint1", names[i], &cmd_values_[i]));
      cmds.emplace_back(cmd_itfs_.back());
    }
    const char * state_names[] = {"position", "velocity", "effort"};
    double * state_values[] = {&position_, &velocity_, &effort_};
    state_itfs_.reserve(3);
    std::vector<hardware_interface::LoanedStateInterface> states;
    for (size_t i = 0; i < 3; ++i) {
      state_itfs_.emplace_back(
        hardware_interface::StateInterface("joint1", state_names[i], state_values[i]));
      states.emplace_back(state_itfs_.back());
    }
    controller_->assign_interfaces(std::move(cmds), std::move(states));
    ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()),
      controller_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()),
      controller_interface::CallbackReturn::SUCCESS);
  }

  void TearDown() override
  {
    controller_.reset();
    std::filesystem::remove_all(out_dir_);
  }

  // One update(): the steady clock and the ROS time advance by period.
  void step(double period = kDt, double ros_time_s = -1.0)
  {
    controller_->now_ns += static_cast<int64_t>(period * 1e9);
    ros_time_s_ = ros_time_s >= 0.0 ? ros_time_s : ros_time_s_ + period;
    ASSERT_EQ(controller_->update(rclcpp::Time(static_cast<int64_t>(ros_time_s_ * 1e9)),
      rclcpp::Duration::from_seconds(period)), controller_interface::return_type::OK);
  }
  void steps(int count, double period = kDt)
  {
    for (int i = 0; i < count; ++i) {
      step(period);
    }
  }
  void advanceClock(double seconds) {controller_->now_ns += static_cast<int64_t>(seconds * 1e9);}

  static Request sine(double amplitude = 0.1, double frequency = 0.5, double cycles = 1.0,
    double ramp_cycles = 1.0)
  {
    Request r;
    r.kind = Request::SINE;
    r.amplitude = amplitude;
    r.frequency = frequency;
    r.cycles = cycles;
    r.ramp_cycles = ramp_cycles;
    r.friction_static = -1.0;
    r.friction_velocity = 0.05;
    r.stribeck_velocity = 0.2;
    r.kp = 20.0;
    r.kd = 1.0;
    r.hold = 0.0;
    r.release = true;
    r.lease = 0.5;
    return r;
  }
  static Request ramp(double target = 0.2, double speed = 0.3, double hold = 0.0, bool release = false)
  {
    Request r;
    r.kind = Request::RAMP;
    r.target = target;
    r.speed = speed;
    r.friction_static = -1.0;
    r.friction_velocity = 0.05;
    r.stribeck_velocity = 0.2;
    r.kp = 10.0;
    r.kd = 2.0;
    r.hold = hold;
    r.release = release;
    r.lease = 0.5;
    return r;
  }

  // Accepts a request, applies it and sends its first keepalive: the next step() starts it.
  uint32_t startAndArm(const Request & r)
  {
    const auto result = controller_->requestStart(r);
    EXPECT_TRUE(result.accepted) << result.message;
    step();
    EXPECT_EQ(controller_->snapshot().state, State::kArmed);
    controller_->onKeepalive(result.id);
    return result.id;
  }

  bool outputsReleased() const
  {
    return cmd_values_[1] == 0.0 && cmd_values_[2] == 0.0 && cmd_values_[3] == 0.0 &&
           cmd_values_[4] == 0.0;
  }
  bool outputsNaN() const
  {
    for (double v : cmd_values_) {
      if (!std::isnan(v)) {
        return false;
      }
    }
    return true;
  }

  std::unique_ptr<TestController> controller_;
  std::vector<hardware_interface::CommandInterface> cmd_itfs_;
  std::vector<hardware_interface::StateInterface> state_itfs_;
  std::vector<double> cmd_values_;
  double position_ = 0.03;
  double velocity_ = 0.0;
  double effort_ = 0.0;
  double ros_time_s_ = 100.0;
  double record_seconds_ = 120.0;
  std::optional<std::string> output_dir_override_;
  std::filesystem::path out_dir_;
  static int serial_;
};
int HilTrajectoryControllerTest::serial_ = 0;

TEST_F(HilTrajectoryControllerTest, IdleAfterActivationWritesNaN)
{
  step();
  EXPECT_TRUE(outputsNaN());
  EXPECT_EQ(controller_->snapshot().state, State::kIdle);
}

TEST_F(HilTrajectoryControllerTest, RejectsRequestsOutsideTheLimits)
{
  std::vector<Request> bad;
  auto r = sine(); r.amplitude = kNaN; bad.push_back(r);
  r = sine(); r.kp = 25.0; bad.push_back(r);
  r = sine(); r.kp = 0.0; bad.push_back(r);
  r = sine(); r.kd = 2.5; bad.push_back(r);
  r = sine(); r.kd = -0.1; bad.push_back(r);
  r = sine(); r.lease = 0.05; bad.push_back(r);
  r = sine(); r.lease = 3.0; bad.push_back(r);
  r = sine(); r.hold = 11.0; bad.push_back(r);
  r = sine(); r.ramp_cycles = 0.4; bad.push_back(r);        // below the 0.5 lower bound
  r = sine(0.1, 0.5, 0.25, 0.000001); bad.push_back(r);     // ramp far too short
  r = sine(0.45, 0.7, 1.0, 0.5); bad.push_back(r);          // velocity bound over 2 rad/s
  r = sine(0.1, 0.5, 20.0, 5.0); r.hold = 10.0; bad.push_back(r);  // 60 + 10 s > 60 s
  r = sine(); r.friction_kinetic = 1.1; bad.push_back(r);
  r = sine(); r.friction_kinetic = 0.6; r.friction_static = 0.4; bad.push_back(r);
  r = sine(); r.frequency = 5.5; bad.push_back(r);
  r = sine(); r.amplitude = 0.6; bad.push_back(r);
  r = ramp(0.6); bad.push_back(r);                            // outside the joint range
  r = ramp(0.2, 0.0); bad.push_back(r);
  r = ramp(0.2, 2.5); bad.push_back(r);
  r = ramp(); r.kind = 7; bad.push_back(r);
  r = ramp(); r.speed = std::numeric_limits<double>::infinity(); bad.push_back(r);
  for (size_t i = 0; i < bad.size(); ++i) {
    const auto result = controller_->requestStart(bad[i]);
    EXPECT_FALSE(result.accepted) << "case " << i;
    EXPECT_FALSE(result.message.empty()) << "case " << i;
  }
  step();
  EXPECT_EQ(controller_->snapshot().state, State::kIdle);
  EXPECT_TRUE(controller_->requestStart(sine()).accepted);
}

TEST_F(HilTrajectoryControllerTest, AcceptedIsNotAppliedUntilUpdateAndMovesOnlyAfterKeepalive)
{
  const auto result = controller_->requestStart(ramp());
  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(controller_->snapshot().applied_id, 0U);  // accepted, not applied
  step();
  auto s = controller_->snapshot();
  EXPECT_EQ(s.state, State::kArmed);
  EXPECT_EQ(s.applied_id, result.id);
  steps(10);
  EXPECT_TRUE(outputsNaN());  // armed: still limp
  controller_->onKeepalive(result.id);
  step();
  EXPECT_EQ(controller_->snapshot().state, State::kRunning);
  EXPECT_DOUBLE_EQ(cmd_values_[0], 0.03);  // starts at the current position
  EXPECT_DOUBLE_EQ(cmd_values_[2], 10.0);
}

TEST_F(HilTrajectoryControllerTest, WritesTheTrajectoryValueEveryCycle)
{
  const auto r = sine(0.1, 0.5, 1.0, 1.0);
  const uint32_t id = startAndArm(r);
  const hil::SineTrajectory expected(position_, 0.1, 0.5, 1.0, 1.0);
  double elapsed = 0.0;
  double previous = kNaN;
  int changed = 0;
  for (int k = 0; k < 7000; ++k) {
    if (k % 50 == 0) {
      controller_->onKeepalive(id);
    }
    step();
    if (controller_->snapshot().state != State::kRunning) {
      break;
    }
    const auto sp = expected.at(elapsed);
    ASSERT_NEAR(cmd_values_[0], sp.position, 1e-12) << k;
    ASSERT_NEAR(cmd_values_[1], sp.velocity, 1e-12) << k;
    changed += cmd_values_[0] != previous ? 1 : 0;
    previous = cmd_values_[0];
    elapsed += kDt;
  }
  EXPECT_GT(changed, 5900);
}

TEST_F(HilTrajectoryControllerTest, RosTimeJumpsDoNotMoveTheTrajectoryAndLatePeriodsAdvanceIt)
{
  const auto r = sine(0.1, 0.5, 1.0, 1.0);
  startAndArm(r);
  const hil::SineTrajectory expected(position_, 0.1, 0.5, 1.0, 1.0);
  step(kDt, 500.0);   // starting cycle, elapsed 0
  step(kDt, 10.0);    // ROS time jumps back: elapsed 1 ms
  step(kDt, 9999.0);  // and forward: elapsed 2 ms
  EXPECT_NEAR(cmd_values_[0], expected.at(0.002).position, 1e-12);
  step(0.003);        // a late cycle: elapsed 5 ms
  EXPECT_NEAR(cmd_values_[0], expected.at(0.005).position, 1e-12);
  EXPECT_EQ(controller_->snapshot().state, State::kRunning);
}

TEST_F(HilTrajectoryControllerTest, PeriodOutOfRangeReleasesInTheSameCycle)
{
  for (const double bad : {0.006, 0.0, -0.001}) {
    SetUp();
    startAndArm(sine());
    steps(10);
    ASSERT_EQ(controller_->snapshot().state, State::kRunning);
    step(bad);
    const auto s = controller_->snapshot();
    EXPECT_EQ(s.state, State::kAborted) << bad;
    EXPECT_EQ(s.end_reason, EndReason::kPeriod) << bad;
    EXPECT_TRUE(outputsReleased()) << bad;
    TearDown();
  }
}

TEST_F(HilTrajectoryControllerTest, TorqueLimitReleasesInTheSameCycle)
{
  startAndArm(ramp(0.2, 0.3));
  steps(10);
  position_ = -0.5;  // Kp 10 x |0.033 - (-0.5)| = 5.3 N·m > 5 N·m
  step();
  const auto s = controller_->snapshot();
  EXPECT_EQ(s.state, State::kAborted);
  EXPECT_EQ(s.end_reason, EndReason::kTorque);
  EXPECT_TRUE(outputsReleased());
  EXPECT_DOUBLE_EQ(cmd_values_[0], -0.5);  // at the current position
}

TEST_F(HilTrajectoryControllerTest, NonFiniteStateReleases)
{
  startAndArm(ramp());
  steps(3);
  position_ = kNaN;
  step();
  EXPECT_EQ(controller_->snapshot().end_reason, EndReason::kStateNotFinite);
  EXPECT_TRUE(outputsNaN() || outputsReleased());
  EXPECT_DOUBLE_EQ(cmd_values_[2], 0.0);
}

TEST_F(HilTrajectoryControllerTest, StopReleasesOnTheNextCycle)
{
  startAndArm(ramp());
  steps(10);
  controller_->requestStop();
  EXPECT_EQ(controller_->snapshot().state, State::kRunning);  // not applied yet
  step();
  const auto s = controller_->snapshot();
  EXPECT_EQ(s.state, State::kReleased);
  EXPECT_EQ(s.end_reason, EndReason::kStop);
  EXPECT_EQ(s.applied_stop_count, 1U);
  EXPECT_TRUE(outputsReleased());
}

TEST_F(HilTrajectoryControllerTest, StopAfterAStartInTheSameCycleCancelsTheStart)
{
  ASSERT_TRUE(controller_->requestStart(ramp()).accepted);
  controller_->requestStop();
  step();
  EXPECT_EQ(controller_->snapshot().state, State::kIdle);
  EXPECT_TRUE(outputsNaN());
  steps(5);
  EXPECT_EQ(controller_->snapshot().applied_id, 0U);
}

TEST_F(HilTrajectoryControllerTest, StartAfterAnAppliedStopRuns)
{
  controller_->requestStop();
  step();
  const auto result = controller_->requestStart(ramp());
  ASSERT_TRUE(result.accepted) << result.message;
  step();
  EXPECT_EQ(controller_->snapshot().state, State::kArmed);
}

TEST_F(HilTrajectoryControllerTest, StartIsRejectedWhileAStopIsPendingOrARunIsActive)
{
  startAndArm(ramp());
  step();
  EXPECT_FALSE(controller_->requestStart(ramp()).accepted);  // running
  controller_->requestStop();
  EXPECT_FALSE(controller_->requestStart(ramp()).accepted);  // stop not applied yet
}

TEST_F(HilTrajectoryControllerTest, FinishOnlyWhileHoldingAndStopWinsOverFinish)
{
  startAndArm(ramp(0.05, 0.3));
  step();
  std::string message;
  EXPECT_FALSE(controller_->requestFinish(&message));  // still moving
  steps(200);
  ASSERT_EQ(controller_->snapshot().state, State::kHolding);
  EXPECT_DOUBLE_EQ(cmd_values_[0], 0.05);
  EXPECT_DOUBLE_EQ(cmd_values_[1], 0.0);
  EXPECT_TRUE(controller_->requestFinish(&message));
  controller_->requestStop();
  step();
  EXPECT_EQ(controller_->snapshot().end_reason, EndReason::kStop);
}

TEST_F(HilTrajectoryControllerTest, FinishWhileHoldingIsANormalEnd)
{
  const uint32_t id = startAndArm(ramp(0.05, 0.3));
  steps(200);
  ASSERT_EQ(controller_->snapshot().state, State::kHolding);
  controller_->onKeepalive(id);
  ASSERT_TRUE(controller_->requestFinish(nullptr));
  step();
  const auto s = controller_->snapshot();
  EXPECT_EQ(s.state, State::kReleased);
  EXPECT_EQ(s.end_reason, EndReason::kFinish);
  EXPECT_TRUE(outputsReleased());
}

TEST_F(HilTrajectoryControllerTest, ReleaseEndsAfterTheHoldTime)
{
  startAndArm(ramp(0.05, 0.3, 0.1, true));
  int running = 0;
  for (int k = 0; k < 1000 && controller_->snapshot().state != State::kReleased; ++k) {
    step();
    running += controller_->snapshot().state == State::kRunning ? 1 : 0;
  }
  const auto s = controller_->snapshot();
  EXPECT_EQ(s.end_reason, EndReason::kRelease);
  // (0.05 - 0.03) / 0.3 s moving + 0.1 s hold, 1 ms cycles.
  EXPECT_NEAR(running, (0.02 / 0.3 + 0.1) * 1000.0, 2.0);
}

TEST_F(HilTrajectoryControllerTest, LeaseExpiryWhileRunningAborts)
{
  const uint32_t id = startAndArm(ramp(0.2, 0.1));  // 1.7 s of motion
  for (int k = 0; k < 400; ++k) {  // keepalives every 50 ms keep it running
    step();
    if (k % 50 == 0) {
      controller_->onKeepalive(id);
    }
  }
  ASSERT_EQ(controller_->snapshot().state, State::kRunning);
  controller_->onKeepalive(id);  // the last one
  steps(499);
  EXPECT_EQ(controller_->snapshot().state, State::kRunning);  // lease 0.5 s not yet over
  steps(3);
  const auto s = controller_->snapshot();
  EXPECT_EQ(s.state, State::kAborted);
  EXPECT_EQ(s.end_reason, EndReason::kLease);
  EXPECT_TRUE(outputsReleased());
}

TEST_F(HilTrajectoryControllerTest, LeaseExpiryWhileHoldingIsAnAbortNotAFinish)
{
  startAndArm(ramp(0.05, 0.3));
  steps(100);
  ASSERT_EQ(controller_->snapshot().state, State::kHolding);
  steps(500);
  const auto s = controller_->snapshot();
  EXPECT_EQ(s.state, State::kAborted);
  EXPECT_EQ(s.end_reason, EndReason::kLease);
}

TEST_F(HilTrajectoryControllerTest, NoKeepaliveNeverMovesAndGivesUp)
{
  ASSERT_TRUE(controller_->requestStart(ramp()).accepted);  // the client never learns the id
  step();
  for (int k = 0; k < 1000; ++k) {
    step();
    ASSERT_TRUE(outputsNaN());
  }
  step();
  const auto s = controller_->snapshot();
  EXPECT_EQ(s.state, State::kAborted);
  EXPECT_EQ(s.end_reason, EndReason::kNoKeepalive);
  EXPECT_EQ(cmd_values_[2], 0.0);
}

TEST_F(HilTrajectoryControllerTest, LateKeepaliveAfterTheArmTimeoutDoesNotStart)
{
  const auto result = controller_->requestStart(ramp());
  step();
  steps(1001);
  ASSERT_EQ(controller_->snapshot().end_reason, EndReason::kNoKeepalive);
  controller_->onKeepalive(result.id);
  steps(5);
  EXPECT_EQ(controller_->snapshot().state, State::kAborted);
  EXPECT_DOUBLE_EQ(cmd_values_[2], 0.0);
}

TEST_F(HilTrajectoryControllerTest, OldIdKeepalivesNeitherStartNorExtendTheLease)
{
  // First run ends normally.
  const uint32_t first = startAndArm(ramp(0.05, 0.3, 0.0, true));
  steps(200);
  ASSERT_EQ(controller_->snapshot().end_reason, EndReason::kRelease);
  steps(600);  // record tail, then the CSV hand-over
  controller_->writeCompletedRecord();
  // Second run: keepalives with the first id do not start it.
  const auto second = controller_->requestStart(ramp(0.2, 0.3));
  ASSERT_TRUE(second.accepted) << second.message;
  step();
  controller_->onKeepalive(first);
  steps(5);
  EXPECT_EQ(controller_->snapshot().state, State::kArmed);
  EXPECT_TRUE(outputsNaN());
  // Once running, the first id does not keep it alive.
  controller_->onKeepalive(second.id);
  step();
  ASSERT_EQ(controller_->snapshot().state, State::kRunning);
  for (int k = 0; k < 600; ++k) {
    step();
    if (k % 50 == 0) {
      controller_->onKeepalive(first);
    }
  }
  EXPECT_EQ(controller_->snapshot().end_reason, EndReason::kLease);
}

TEST_F(HilTrajectoryControllerTest, SineOutOfRangeAtTheStartDoesNotMove)
{
  position_ = 0.45;  // 0.45 + 0.1 > 0.5
  startAndArm(sine(0.1));
  step();
  EXPECT_EQ(controller_->snapshot().end_reason, EndReason::kStartRange);
  EXPECT_DOUBLE_EQ(cmd_values_[2], 0.0);
}

TEST_F(HilTrajectoryControllerTest, RampLongerThanTheRunLimitAtTheStartDoesNotMove)
{
  position_ = -0.5;
  auto r = ramp(0.5, 0.01, 10.0);  // 100 s + 10 s
  startAndArm(r);
  step();
  EXPECT_EQ(controller_->snapshot().end_reason, EndReason::kDuration);
  EXPECT_DOUBLE_EQ(cmd_values_[2], 0.0);
}

TEST_F(HilTrajectoryControllerTest, DeactivationWritesNaN)
{
  startAndArm(ramp());
  steps(5);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(outputsNaN());
  EXPECT_EQ(controller_->snapshot().end_reason, EndReason::kDeactivated);
  EXPECT_TRUE(controller_->writeCompletedRecord());
}

TEST_F(HilTrajectoryControllerTest, RecordsFromArmingUntilTheReleasePlusTailAndWritesCsv)
{
  const uint32_t id = startAndArm(ramp(0.05, 0.3, 0.0, true));
  int cycles = 1;  // the arming cycle
  for (int k = 0; k < 1000 && controller_->snapshot().state != State::kReleased; ++k) {
    step();
    ++cycles;
  }
  ASSERT_EQ(controller_->snapshot().state, State::kReleased);
  const int release_cycles = cycles;
  steps(499);
  EXPECT_TRUE(controller_->snapshot().recording);  // 0.5 s tail not over
  step();
  EXPECT_FALSE(controller_->snapshot().recording);
  EXPECT_FALSE(controller_->requestStart(ramp()).accepted);  // CSV not written yet
  ASSERT_TRUE(controller_->writeCompletedRecord());
  const auto path = controller_->csvPath();
  std::ifstream file(path);
  ASSERT_TRUE(file) << path;
  std::string metadata;
  std::string header;
  std::getline(file, metadata);
  std::getline(file, header);
  EXPECT_NE(metadata.find("#run_id=" + std::to_string(id)), std::string::npos) << metadata;
  EXPECT_NE(metadata.find("end_reason=release"), std::string::npos) << metadata;
  EXPECT_NE(metadata.find("dropped_cycles=0"), std::string::npos) << metadata;
  int rows = 0;
  std::string last;
  for (std::string line; std::getline(file, line); ) {
    ++rows;
    last = line;
  }
  EXPECT_EQ(rows, release_cycles + 500);
  // The last recorded outputs are zero gains (kp and kd columns 7 and 8).
  std::vector<std::string> fields;
  std::stringstream stream(last);
  for (std::string f; std::getline(stream, f, ','); ) {
    fields.push_back(f);
  }
  ASSERT_GE(fields.size(), 8U);
  EXPECT_EQ(std::stod(fields[6]), 0.0);
  EXPECT_EQ(std::stod(fields[7]), 0.0);
  EXPECT_TRUE(controller_->requestStart(ramp()).accepted);
}

class SmallRecord : public HilTrajectoryControllerTest
{
public:
  void SetUp() override
  {
    record_seconds_ = 0.1;  // 100 rows
    HilTrajectoryControllerTest::SetUp();
  }
};

TEST_F(SmallRecord, FullRecordCountsDroppedCyclesAndKeepsControlling)
{
  const uint32_t id = startAndArm(ramp(0.2, 0.3, 0.0, true));
  for (int k = 0; k < 300; ++k) {
    step();
    if (k % 50 == 0) {
      controller_->onKeepalive(id);
    }
  }
  const auto s = controller_->snapshot();
  EXPECT_EQ(s.state, State::kRunning);  // control goes on after the record is full
  EXPECT_EQ(s.recorded_cycles, 100U);
  EXPECT_EQ(s.dropped_cycles, 201U);
}

TEST_F(HilTrajectoryControllerTest, UpdateDoesNotAllocate)
{
  // 1.5 s of sine (velocity bound 1.57 rad/s), then the release and part of the record tail.
  const uint32_t id = startAndArm(sine(0.1, 2.0, 1.0, 1.0));
  g_allocations = 0;
  for (int k = 0; k < 1800; ++k) {
    if (k % 50 == 0) {
      controller_->onKeepalive(id);  // outside the counted region
    }
    controller_->now_ns += 1'000'000;
    ros_time_s_ += kDt;
    const rclcpp::Time time(static_cast<int64_t>(ros_time_s_ * 1e9));
    const auto period = rclcpp::Duration::from_seconds(kDt);
    g_count_allocations = true;
    const auto ret = controller_->update(time, period);
    g_count_allocations = false;
    ASSERT_EQ(ret, controller_interface::return_type::OK);
  }
  EXPECT_EQ(controller_->snapshot().end_reason, EndReason::kRelease);
  EXPECT_EQ(g_allocations.load(), 0);
}

// The non-real-time entry points run on another thread while update() runs, as with the
// controller_manager's executor and its update loop. Run under ThreadSanitizer to find races.
TEST_F(HilTrajectoryControllerTest, NonRealTimeCallsRaceSafelyWithUpdate)
{
  std::atomic<bool> running{true};
  std::atomic<int> accepted{0};
  std::thread other([&]() {
      uint32_t id = 0;
      for (int k = 0; running; ++k) {
        const auto result = controller_->requestStart(ramp(0.05, 0.3, 0.0, k % 2 == 0));
        if (result.accepted) {
          id = result.id;
          ++accepted;
        }
        controller_->onKeepalive(id);
        if (k % 7 == 0) {
          controller_->requestFinish(nullptr);
        }
        if (k % 13 == 0) {
          controller_->requestStop();
        }
        (void)controller_->snapshot();
        controller_->writeCompletedRecord();
        (void)controller_->csvPath();
        std::this_thread::sleep_for(std::chrono::microseconds(200));
      }
    });
  for (int k = 0; k < 20000; ++k) {
    step();
  }
  running = false;
  other.join();
  EXPECT_GT(accepted.load(), 1);
}

TEST_F(HilTrajectoryControllerTest, RefusedGainWriteReleasesInTheSameCycle)
{
  startAndArm(ramp(0.2, 0.3));
  steps(10);
  controller_->fail_index = 2;  // Kp
  controller_->fail_count = 1;
  step();
  const auto s = controller_->snapshot();
  EXPECT_EQ(s.state, State::kAborted);
  EXPECT_EQ(s.end_reason, EndReason::kWriteFailed);
  EXPECT_EQ(s.write_failures, 1U);
  EXPECT_TRUE(outputsReleased());  // the released outputs went through in the same cycle
  EXPECT_FALSE(s.release_write_pending);
  steps(600);
  ASSERT_TRUE(controller_->writeCompletedRecord());
  std::ifstream file(controller_->csvPath());
  std::string line;
  std::getline(file, line);  // metadata
  std::getline(file, line);  // header
  bool saw_failed_cycle = false;
  while (std::getline(file, line)) {
    if (line.substr(line.rfind(',') + 1) == "0") {
      saw_failed_cycle = true;
    }
  }
  EXPECT_TRUE(saw_failed_cycle);  // the first refusal remains visible after the release succeeds
}

TEST_F(HilTrajectoryControllerTest, RefusedWriteWhileHoldingAlsoReleases)
{
  startAndArm(ramp(0.05, 0.3));
  steps(100);
  ASSERT_EQ(controller_->snapshot().state, State::kHolding);
  controller_->fail_index = 3;  // Kd
  controller_->fail_count = 1;
  step();
  EXPECT_EQ(controller_->snapshot().end_reason, EndReason::kWriteFailed);
  EXPECT_TRUE(outputsReleased());
}

TEST_F(HilTrajectoryControllerTest, RefusedReleasedWritesAreRetriedEveryCycleAndRecorded)
{
  startAndArm(ramp(0.2, 0.3, 0.0, true));
  steps(10);
  controller_->fail_index = 3;  // Kd refused five times: the gain write and four releases
  controller_->fail_count = 5;
  step();
  EXPECT_EQ(controller_->snapshot().end_reason, EndReason::kWriteFailed);
  EXPECT_DOUBLE_EQ(cmd_values_[3], 2.0);  // not yet zero
  steps(3);
  EXPECT_DOUBLE_EQ(cmd_values_[3], 2.0);
  step();                                  // the fifth refusal was the previous one
  EXPECT_DOUBLE_EQ(cmd_values_[3], 0.0);
  EXPECT_TRUE(outputsReleased());
  EXPECT_EQ(controller_->snapshot().write_failures, 4U);  // four failed cycles, five refusals
  steps(600);
  ASSERT_TRUE(controller_->writeCompletedRecord());
  std::ifstream file(controller_->csvPath());
  std::string line;
  std::getline(file, line);
  std::getline(file, line);
  EXPECT_NE(line.find(",write_ok"), std::string::npos) << line;
  int refused_rows = 0;
  while (std::getline(file, line)) {
    refused_rows += line.substr(line.rfind(',') + 1) == "0" ? 1 : 0;
  }
  EXPECT_EQ(refused_rows, 4);  // the gain cycle and the three released cycles still refused
}

TEST_F(HilTrajectoryControllerTest, PersistentReleaseFailureKeepsRecordingAndBlocksTheNextRun)
{
  startAndArm(ramp(0.2, 0.3, 0.0, true));
  steps(10);
  controller_->fail_index = 3;  // Kd stays nonzero while this interface refuses the release
  controller_->fail_count = 1000;
  step();
  ASSERT_EQ(controller_->snapshot().end_reason, EndReason::kWriteFailed);
  steps(600);  // beyond the 0.5 s record tail
  EXPECT_DOUBLE_EQ(cmd_values_[3], 2.0);
  EXPECT_TRUE(controller_->snapshot().recording);
  EXPECT_TRUE(controller_->snapshot().release_write_pending);
  EXPECT_FALSE(controller_->writeCompletedRecord());
  EXPECT_FALSE(controller_->requestStart(ramp()).accepted);
  controller_->fail_count = 0;
  step();  // the first successful release is recorded before the previous run is handed over
  EXPECT_TRUE(outputsReleased());
  EXPECT_FALSE(controller_->snapshot().recording);
  EXPECT_FALSE(controller_->snapshot().release_write_pending);
  ASSERT_TRUE(controller_->writeCompletedRecord());
  EXPECT_TRUE(controller_->requestStart(ramp()).accepted);
}

TEST_F(HilTrajectoryControllerTest, NonFiniteVelocityOrEffortAborts)
{
  for (const int which : {0, 1}) {
    for (const State where : {State::kArmed, State::kRunning, State::kHolding}) {
      SetUp();
      const uint32_t id = startAndArm(ramp(0.05, 0.3));
      if (where != State::kArmed) {
        step();
      }
      if (where == State::kHolding) {
        steps(100);
        controller_->onKeepalive(id);
        ASSERT_EQ(controller_->snapshot().state, State::kHolding);
      }
      (which == 0 ? velocity_ : effort_) = kNaN;
      step();
      EXPECT_EQ(controller_->snapshot().end_reason, EndReason::kStateNotFinite)
        << (which == 0 ? "velocity" : "effort") << " state " << static_cast<int>(where);
      EXPECT_DOUBLE_EQ(cmd_values_[2], 0.0);
      TearDown();
      velocity_ = 0.0;
      effort_ = 0.0;
    }
  }
}

// The published snapshot can lag (update() only try-locks it); start and finish decide on the
// state update() stores every cycle instead.
TEST_F(HilTrajectoryControllerTest, StartAndFinishDoNotDependOnTheSnapshot)
{
  // The snapshot lock is held only while update() runs, so that update() cannot refresh the
  // snapshot but the requests (and a mistaken use of snapshot() in them) never block.
  const auto stale_steps = [this](int n) {
      auto frozen = controller_->holdSnapshotLockForTest();
      steps(n);
    };
  const auto first = controller_->requestStart(ramp(0.05, 0.3));
  ASSERT_TRUE(first.accepted);
  stale_steps(1);                           // applied; the snapshot still says idle
  ASSERT_EQ(controller_->snapshot().state, State::kIdle);
  EXPECT_FALSE(controller_->requestStart(ramp()).accepted);
  controller_->onKeepalive(first.id);
  stale_steps(100);                         // holding now; the snapshot still says idle
  ASSERT_EQ(controller_->snapshot().state, State::kIdle);
  EXPECT_TRUE(controller_->requestFinish(nullptr));
  step();
  EXPECT_EQ(controller_->snapshot().end_reason, EndReason::kFinish);
}

TEST_F(HilTrajectoryControllerTest, FailedCsvWriteKeepsTheRecordAndRefusesTheNextStart)
{
  startAndArm(ramp(0.05, 0.3, 0.0, true));
  steps(700);
  std::filesystem::remove_all(out_dir_);    // the output directory is gone
  EXPECT_FALSE(controller_->writeCompletedRecord());
  const auto refused = controller_->requestStart(ramp());
  EXPECT_FALSE(refused.accepted);
  EXPECT_NE(refused.message.find("書き出せていない"), std::string::npos) << refused.message;
  step();
  EXPECT_EQ(controller_->snapshot().state, State::kReleased);  // not overwritten by a new run
  std::filesystem::create_directories(out_dir_);
  ASSERT_TRUE(controller_->writeCompletedRecord());          // the retry writes the kept rows
  std::ifstream file(controller_->csvPath());
  int lines = 0;
  for (std::string line; std::getline(file, line); ) {
    ++lines;
  }
  EXPECT_GT(lines, 500);
  EXPECT_TRUE(controller_->requestStart(ramp()).accepted);
}

TEST_F(HilTrajectoryControllerTest, RecordEndedByDeactivationIsWrittenOnDestruction)
{
  // The HIL runner stops the launch at once on an NG: the run ends by deactivation and the status
  // timer does not run again. The record must still reach a CSV.
  startAndArm(ramp());
  steps(20);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  controller_.reset();
  std::vector<std::filesystem::path> files;
  for (const auto & entry : std::filesystem::directory_iterator(out_dir_)) {
    files.push_back(entry.path());
  }
  ASSERT_EQ(files.size(), 1U);
  std::ifstream file(files[0]);
  std::string metadata;
  std::getline(file, metadata);
  EXPECT_NE(metadata.find("end_reason=deactivated"), std::string::npos) << metadata;
}

class NoOutputDir : public HilTrajectoryControllerTest
{
public:
  void SetUp() override
  {
    unsetenv("BXI_WORKER_TIMING_DIR");
    output_dir_override_ = "";
    HilTrajectoryControllerTest::SetUp();
  }
};

TEST_F(NoOutputDir, RecordIsDiscardedAndTheNextStartAccepted)
{
  startAndArm(ramp(0.05, 0.3, 0.0, true));
  steps(700);
  EXPECT_FALSE(controller_->writeCompletedRecord());  // nothing written
  EXPECT_TRUE(controller_->csvPath().empty());
  EXPECT_TRUE(controller_->requestStart(ramp()).accepted);
}

// The test-only service delays (vcan fault paths) must never be usable by accident.
class TestFaultConfig : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  static void TearDownTestCase() {rclcpp::shutdown();}
  void TearDown() override {unsetenv("BXI_ALLOW_TEST_FAULTS");}

  // init() and on_configure() with the given test_fault parameters.
  static std::pair<std::unique_ptr<HilTrajectoryController>, controller_interface::CallbackReturn>
  configure(const std::vector<rclcpp::Parameter> & faults)
  {
    auto controller = std::make_unique<HilTrajectoryController>();
    std::vector<rclcpp::Parameter> overrides{rclcpp::Parameter("joint", "joint1"),
      rclcpp::Parameter("record_seconds", 1.0)};
    overrides.insert(overrides.end(), faults.begin(), faults.end());
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "hil_trajectory_fault_" + std::to_string(serial_++);
    params.update_rate = 1000;
    params.node_options = options;
    EXPECT_EQ(controller->init(params), controller_interface::return_type::OK);
    const auto result = controller->on_configure(rclcpp_lifecycle::State());
    return {std::move(controller), result};
  }

private:
  static inline int serial_ = 0;
};

TEST_F(TestFaultConfig, DelayIsRefusedWithoutTheEnvironmentSwitch)
{
  unsetenv("BXI_ALLOW_TEST_FAULTS");
  EXPECT_EQ(configure({rclcpp::Parameter("test_fault.stop_delay", 0.2)}).second,
    controller_interface::CallbackReturn::ERROR);
  ASSERT_EQ(setenv("BXI_ALLOW_TEST_FAULTS", "yes", 1), 0);  // only "1" enables it
  EXPECT_EQ(configure({rclcpp::Parameter("test_fault.start_delay", 0.2)}).second,
    controller_interface::CallbackReturn::ERROR);
  // Zero (the default) needs no switch.
  unsetenv("BXI_ALLOW_TEST_FAULTS");
  EXPECT_EQ(configure({}).second, controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestFaultConfig, DelayOutOfRangeIsRefusedEvenWhenAllowed)
{
  ASSERT_EQ(setenv("BXI_ALLOW_TEST_FAULTS", "1", 1), 0);
  EXPECT_EQ(configure({rclcpp::Parameter("test_fault.finish_delay", -0.1)}).second,
    controller_interface::CallbackReturn::ERROR);
  EXPECT_EQ(configure({rclcpp::Parameter("test_fault.stop_delay", 31.0)}).second,
    controller_interface::CallbackReturn::ERROR);
  EXPECT_EQ(configure({rclcpp::Parameter("test_fault.start_delay", kNaN)}).second,
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(TestFaultConfig, StopServiceAnswersOnlyAfterTheDelay)
{
  ASSERT_EQ(setenv("BXI_ALLOW_TEST_FAULTS", "1", 1), 0);
  auto [controller, result] = configure({rclcpp::Parameter("test_fault.stop_delay", 0.3)});
  ASSERT_EQ(result, controller_interface::CallbackReturn::SUCCESS);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller->get_node()->get_node_base_interface());
  auto client_node = std::make_shared<rclcpp::Node>("stop_delay_client");
  executor.add_node(client_node);
  auto client = client_node->create_client<std_srvs::srv::Trigger>(
    "/" + std::string(controller->get_node()->get_name()) + "/stop");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  const auto sent = std::chrono::steady_clock::now();
  auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(executor.spin_until_future_complete(future, std::chrono::seconds(3)),
    rclcpp::FutureReturnCode::SUCCESS);
  const double waited = std::chrono::duration<double>(std::chrono::steady_clock::now() - sent).count();
  EXPECT_GE(waited, 0.3);
  EXPECT_TRUE(future.get()->success);
}

class WideSineControllerTest : public HilTrajectoryControllerTest
{
public:
  std::vector<rclcpp::Parameter> extraParameters() override
  {
    return {rclcpp::Parameter("lower", -1.65), rclcpp::Parameter("upper", 1.65),
      rclcpp::Parameter("max_amplitude", 1.5707963267948966),
      rclcpp::Parameter("max_velocity", 1.0), rclcpp::Parameter("max_run_seconds", 65.0),
      rclcpp::Parameter("fixed_sine_center", true), rclcpp::Parameter("sine_center", 0.0)};
  }
};

TEST_F(WideSineControllerTest, FasterNinetyDegreesRespectsVelocityAndReleases)
{
  const auto request = sine(1.5707963267948966, 0.1, 1.0, 2.0);
  const auto id = startAndArm(request);
  const hil::SineTrajectory expected(0.0, request.amplitude, 0.1, 1.0, 2.0);
  EXPECT_LT(expected.velocityUpperBound(), 1.0);
  double elapsed = 0.0, minimum = 10.0, maximum = -10.0;
  for (int k = 0; k < 52500; ++k) {
    if (k % 50 == 0) {controller_->onKeepalive(id);}
    step();
    if (controller_->snapshot().state == State::kRunning) {
      const auto target = expected.at(elapsed);
      ASSERT_NEAR(cmd_values_[0], target.position, 1e-10);
      ASSERT_NEAR(cmd_values_[1], target.velocity, 1e-10);
      ASSERT_LE(std::abs(cmd_values_[1]), 1.0);
      minimum = std::min(minimum, cmd_values_[0]);
      maximum = std::max(maximum, cmd_values_[0]);
      position_ = cmd_values_[0];
      elapsed += kDt;
    }
  }
  EXPECT_NEAR(minimum, -1.5707963267948966, 1e-7);
  EXPECT_NEAR(maximum, 1.5707963267948966, 1e-7);
  EXPECT_TRUE(outputsReleased());
}

TEST_F(WideSineControllerTest, ZeroCenteredNinetyDegreesAndRelease)
{
  const auto request = sine(1.5707963267948966, 0.05, 1.0, 1.0);
  const auto id = startAndArm(request);
  const hil::SineTrajectory expected(0.0, request.amplitude, 0.05, 1.0, 1.0);
  double elapsed = 0.0;
  double minimum = 10.0, maximum = -10.0;
  for (int k = 0; k < 62500; ++k) {
    if (k % 50 == 0) {controller_->onKeepalive(id);}
    step();
    if (controller_->snapshot().state == State::kRunning) {
      const auto target = expected.at(elapsed);
      ASSERT_NEAR(cmd_values_[0], target.position, 1e-10);
      ASSERT_NEAR(cmd_values_[1], target.velocity, 1e-10);
      minimum = std::min(minimum, cmd_values_[0]);
      maximum = std::max(maximum, cmd_values_[0]);
      position_ = cmd_values_[0];  // tracking model; torque protection remains enabled
      elapsed += kDt;
    }
  }
  EXPECT_NEAR(minimum, -1.5707963267948966, 1e-7);
  EXPECT_NEAR(maximum, 1.5707963267948966, 1e-7);
  EXPECT_TRUE(outputsReleased());
}

TEST_F(WideSineControllerTest, RejectsExcessAmplitudeSpeedAndDuration)
{
  EXPECT_FALSE(controller_->requestStart(sine(1.58, 0.05, 1, 1)).accepted);
  EXPECT_FALSE(controller_->requestStart(sine(1.5707963267948966, 0.5, 1, 1)).accepted);
  EXPECT_FALSE(controller_->requestStart(sine(1.5707963267948966, 0.05, 5, 1)).accepted);
}

TEST_F(WideSineControllerTest, RefusesDisplacedStartBeforeApplyingGains)
{
  position_ = 0.06;
  startAndArm(sine(1.5707963267948966, 0.05, 1.0, 1.0));
  step();
  EXPECT_NE(controller_->snapshot().state, State::kRunning);
  EXPECT_TRUE(outputsReleased());
  EXPECT_EQ(controller_->snapshot().end_reason, EndReason::kStartRange);
}

}  // namespace
}  // namespace bxi_hardware
