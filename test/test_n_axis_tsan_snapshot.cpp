#include <gtest/gtest.h>
#include <array>
#include <atomic>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <thread>
#include "controller_interface/controller_interface_params.hpp"
#include "bxi_hardware/n_axis_trajectory_controller.hpp"

namespace bxi_hardware
{
// Test-only friend; production sources and existing tests are unchanged.
struct NAxisTestAccess
{
  static bool lockFree(const NAxisTrajectoryController & c)
  {return c.requests_.lockFree() && c.hearts_.lockFree();}
  static void publish(NAxisTrajectoryController & c,
    const NAxisTrajectoryController::Request & r,
    const NAxisTrajectoryController::Heartbeat & h)
  {
    std::lock_guard<std::mutex> lock(c.input_mutex_);
    c.requests_.publish(&r); c.hearts_.publish(&h);
  }
  static bool request(NAxisTrajectoryController & c, NAxisTrajectoryController::Request & r)
  {return c.requests_.tryRead(&r);}
  static bool heartbeat(NAxisTrajectoryController & c, NAxisTrajectoryController::Heartbeat & h)
  {return c.hearts_.tryRead(&h);}
  // Read only on the update thread, or after both threads have joined.
  static const auto & run(const NAxisTrajectoryController & c) {return c.run_;}
  static const auto & heart(const NAxisTrajectoryController & c) {return c.heart_;}
  static const auto & row(const NAxisTrajectoryController & c) {return c.final_row_;}
  static void holdingTime(NAxisTrajectoryController & c) {c.elapsed_ = 35.1;}
};
namespace
{
using C = NAxisTrajectoryController;
using CB = controller_interface::CallbackReturn;
using Ret = controller_interface::return_type;
constexpr uint64_t kCount = 100000;
constexpr int64_t kSent = 2000000000;

C::Request patternedRequest(uint64_t sequence, size_t n)
{
  C::Request r;
  r.nonce = 3 * sequence + n; r.id = sequence;
  r.sent_ns = kSent + 3 * sequence; r.received_ns = r.sent_ns + 2; r.axes = n;
  r.amplitude = sequence % 11; r.frequency = sequence % 13;
  r.cycles = sequence % 17; r.ramp_cycles = sequence % 19; r.hold = sequence % 23;
  for (size_t a = 0; a < 3; ++a) {
    r.kp[a] = sequence + a; r.kd[a] = 2 * sequence + a;
  }
  r.release = (sequence % 2) != 0;
  return r;
}
bool sameRequest(const C::Request & a, const C::Request & b)
{
  return a.nonce == b.nonce && a.id == b.id && a.sent_ns == b.sent_ns &&
    a.received_ns == b.received_ns && a.axes == b.axes && a.amplitude == b.amplitude &&
    a.frequency == b.frequency && a.cycles == b.cycles && a.ramp_cycles == b.ramp_cycles &&
    a.hold == b.hold && a.kp == b.kp && a.kd == b.kd && a.release == b.release;
}
C::Heartbeat patternedHeart(uint64_t sequence, size_t n)
{return {5 * sequence + n, 7 * sequence + n, sequence, kSent + 3 * static_cast<int64_t>(sequence)};}
bool sameHeart(const C::Heartbeat & a, const C::Heartbeat & b)
{return a.nonce == b.nonce && a.id == b.id && a.sequence == b.sequence && a.sent_ns == b.sent_ns;}

class FakeController : public C
{
public:
  // Only this clock is shared by the harness. The command array is update-thread-only.
  std::atomic<int64_t> now{kSent + 200000};
  std::array<double, 16> command{};
  size_t axes = 0;
  uint64_t setters = 0;
protected:
  int64_t steadyNow() const noexcept override {return now.load();}
  bool setCommand(size_t i, double value) override
  {++setters; command[i] = value; return true;}
  std::optional<double> getCommand(size_t i) override {return command[i];}
  std::optional<double> getState(size_t i) override
  {return i == 3 * axes ? command[5 * axes] : 0.;}
};
struct Rig
{
  // Backing loans are bound before threads start, and outlive the controller.
  std::array<double, 16> command_backing{};
  std::array<double, 10> state_backing{};
  std::vector<hardware_interface::CommandInterface> commands;
  std::vector<hardware_interface::StateInterface> states;
  FakeController c;
  explicit Rig(size_t n)
  {
    c.axes = n;
    std::vector<std::string> joints;
    for (size_t a = 0; a < n; ++a) {joints.push_back("joint" + std::to_string(a + 1));}
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("joints", joints),
      rclcpp::Parameter("record_seconds", 120.)});
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "g2a_snapshot"; params.update_rate = 1000; params.node_options = options;
    EXPECT_EQ(c.init(params), Ret::OK);
    EXPECT_EQ(c.on_configure(rclcpp_lifecycle::State()), CB::SUCCESS);
    EXPECT_TRUE(NAxisTestAccess::lockFree(c));
    const auto ck = c.command_interface_configuration().names;
    const auto sk = c.state_interface_configuration().names;
    commands.reserve(ck.size()); states.reserve(sk.size());
    std::vector<hardware_interface::LoanedCommandInterface> cl;
    std::vector<hardware_interface::LoanedStateInterface> sl;
    for (size_t i = 0; i < ck.size(); ++i) {
      const auto p = ck[i].find('/');
      commands.emplace_back(ck[i].substr(0, p), ck[i].substr(p + 1), &command_backing[i]);
      cl.emplace_back(commands.back());
    }
    for (size_t i = 0; i < sk.size(); ++i) {
      const auto p = sk[i].find('/');
      states.emplace_back(sk[i].substr(0, p), sk[i].substr(p + 1), &state_backing[i]);
      sl.emplace_back(states.back());
    }
    c.assign_interfaces(std::move(cl), std::move(sl));
  }
  Ret step()
  {return c.update(rclcpp::Time(c.now.load()), rclcpp::Duration::from_nanoseconds(1000));}
  C::Request request() const
  {
    C::Request r; r.nonce = 0xabc000 + c.axes; r.id = 41 + c.axes;
    r.sent_ns = kSent; r.axes = c.axes; r.release = false; return r;
  }
  void deactivate() {EXPECT_EQ(c.on_deactivate(rclcpp_lifecycle::State()), CB::SUCCESS);}
};
void waitFor(const std::atomic<bool> & ready)
{while (!ready.load()) {std::this_thread::yield();}}
class SnapshotStress : public ::testing::TestWithParam<size_t>
{
public:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_P(SnapshotStress, WholeRequestAndHeartbeatSnapshotsNeverMixFields)
{
  // Synthetic payloads exercise every field, independently of the one-run public gate.
  C c;
  ASSERT_TRUE(NAxisTestAccess::lockFree(c));
  const size_t n = GetParam();
  std::atomic<bool> ready{false}, done{false};
  uint64_t request_reads = 0, heart_reads = 0, reads = 0, bad = 0, previous_request = 0, previous_heart = 0;
  std::thread writer([&] {
    waitFor(ready);
    for (uint64_t i = 1; i <= kCount; ++i) {
      NAxisTestAccess::publish(c, patternedRequest(i, n), patternedHeart(i, n));
    }
    done = true;
  });
  std::thread reader([&] {
    ready = true;
    do {
      C::Request r; C::Heartbeat h;
      if (NAxisTestAccess::request(c, r)) {
        ++request_reads;
        if (r.id < previous_request || !sameRequest(r, patternedRequest(r.id, n))) {++bad;}
        previous_request = r.id;
      }
      if (NAxisTestAccess::heartbeat(c, h)) {
        ++heart_reads;
        if (h.sequence < previous_heart || !sameHeart(h, patternedHeart(h.sequence, n))) {++bad;}
        previous_heart = h.sequence;
      }
      ++reads;
    } while (reads < kCount || !done.load());
  });
  writer.join(); reader.join();
  C::Request final_request; C::Heartbeat final_heart;
  ASSERT_TRUE(NAxisTestAccess::request(c, final_request));
  ASSERT_TRUE(NAxisTestAccess::heartbeat(c, final_heart));
  EXPECT_TRUE(sameRequest(final_request, patternedRequest(kCount, n)));
  EXPECT_TRUE(sameHeart(final_heart, patternedHeart(kCount, n)));
  EXPECT_EQ(bad, 0U); EXPECT_GT(request_reads, 0U); EXPECT_GT(heart_reads, 0U);
  std::cout << "G2a N=" << n << " synthetic_writes=" << kCount << " reader_cycles=" << reads
            << " request_reads=" << request_reads << " heart_reads=" << heart_reads << '\n';
}

TEST_P(SnapshotStress, PublicRequestHeartbeatWithUpdateThenSenderSilenceExpiresLease)
{
  Rig r(GetParam());
  ASSERT_EQ(r.c.on_activate(rclcpp_lifecycle::State()), CB::SUCCESS);
  ASSERT_EQ(r.step(), Ret::OK);
  const auto request = r.request();
  std::atomic<bool> ready{false}, requested{false};
  uint64_t written = 0, rejected_requests = 0, observed = 0, bad = 0;
  std::thread writer([&] {
    waitFor(ready);
    requested = r.c.requestStart(request);
    for (uint64_t i = 1; i <= kCount; ++i) {
      if (r.c.keepalive({request.nonce, request.id, i, kSent + static_cast<int64_t>(i)})) {++written;}
      if (!r.c.requestStart(request)) {++rejected_requests;}
    }
  });
  std::thread updater([&] {
    ready = true;
    uint64_t previous = 0;
    for (uint64_t i = 0; i < kCount; ++i) {
      if (r.step() != Ret::OK) {++bad;}
      const auto & run = NAxisTestAccess::run(r.c);
      const auto & h = NAxisTestAccess::heart(r.c);
      if (run.id) {
        auto expected = request; expected.received_ns = kSent + 200000;
        if (!sameRequest(run, expected)) {++bad;}
      }
      if (h.sequence) {
        ++observed;
        if (h.sequence < previous || h.nonce != request.nonce || h.id != request.id ||
          h.sent_ns != kSent + static_cast<int64_t>(h.sequence) ||
          NAxisTestAccess::row(r.c).deadline_ns != h.sent_ns + C::kLeaseNs) {++bad;}
        previous = h.sequence;
      }
    }
  });
  writer.join(); updater.join();
  EXPECT_TRUE(requested); EXPECT_EQ(written, kCount); EXPECT_EQ(rejected_requests, kCount);
  EXPECT_EQ(bad, 0U);
  // Consume the last publication after joining; no scheduling assumption about overlap.
  ASSERT_EQ(r.step(), Ret::OK);
  const auto h = NAxisTestAccess::heart(r.c);
  EXPECT_EQ(h.sequence, kCount);
  r.c.now = h.sent_ns + C::kLeaseNs - 1;
  EXPECT_EQ(r.step(), Ret::OK);
  r.c.now = h.sent_ns + C::kLeaseNs;
  EXPECT_EQ(r.step(), Ret::ERROR);
  const auto row = NAxisTestAccess::row(r.c);
  EXPECT_EQ(row.reason, C::Reason::kLease); EXPECT_TRUE(row.released_attempted);
  EXPECT_EQ(row.released.read_ok, (1U << (5 * GetParam() + 1)) - 1);
  const auto setters = r.c.setters;
  for (int i = 0; i < 3; ++i) {EXPECT_EQ(r.step(), Ret::ERROR);}
  EXPECT_EQ(r.c.setters, setters);
  EXPECT_FALSE(r.c.keepalive({request.nonce, request.id, kCount + 1, r.c.now.load()}));
  r.deactivate();
  std::cout << "G2a N=" << GetParam() << " update_cycles=" << kCount
            << " heart_accepted=" << written << " request_rejected=" << rejected_requests
            << " heart_observations=" << observed << " lease_boundary=expired\n";
}

TEST_P(SnapshotStress, FinishAndStopConcurrentWithUpdate)
{
  Rig r(GetParam());
  ASSERT_EQ(r.c.on_activate(rclcpp_lifecycle::State()), CB::SUCCESS);
  ASSERT_EQ(r.step(), Ret::OK);
  const auto request = r.request(); ASSERT_TRUE(r.c.requestStart(request));
  ASSERT_TRUE(r.c.keepalive({request.nonce, request.id, 1, kSent}));
  ASSERT_EQ(r.step(), Ret::OK);
  NAxisTestAccess::holdingTime(r.c); ASSERT_EQ(r.step(), Ret::OK);
  ASSERT_EQ(NAxisTestAccess::row(r.c).after, C::State::kHolding);
  std::atomic<bool> ready{false};
  uint64_t finishes = 0, bad = 0;
  std::thread writer([&] {
    waitFor(ready);
    for (uint64_t i = 0; i < kCount; ++i) {
      if (r.c.requestFinish()) {++finishes;}
      r.c.requestStop();
      r.c.requestStart(request);
      r.c.keepalive({request.nonce, request.id, i + 2, kSent});
    }
  });
  std::thread updater([&] {
    ready = true;
    for (uint64_t i = 0; i < kCount; ++i) {if (r.step() != Ret::OK) {++bad;}}
  });
  writer.join(); updater.join();
  ASSERT_EQ(r.step(), Ret::OK);
  EXPECT_EQ(bad, 0U); EXPECT_GT(finishes, 0U);
  EXPECT_EQ(NAxisTestAccess::row(r.c).after, C::State::kStopped);
  EXPECT_EQ(NAxisTestAccess::row(r.c).reason, C::Reason::kStop);
  r.deactivate();
  std::cout << "G2a N=" << GetParam() << " stop_finish_calls=" << kCount
            << " update_cycles=" << kCount << " finish_accepted=" << finishes << '\n';
}

TEST_P(SnapshotStress, LifecycleAtomicsConcurrentWithPublicEntryOnly)
{
  Rig r(GetParam()); const auto request = r.request();
  std::atomic<uint64_t> calls{0};
  uint64_t accepted = 0;
  std::thread entry([&] {
    for (uint64_t i = 0; i < kCount; ++i) {
      accepted += r.c.requestStart(request);
      accepted += r.c.keepalive({request.nonce, request.id, i + 1, kSent});
      accepted += r.c.requestFinish(); r.c.requestStop();
      calls.fetch_add(1);
    }
  });
  while (calls.load() < 1000) {std::this_thread::yield();}
  EXPECT_EQ(r.c.on_activate(rclcpp_lifecycle::State()), CB::SUCCESS);
  r.deactivate(); // no concurrent update; lifecycle/CM serialization remains a prerequisite
  entry.join();
  EXPECT_EQ(accepted, 0U); EXPECT_EQ(calls.load(), kCount);
  std::cout << "G2a N=" << GetParam() << " lifecycle_entry_cycles=" << calls.load()
            << " activate=1 deactivate=1 update_overlap=0\n";
}
INSTANTIATE_TEST_SUITE_P(TwoAndThree, SnapshotStress, ::testing::Values(2U, 3U));
}  // namespace
}  // namespace bxi_hardware
