#include <gtest/gtest.h>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <new>
#include "controller_interface/controller_interface_params.hpp"
#include "pluginlib/class_loader.hpp"
#include "bxi_hardware/n_axis_trajectory_controller.hpp"

namespace {thread_local bool allocations_enabled = false; thread_local size_t allocations = 0;}
[[gnu::noinline]] void * operator new(size_t n) {
  if (allocations_enabled) {++allocations;}
  if (auto * p = std::malloc(n ? n : 1)) {return p;} throw std::bad_alloc();
}
[[gnu::noinline]] void operator delete(void * p) noexcept {std::free(p);}
[[gnu::noinline]] void operator delete(void * p, size_t) noexcept {std::free(p);}
namespace bxi_hardware {
struct AtomicSnapshotTestAccess {
  template<class T> static void pause(AtomicSnapshot<T> & s) {s.version_.store(s.version_.load()+1);}
};
struct NAxisTestAccess {
  static void generation(NAxisTrajectoryController & c, uint64_t g) {c.generation_ = g; c.previous_complete_ = true;}
  static auto & inputMutex(NAxisTrajectoryController & c) {return c.input_mutex_;}
  static auto & hearts(NAxisTrajectoryController & c) {return c.hearts_;}
  static void elapsed(NAxisTrajectoryController & c, double t) {c.elapsed_ = t;}
  static void invalidTrajectory(NAxisTrajectoryController & c) {c.run_.amplitude = 100.; c.sine_.emplace(0., 100., .2, 5., 1.);}
};
namespace {
using C = NAxisTrajectoryController;
using CB = controller_interface::CallbackReturn;
using Ret = controller_interface::return_type;
constexpr double nan = std::numeric_limits<double>::quiet_NaN();
class TestController : public C {
public:
  int64_t now = 1000000000;
  int bad_set = -1, bad_read = -1, bad_state = -1, bad_read_kind = 0;
  int remaining = 1;
  size_t sets = 0, reads = 0, state_reads = 0;
protected:
  int64_t steadyNow() const noexcept override {return now;}
  bool setCommand(size_t i, double v) override {
    ++sets;
    if (static_cast<int>(i) == bad_set && remaining) {--remaining; return false;}
    return C::setCommand(i, v);
  }
  std::optional<double> getCommand(size_t i) override {
    ++reads;
    if (static_cast<int>(i) == bad_read && remaining) {
      --remaining;
      if (bad_read_kind == 0) {return std::nullopt;}
      if (bad_read_kind == 1) {return 12345.;}
      return nan;
    }
    return C::getCommand(i);
  }
  std::optional<double> getState(size_t i) override {
    ++state_reads;
    if (static_cast<int>(i) == bad_state) {return std::nullopt;}
    return C::getState(i);
  }
};
struct Rig {
  std::string output_dir;
  size_t n;
  std::array<double,16> commands{};
  std::array<double,10> states{};
  std::vector<hardware_interface::CommandInterface> ci;
  std::vector<hardware_interface::StateInterface> si;
  // Controller must die before loans' backing interfaces.
  TestController c;
  explicit Rig(size_t axes, double seconds = 1., int mutation = -1, bool state_mutation = false,
    double initial = 0., unsigned tries = 1) : n(axes) {
    std::string pattern = (std::filesystem::temp_directory_path() / "bxi_g14_XXXXXX").string();
    const auto directory = ::mkdtemp(pattern.data());
    if (!directory) {throw std::runtime_error("cannot create fixture output_dir");}
    output_dir = directory;
    rclcpp::NodeOptions options;
    std::vector<std::string> names;
    for (size_t a=0;a<n;++a) {names.push_back("joint"+std::to_string(a+1));}
    options.parameter_overrides({rclcpp::Parameter("output_dir", output_dir), rclcpp::Parameter("joints", names), rclcpp::Parameter("record_seconds",seconds),
      rclcpp::Parameter("api_max_tries", static_cast<int>(tries))});
    controller_interface::ControllerInterfaceParams p; p.controller_name="g_unit"; p.update_rate=1000; p.node_options=options;
    EXPECT_EQ(c.init(p),Ret::OK); EXPECT_EQ(c.on_configure(rclcpp_lifecycle::State()),CB::SUCCESS);
    const auto cmd = c.command_interface_configuration().names;
    const auto st = c.state_interface_configuration().names;
    ci.reserve(20); si.reserve(12);
    auto mutate = [&](std::vector<std::string> keys, bool selected) {
      if (!selected || mutation < 0) {return keys;}
      if (mutation == 0) {keys.pop_back();}
      if (mutation == 1) {keys.push_back("extra/value");}
      if (mutation == 2) {keys[0]=keys[1];}
      if (mutation == 3) {keys[0]="alias/position";}
      if (mutation == 5) {std::reverse(keys.begin(),keys.end());}
      return keys;
    };
    const auto ck=mutate(cmd,!state_mutation), sk=mutate(st,state_mutation);
    std::vector<hardware_interface::LoanedCommandInterface> cl;
    for (size_t i=0;i<ck.size();++i) {
      const auto split=ck[i].find('/');
      const auto logical=std::find(cmd.begin(),cmd.end(),ck[i])-cmd.begin();
      if (mutation==4 && !state_mutation && i==0) {ci.emplace_back(ck[i].substr(0,split),ck[i].substr(split+1),"bool");}
      else {ci.emplace_back(ck[i].substr(0,split),ck[i].substr(split+1),&commands[std::min<size_t>(logical,15)]);}
      cl.emplace_back(ci.back());
    }
    states[3*n]=initial;
    std::vector<hardware_interface::LoanedStateInterface> sl;
    for (size_t i=0;i<sk.size();++i) {
      const auto split=sk[i].find('/'); const auto logical=std::find(st.begin(),st.end(),sk[i])-st.begin();
      if (mutation==4 && state_mutation && i==0) {si.emplace_back(sk[i].substr(0,split),sk[i].substr(split+1),"bool");}
      else {si.emplace_back(sk[i].substr(0,split),sk[i].substr(split+1),&states[std::min<size_t>(logical,9)]);}
      sl.emplace_back(si.back());
    }
    c.assign_interfaces(std::move(cl),std::move(sl));
  }
  ~Rig() {std::filesystem::remove_all(output_dir);}
  CB activate() {return c.on_activate(rclcpp_lifecycle::State());}
  Ret step(int64_t ns=1000000) {c.now+=ns; return c.update(rclcpp::Time(c.now),rclcpp::Duration::from_nanoseconds(ns));}
  void ack() {states[3*n]=commands[5*n];}
  C::Request request() {C::Request r; r.axes=n; r.nonce=123; r.id=7; r.sent_ns=c.now; return r;}
  bool heart(uint64_t seq) {return c.keepalive({123,7,seq,c.now});}
  void stop() {c.on_deactivate(rclcpp_lifecycle::State());}
  const C::Row & row() {return c.finalRowAfterStop();} // only inspected with no concurrent update
};
class GController : public ::testing::TestWithParam<size_t> {
public:
  static void SetUpTestSuite() {rclcpp::init(0,nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};
TEST_P(GController, BootIdleCommitExactClaimsAndTerminal) {
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
    "controller_interface", "controller_interface::ControllerInterface");
  ASSERT_TRUE(loader.isClassAvailable("bxi_hardware/NAxisTrajectoryController"));
  auto plugin = loader.createSharedInstance("bxi_hardware/NAxisTrajectoryController");
  EXPECT_NE(dynamic_cast<C *>(plugin.get()), nullptr);
  Rig r(GetParam()); ASSERT_EQ(r.activate(),CB::SUCCESS);
  EXPECT_EQ(r.c.command_interface_configuration().names.size(),5*r.n+1);
  EXPECT_EQ(r.c.state_interface_configuration().names.size(),3*r.n+1);
  EXPECT_EQ(r.step(),Ret::OK); EXPECT_EQ(r.commands[5*r.n],1.);
  for (size_t i=0;i<5*r.n;++i) {EXPECT_EQ(r.commands[i],0.);}
  r.ack(); EXPECT_EQ(r.step(),Ret::OK); EXPECT_EQ(r.commands[5*r.n],2.);
  EXPECT_TRUE(r.row().advanced); r.stop(); EXPECT_EQ(r.activate(),CB::ERROR);
}
TEST_P(GController, SelfCheckAllMalformedCommandAndStateSets) {
  for (bool state : {false,true}) {
    for (int fault=0;fault<5;++fault) {Rig r(GetParam(),1.,fault,state); EXPECT_EQ(r.activate(),CB::ERROR);}
  }
  for (bool state : {false,true}) {Rig r(GetParam(),1.,5,state); EXPECT_EQ(r.activate(),CB::SUCCESS);}
}
TEST_P(GController, InitialAcceptedAndUnavailableAreRejected) {
  for (double value : {nan, .5, 1., -1., std::numeric_limits<double>::infinity()}) {
    Rig r(GetParam(),1.,-1,false,value); EXPECT_EQ(r.activate(),CB::ERROR);
  }
  Rig r(GetParam()); r.c.bad_state=3*r.n; EXPECT_EQ(r.activate(),CB::ERROR);
}
TEST_P(GController, SupplementDelayedMissingAndNonfiniteAcceptedHoldLatestValues) {
  Rig r(GetParam()); ASSERT_EQ(r.activate(),CB::SUCCESS); ASSERT_EQ(r.step(),Ret::OK);
  for (int cycle=0;cycle<20;++cycle) {
    if (cycle==0) {r.c.bad_state=3*r.n;}
    if (cycle==1) {r.c.bad_state=-1; r.states[3*r.n]=nan;}
    if (cycle==2) {r.states[3*r.n]=100.;}
    if (cycle==3) {r.states[3*r.n]=0.;}
    r.states[0]=.001*cycle;
    EXPECT_EQ(r.step(),Ret::OK); EXPECT_EQ(r.commands[5*r.n],1.);
    EXPECT_EQ(r.commands[0],r.states[0]); EXPECT_TRUE(r.row().held);
    EXPECT_EQ(r.row().max_holds,static_cast<uint64_t>(cycle+1));
  }
  r.ack(); EXPECT_EQ(r.step(),Ret::OK); EXPECT_EQ(r.commands[5*r.n],2.);
  EXPECT_EQ(r.row().max_holds,20U); r.stop();
}
TEST_P(GController, EverySetterAndReadbackFaultMakesOneReleaseWithSameGeneration) {
  const size_t n=GetParam();
  for (size_t item=0;item<5*n+1;++item) {
    for (int mode=0;mode<4;++mode) {
      Rig r(n); ASSERT_EQ(r.activate(),CB::SUCCESS);
      if (mode==0) {r.c.bad_set=item;} else {r.c.bad_read=item; r.c.bad_read_kind=mode-1;}
      EXPECT_EQ(r.step(),Ret::ERROR); const auto row=r.row();
      EXPECT_EQ(row.generation,1U); EXPECT_TRUE(row.released_attempted);
      EXPECT_EQ(row.released.read_ok,(1U<<(5*n+1))-1);
      if (item<5*n) {EXPECT_EQ(row.normal.write_attempted & (1U<<(5*n)),0U);}
      const auto calls=r.c.sets+r.c.reads+r.c.state_reads-1; // activation accepted getter
      EXPECT_LE(calls,C::apiBudget(n).calls);
      const auto total=r.c.sets; EXPECT_EQ(r.step(),Ret::ERROR); EXPECT_EQ(r.c.sets,total);
      EXPECT_EQ(row.failures,1U); EXPECT_GE(row.error_ns,row.released.commit_read_ns);
      EXPECT_FALSE(r.c.requestStart(r.request())); r.stop();
    }
  }
}
TEST_P(GController, NoPartialReleaseIfPositionMissingAndBadPeriodIsTerminal) {
  for (size_t a=0;a<GetParam();++a) {
    for (bool missing : {false,true}) {
      Rig r(GetParam()); ASSERT_EQ(r.activate(),CB::SUCCESS);
      if (missing) {r.c.bad_state=3*a;} else {r.states[3*a]=nan;}
      EXPECT_EQ(r.step(),Ret::ERROR); EXPECT_FALSE(r.row().released_attempted); EXPECT_EQ(r.c.sets,0U);
    }
  }
  Rig r(GetParam()); ASSERT_EQ(r.activate(),CB::SUCCESS);
  EXPECT_EQ(r.step(5000001),Ret::ERROR); EXPECT_TRUE(r.row().released_attempted);
}
TEST_P(GController, LeaseRequestValidationAndNanosecondBoundaries) {
  EXPECT_FALSE(C::leaseExpired(1000000000+499999999,1000000000));
  EXPECT_TRUE(C::leaseExpired(1500000000,1000000000));
  EXPECT_TRUE(C::leaseExpired(1500000001,1000000000));
  Rig r(GetParam()); ASSERT_EQ(r.activate(),CB::SUCCESS); ASSERT_EQ(r.step(),Ret::OK);
  auto request=r.request(); auto bad=request; bad.amplitude=nan; EXPECT_FALSE(r.c.requestStart(bad));
  bad=request; bad.frequency=.21; EXPECT_FALSE(r.c.requestStart(bad));
  bad=request; bad.kp[r.n-1]=21.; EXPECT_FALSE(r.c.requestStart(bad));
  bad=request; bad.sent_ns=r.c.now+1; EXPECT_FALSE(r.c.requestStart(bad));
  bad=request; bad.sent_ns=r.c.now-C::kLeaseNs; EXPECT_FALSE(r.c.requestStart(bad));
  bad=request; bad.axes=1; EXPECT_FALSE(r.c.requestStart(bad));
  ASSERT_TRUE(r.c.requestStart(request));
  EXPECT_FALSE(r.c.keepalive({124,7,1,r.c.now})); EXPECT_FALSE(r.c.keepalive({123,8,1,r.c.now}));
  EXPECT_FALSE(r.c.keepalive({123,7,1,r.c.now+1})); ASSERT_TRUE(r.heart(1));
  EXPECT_FALSE(r.heart(1));
  r.ack(); ASSERT_EQ(r.step(),Ret::OK); EXPECT_EQ(r.row().after,C::State::kRunning);
  const auto sent=r.row().heartbeat_sent_ns;
  r.c.now=sent+C::kLeaseNs-1; r.ack();
  EXPECT_EQ(r.c.update(rclcpp::Time(r.c.now),rclcpp::Duration::from_nanoseconds(1000000)),Ret::OK);
  r.c.now=sent+C::kLeaseNs; r.ack();
  EXPECT_EQ(r.c.update(rclcpp::Time(r.c.now),rclcpp::Duration::from_nanoseconds(1000000)),Ret::ERROR);
  EXPECT_EQ(r.row().reason,C::Reason::kLease); EXPECT_TRUE(r.row().released_attempted);
}
TEST_P(GController, ArmedCommitsAndOneSecondTimeout) {
  Rig r(GetParam()); ASSERT_EQ(r.activate(),CB::SUCCESS); ASSERT_EQ(r.step(),Ret::OK);
  ASSERT_TRUE(r.c.requestStart(r.request())); r.ack(); ASSERT_EQ(r.step(),Ret::OK);
  EXPECT_EQ(r.row().after,C::State::kArmed); const auto armed=r.c.now;
  for (int i=0;i<5;++i) {r.ack(); EXPECT_EQ(r.step(),Ret::OK); EXPECT_EQ(r.row().after,C::State::kArmed);}
  r.c.now=armed+C::kArmNs-1; r.ack();
  EXPECT_EQ(r.c.update(rclcpp::Time(r.c.now),rclcpp::Duration::from_nanoseconds(1000000)),Ret::OK);
  r.c.now=armed+C::kArmNs; r.ack();
  EXPECT_EQ(r.c.update(rclcpp::Time(r.c.now),rclcpp::Duration::from_nanoseconds(1000000)),Ret::ERROR);
  EXPECT_EQ(r.row().reason,C::Reason::kArmTimeout);
}
TEST_P(GController, B1StartsAtPositionBoundaryWithoutVelocityOrAcceptedGate) {
  for (double q : {-.05,.05}) {
    Rig r(GetParam()); ASSERT_EQ(r.activate(),CB::SUCCESS); ASSERT_EQ(r.step(),Ret::OK);
    auto request=r.request(); ASSERT_TRUE(r.c.requestStart(request)); ASSERT_TRUE(r.heart(1));
    for (size_t a=0;a<r.n;++a) {r.states[3*a]=q; r.states[3*a+1]=100.;}
    r.c.bad_state=3*r.n; // accepted unavailable is recorded, not a start condition
    EXPECT_EQ(r.step(),Ret::OK); EXPECT_EQ(r.row().after,C::State::kRunning);
    EXPECT_EQ(r.row().generation,1U); EXPECT_EQ(r.row().accepted_result,C::Accepted::kUnavailable);
  }
  for (size_t axis=0;axis<GetParam();++axis) {
    Rig r(GetParam()); ASSERT_EQ(r.activate(),CB::SUCCESS); ASSERT_EQ(r.step(),Ret::OK);
    ASSERT_TRUE(r.c.requestStart(r.request())); ASSERT_TRUE(r.heart(1)); r.states[3*axis]=.050000001;
    EXPECT_EQ(r.step(),Ret::ERROR); EXPECT_EQ(r.row().reason,C::Reason::kStartRange);
    EXPECT_TRUE(r.row().released_attempted);
  }
}
TEST_P(GController, HoldingLeaseExpiryAndStopPriority) {
  for (bool stop : {false,true}) {
    Rig r(GetParam()); ASSERT_EQ(r.activate(),CB::SUCCESS); ASSERT_EQ(r.step(),Ret::OK);
    auto req=r.request(); req.release=false; ASSERT_TRUE(r.c.requestStart(req)); ASSERT_TRUE(r.heart(1));
    r.ack(); ASSERT_EQ(r.step(),Ret::OK); NAxisTestAccess::elapsed(r.c,35.1);
    r.ack(); ASSERT_EQ(r.step(),Ret::OK); ASSERT_EQ(r.row().after,C::State::kHolding);
    if (stop) {
      ASSERT_TRUE(r.c.requestFinish()); r.c.requestStop(); r.ack();
      EXPECT_EQ(r.step(),Ret::OK); EXPECT_EQ(r.row().reason,C::Reason::kStop);
      EXPECT_EQ(r.row().after,C::State::kStopped);
    } else {
      r.c.now=r.row().deadline_ns; r.ack();
      EXPECT_EQ(r.c.update(rclcpp::Time(r.c.now),rclcpp::Duration::from_nanoseconds(1000000)),Ret::ERROR);
      EXPECT_EQ(r.row().reason,C::Reason::kLease);
    }
  }
}
TEST_P(GController, TrajectoryProtectionAndSameCandidateReleaseWhileHeld) {
  for (bool torque : {false,true}) {
    Rig r(GetParam()); ASSERT_EQ(r.activate(),CB::SUCCESS); ASSERT_EQ(r.step(),Ret::OK);
    ASSERT_TRUE(r.c.requestStart(r.request())); ASSERT_TRUE(r.heart(1));
    ASSERT_EQ(r.step(),Ret::OK); EXPECT_TRUE(r.row().held);
    if (torque) {r.states[0]=.30;} else {NAxisTestAccess::invalidTrajectory(r.c); NAxisTestAccess::elapsed(r.c,1.);}
    EXPECT_EQ(r.step(),Ret::ERROR);
    EXPECT_EQ(r.row().reason,torque?C::Reason::kTorque:C::Reason::kTrajectory);
    EXPECT_EQ(r.row().generation,1U); EXPECT_TRUE(r.row().released_attempted);
    EXPECT_EQ(r.row().released.values[5*r.n],1.);
  }
}
TEST_P(GController, GenerationUpperLimitNeverWrapsOrUsesSpecialReleaseGeneration) {
  Rig r(GetParam()); ASSERT_EQ(r.activate(),CB::SUCCESS);
  NAxisTestAccess::generation(r.c,C::kMaxGeneration-1); r.states[3*r.n]=C::kMaxGeneration-1;
  ASSERT_EQ(r.step(),Ret::OK); EXPECT_EQ(r.commands[5*r.n],static_cast<double>(C::kMaxGeneration));
  r.ack(); auto count=r.c.sets; EXPECT_EQ(r.step(),Ret::OK);
  EXPECT_EQ(r.c.sets,count); EXPECT_EQ(r.row().reason,C::Reason::kUpperLimit);
  EXPECT_FALSE(r.row().released_attempted); EXPECT_EQ(r.row().error_ns,0);
}
TEST_P(GController, CapacitySaveFailuresMissingOriginalAndTiming) {
  for (double seconds : {0.,.001}) {
    Rig r(GetParam(),seconds); ASSERT_EQ(r.activate(),CB::SUCCESS);
    if (seconds>0) {ASSERT_EQ(r.step(),Ret::OK); r.ack();}
    EXPECT_EQ(r.step(),Ret::ERROR); EXPECT_EQ(r.row().reason,C::Reason::kCapacity);
    EXPECT_EQ(r.c.droppedAfterStop(),1U);
    const auto path=::testing::TempDir()+"g_capacity.csv";
    EXPECT_FALSE(r.c.saveRecord(path)); EXPECT_FALSE(r.c.requestStart(r.request()));
  }
  Rig r(GetParam()); ASSERT_EQ(r.activate(),CB::SUCCESS); ASSERT_EQ(r.step(),Ret::OK);
  const auto path=::testing::TempDir()+"g_missing_"+std::to_string(GetParam())+".csv";
  std::filesystem::remove(path); EXPECT_FALSE(std::filesystem::exists(path));
  EXPECT_FALSE(r.c.saveRecord(path)); // incomplete/active is not a completed original
  EXPECT_FALSE(std::filesystem::exists(path)); r.stop();
  EXPECT_TRUE(r.c.saveRecord(path)); EXPECT_TRUE(std::filesystem::exists(path));
  EXPECT_FALSE(r.c.saveRecord("/unavailable/g.csv"));
  const auto timing=r.c.timingAfterStop();
  EXPECT_GT(timing.configure_in,0); EXPECT_GE(timing.configure_out,timing.configure_in);
  EXPECT_GE(timing.activate_out,timing.activate_in); EXPECT_GT(timing.first_update,0);
  std::cout << "N=" << GetParam() << " row_bytes=" << sizeof(C::Row) << " 120000_rows=" << 120000*sizeof(C::Row) << std::endl;
}
TEST_P(GController, NoRtAllocationOrInputMutexWaitAndApiBudget) {
  Rig r(GetParam()); ASSERT_EQ(r.activate(),CB::SUCCESS); ASSERT_EQ(r.step(),Ret::OK);
  ASSERT_TRUE(r.c.requestStart(r.request())); ASSERT_TRUE(r.heart(1)); r.ack();
  const auto before=r.c.sets+r.c.reads+r.c.state_reads;
  std::unique_lock<std::mutex> held(NAxisTestAccess::inputMutex(r.c));
  allocations=0; allocations_enabled=true; const auto result=r.step(); allocations_enabled=false;
  EXPECT_EQ(result,Ret::OK); EXPECT_EQ(allocations,0U);
  EXPECT_LE(r.c.sets+r.c.reads+r.c.state_reads-before,C::apiBudget(r.n).calls);
  const auto one=C::apiBudget(r.n); EXPECT_EQ(one.calls, r.n==2?51U:74U);
  EXPECT_EQ(one.attempts,one.calls); EXPECT_EQ(one.yields,r.n==2?29U:42U);
  const auto ten=C::apiBudget(r.n,10); EXPECT_EQ(ten.attempts,r.n==2?510U:740U);
  EXPECT_EQ(ten.yields,r.n==2?488U:708U);
}
TEST_P(GController, FullRunIndependentAnalyticReferenceAndNormalReleaseKeepsCommitting) {
  Rig r(GetParam(),40.); ASSERT_EQ(r.activate(),CB::SUCCESS); ASSERT_EQ(r.step(),Ret::OK);
  ASSERT_TRUE(r.c.requestStart(r.request())); ASSERT_TRUE(r.heart(1));
  uint64_t heartbeat=1;
  for (int tick=0;tick<36100;++tick) {
    if (tick && tick%40==0 && tick<36000) {ASSERT_TRUE(r.heart(++heartbeat));}
    r.ack(); ASSERT_EQ(r.step(),Ret::OK);
    const auto & row=r.row();
    if (row.after==C::State::kRunning || row.after==C::State::kHolding) {
      const double t=std::min(row.elapsed,35.); double e=1.,de=0.,dde=0.;
      if (t<5.) {e=(1.-std::cos(C::kPi*t/5.))/2.;de=C::kPi/10.*std::sin(C::kPi*t/5.);dde=C::kPi*C::kPi/50.*std::cos(C::kPi*t/5.);}
      else if (t>30.) {e=(1.+std::cos(C::kPi*(t-30.)/5.))/2.;de=-C::kPi/10.*std::sin(C::kPi*(t-30.)/5.);dde=-C::kPi*C::kPi/50.*std::cos(C::kPi*(t-30.)/5.);}
      const double w=2.*C::kPi*.2,A=C::kPi/6.;
      const double q=A*e*std::sin(w*t),v=A*(de*std::sin(w*t)+e*w*std::cos(w*t));
      const double a=A*(dde*std::sin(w*t)+2*w*de*std::cos(w*t)-w*w*e*std::sin(w*t));
      for (size_t axis=0;axis<r.n;++axis) {
        EXPECT_NEAR(r.commands[5*axis],q,1e-12);
        EXPECT_NEAR(r.commands[5*axis+1],row.after==C::State::kHolding?0.:v,1e-12);
        // Track the command in the fake state; no physical dynamics are modeled.
        r.states[3*axis]=q;
      }
      EXPECT_NEAR(row.acceleration,a,1e-12);
    }
  }
  EXPECT_EQ(r.row().after,C::State::kReleased); EXPECT_EQ(r.row().reason,C::Reason::kRelease);
  const auto g=r.commands[5*r.n]; r.ack(); EXPECT_EQ(r.step(),Ret::OK); EXPECT_EQ(r.commands[5*r.n],g+1.);
  r.stop(); const char * dir=std::getenv("BXI_N_AXIS_OUTPUT_DIR");
  const auto path=(dir?std::string(dir): ::testing::TempDir())+"/controller_n"+std::to_string(r.n)+".csv";
  std::filesystem::create_directories(std::filesystem::path(path).parent_path()); EXPECT_TRUE(r.c.saveRecord(path));
}
TEST_P(GController, MultipleAndRepeatedWriteFailuresStillHaveOnlyOneRelease) {
  Rig r(GetParam()); ASSERT_EQ(r.activate(),CB::SUCCESS);
  r.c.bad_set=0; r.c.bad_read=5*r.n-1; r.c.remaining=3;
  EXPECT_EQ(r.step(),Ret::ERROR); const auto row=r.row();
  EXPECT_TRUE(row.released_attempted); EXPECT_NE(row.normal.write_attempted,0U);
  EXPECT_EQ(row.normal.write_attempted & (1U<<(5*r.n)),0U);
  EXPECT_EQ(row.released.write_attempted & (1U<<(5*r.n)),0U);
  const auto sets=r.c.sets;
  for (int i=0;i<5;++i) {EXPECT_EQ(r.step(),Ret::ERROR);}
  EXPECT_EQ(r.c.sets,sets); EXPECT_EQ(r.row().failures,1U);
}
TEST_P(GController, FinishIsNormalAndContinuesReleasedCommits) {
  Rig r(GetParam()); ASSERT_EQ(r.activate(),CB::SUCCESS); ASSERT_EQ(r.step(),Ret::OK);
  auto request=r.request(); request.release=false;
  ASSERT_TRUE(r.c.requestStart(request)); ASSERT_TRUE(r.heart(1)); r.ack(); ASSERT_EQ(r.step(),Ret::OK);
  NAxisTestAccess::elapsed(r.c,35.1); r.ack(); ASSERT_EQ(r.step(),Ret::OK);
  ASSERT_TRUE(r.c.requestFinish()); r.ack(); EXPECT_EQ(r.step(),Ret::OK);
  EXPECT_EQ(r.row().after,C::State::kReleased); EXPECT_EQ(r.row().reason,C::Reason::kFinish);
  const auto g=r.commands[5*r.n]; r.ack(); EXPECT_EQ(r.step(),Ret::OK);
  EXPECT_EQ(r.commands[5*r.n],g+1.); EXPECT_EQ(r.row().error_ns,0);
}
TEST_P(GController, FailedHeartbeatSnapshotDoesNotExtendSenderDeadline) {
  Rig r(GetParam()); ASSERT_EQ(r.activate(),CB::SUCCESS); ASSERT_EQ(r.step(),Ret::OK);
  ASSERT_TRUE(r.c.requestStart(r.request())); ASSERT_TRUE(r.heart(1)); r.ack(); ASSERT_EQ(r.step(),Ret::OK);
  const auto deadline=r.row().deadline_ns;
  r.c.now+=100000000; ASSERT_TRUE(r.heart(2));
  EXPECT_FALSE(r.c.keepalive({123,7,1,r.c.now}));
  EXPECT_FALSE(r.c.keepalive({123,7,3,r.c.now-1}));
  AtomicSnapshotTestAccess::pause(NAxisTestAccess::hearts(r.c)); r.ack(); EXPECT_EQ(r.step(),Ret::OK);
  EXPECT_EQ(r.row().deadline_ns,deadline);
  r.c.now=deadline; r.ack();
  EXPECT_EQ(r.c.update(rclcpp::Time(r.c.now),rclcpp::Duration::from_nanoseconds(1000000)),Ret::ERROR);
  EXPECT_EQ(r.row().reason,C::Reason::kLease);
}
TEST_P(GController, AllAxisStateFailuresAndInitialPositionUseCurrentValuesOnly) {
  for (size_t i=0;i<3*GetParam();++i) {
    Rig r(GetParam()); ASSERT_EQ(r.activate(),CB::SUCCESS); ASSERT_EQ(r.step(),Ret::OK);
    r.states[i]=nan; EXPECT_EQ(r.step(),Ret::ERROR); EXPECT_EQ(r.row().reason,C::Reason::kState);
    EXPECT_EQ(r.row().released_attempted,i%3!=0);
  }
  Rig r(GetParam()); r.states[0]=.02; ASSERT_EQ(r.activate(),CB::SUCCESS);
  EXPECT_EQ(r.step(),Ret::OK); EXPECT_EQ(r.commands[0],.02);
}
INSTANTIATE_TEST_SUITE_P(TwoAndThree,GController,::testing::Values(2U,3U));
} // namespace
} // namespace bxi_hardware
