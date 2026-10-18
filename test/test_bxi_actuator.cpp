#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>
#include <thread>

#include <gtest/gtest.h>

#include "bxi_hardware/actuator/bxi_actuator.hpp"
#include "bxi_hardware/activation_startup.hpp"

namespace bxi_hardware
{
namespace
{

class EmulatedBus : public CanBus
{
public:
  bool open(const std::string & ifname) override {name_ = ifname; open_ = true; return true;}
  void close() override {open_ = false;}
  bool isOpen() const override {return open_;}
  const std::string & name() const override {return name_;}
  void flushRx() override {responses_.clear();}

  bool send(const CanFrame & frame) override
  {
    sent_.push_back(frame);
    if (respond_) {
      responses_.push_back(feedbackFrame(static_cast<uint8_t>(frame.id)));
    }
    return open_;
  }

  bool recv(CanFrame & out, int) override
  {
    if (unrelated_remaining_ > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      --unrelated_remaining_;
      ++unrelated_delivered_;
      out = feedbackFrame(2);
      return true;
    }
    if (responses_.empty()) {
      return false;
    }
    out = responses_.front();
    out.rx_kernel_realtime_ns = kernel_realtime_ns_;
    if (bad_dlc_) {out.dlc = 7;}
    responses_.pop_front();
    return true;
  }

  CanFrame feedbackFrame(uint8_t motor_id) const
  {
    CanFrame frame;
    frame.id = protocol::defaultMasterId(motor_id);
    frame.extended = false;
    frame.dlc = 8;
    frame.data = {{motor_id, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, ntc1_raw_, ntc2_raw_}};
    return frame;
  }

  bool respond_{true};
  bool bad_dlc_{false};
  int64_t kernel_realtime_ns_{0};
  int unrelated_remaining_{0};
  int unrelated_delivered_{0};
  uint8_t ntc1_raw_{0x55};  // 30 degC
  uint8_t ntc2_raw_{0x5A};  // about 33.5 degC
  std::vector<CanFrame> sent_;

private:
  bool open_{false};
  std::string name_;
  std::deque<CanFrame> responses_;
};

BxiActuatorDriver makeDriver(EmulatedBus & bus, uint32_t maximum_timeouts = 3)
{
  BxiSpec spec;
  spec.t_min = -80.0;
  spec.t_max = 80.0;
  return BxiActuatorDriver(&bus, 1, 1, 0.0, -1.0, 1.0, 20.0, 1.0, spec, 1,
    maximum_timeouts);
}

class StartupChatterBus : public EmulatedBus
{
public:
  unsigned controls = 0;
  std::deque<CanFrame> chatter;
  bool send(const CanFrame & frame) override
  {
    const bool special = frame.data8() == protocol::encodeSpecialCommand(protocol::SpecialCommand::kEnable) ||
      frame.data8() == protocol::encodeSpecialCommand(protocol::SpecialCommand::kDisable);
    if (!special && ++controls == 1) {
      sent_.push_back(frame);
      // Three interleaved mode-text IDs plus another motor's binary reply.
      for (uint32_t id : {0x7F1U, 0x7F2U, 0x7F3U, 0x12U}) {
        auto other = feedbackFrame(2); other.id = id;
        chatter.push_back(other);
      }
      return true;
    }
    return EmulatedBus::send(frame);
  }
  bool recv(CanFrame & out, int timeout) override
  {
    if (!chatter.empty()) {out = chatter.front(); chatter.pop_front(); return true;}
    return EmulatedBus::recv(out, timeout);
  }
};

TEST(ActivationClassification, ChatterTimeoutRetriesAndSucceeds)
{
  StartupChatterBus bus; ASSERT_TRUE(bus.open("fake"));
  BxiActuatorDriver driver(&bus, 1, 1, 0, -1, 1, 0, 0, BxiSpec{}, 10, 3);
  ASSERT_TRUE(driver.enable().valid);
  // Only policy waits/deadlines are injected; driver timeout clock is real and empty recv immediate.
  int64_t now = 1000000000;
  ActivationStartup policy{{[&]() {return now;}, [&](int64_t until) {now = until;}},
    now + ActivationPolicy::budget_ns, 10000000};
  ASSERT_TRUE(policy.settle(now));
  std::vector<InitialAttempt> records; ActuatorFeedback feedback;
  EXPECT_TRUE(policy.initial([&](TransactionObservation & o) {return driver.initialControlObserved(o);},
    [&]() {return driver.state() == ActuatorState::kEnabled;}, feedback, records));
  EXPECT_EQ(bus.controls, 2U);
  ASSERT_FALSE(records.empty());
  EXPECT_STREQ(records[0].observation.result, "timeout");
  EXPECT_EQ(records[0].observation.replies.size(), 4U);
}

// Driver clocks are real, not injected. Only order is asserted; fake kernel stamps are injected.
TEST(ActivationObservation, RecordsOkTimeoutInvalidSendFailureAndSafeStop)
{
  EmulatedBus bus; ASSERT_TRUE(bus.open("fake"));
  auto driver = makeDriver(bus, 3);
  TransactionObservation observation;
  ASSERT_TRUE(driver.enableObserved(observation).valid);
  EXPECT_STREQ(observation.result, "ok");
  ASSERT_EQ(observation.replies.size(), 1U);
  EXPECT_GT(observation.tx_call_realtime_ns, 0);
  EXPECT_LE(observation.tx_call_steady_ns, observation.replies[0].rx_dequeue_steady_ns);
  EXPECT_EQ(observation.replies[0].rx_kernel_realtime_ns, 0);
  bus.kernel_realtime_ns_ = 1234;
  ASSERT_TRUE(driver.initialControlObserved(observation).valid);
  EXPECT_EQ(observation.replies[0].rx_kernel_realtime_ns, 1234);
  bus.respond_ = false;
  EXPECT_FALSE(driver.initialControlObserved(observation).valid);
  EXPECT_STREQ(observation.result, "timeout");
  EXPECT_TRUE(observation.replies.empty());
  bus.respond_ = true; bus.bad_dlc_ = true;
  EXPECT_FALSE(driver.initialControlObserved(observation).valid);
  EXPECT_STREQ(observation.result, "timeout");
  ASSERT_EQ(observation.replies.size(), 1U);
  EXPECT_FALSE(observation.replies[0].accepted);
  EXPECT_EQ(observation.replies[0].dlc, 7);
  bus.bad_dlc_ = false; bus.close();
  EXPECT_FALSE(driver.initialControlObserved(observation).valid);
  EXPECT_STREQ(observation.result, "send_failed");
  EXPECT_EQ(driver.state(), ActuatorState::kSafeStopped);
  const auto count = bus.sent_.size();
  EXPECT_FALSE(driver.initialControlObserved(observation).valid);
  EXPECT_STREQ(observation.result, "safe_stop");
  EXPECT_EQ(observation.tx_call_steady_ns, 0);
  EXPECT_EQ(bus.sent_.size(), count);
}

TEST(ActivationObservation, ObservationDoesNotChangeSuccessfulFrames)
{
  EmulatedBus plain, observed;
  ASSERT_TRUE(plain.open("fake")); ASSERT_TRUE(observed.open("fake"));
  auto a = makeDriver(plain); auto b = makeDriver(observed);
  TransactionObservation observation;
  ASSERT_TRUE(a.enable().valid); ASSERT_TRUE(b.enableObserved(observation).valid);
  ASSERT_TRUE(a.sendControl(0, 0, 0, 0, 0).valid);
  ASSERT_TRUE(b.initialControlObserved(observation).valid);
  ASSERT_TRUE(a.disable().valid); ASSERT_TRUE(b.disable().valid);
  ASSERT_EQ(plain.sent_.size(), observed.sent_.size());
  for (size_t i = 0; i < plain.sent_.size(); ++i) {
    EXPECT_EQ(plain.sent_[i].id, observed.sent_[i].id);
    EXPECT_EQ(plain.sent_[i].data, observed.sent_[i].data);
    EXPECT_EQ(plain.sent_[i].dlc, observed.sent_[i].dlc);
  }
}

TEST(ActivationObservation, DriverJudgmentMatchesWithoutObservation)
{
  for (int mode = 0; mode < 5; ++mode) {
    EmulatedBus plain, observed;
    ASSERT_TRUE(plain.open("fake")); ASSERT_TRUE(observed.open("fake"));
    auto a = makeDriver(plain, 1); auto b = makeDriver(observed, 1);
    ASSERT_TRUE(a.enable().valid); ASSERT_TRUE(b.enable().valid);
    if (mode == 1) {plain.respond_ = observed.respond_ = false;}
    if (mode == 2) {plain.bad_dlc_ = observed.bad_dlc_ = true;}
    if (mode == 3) {plain.ntc1_raw_ = observed.ntc1_raw_ = 0; plain.ntc2_raw_ = observed.ntc2_raw_ = 0;}
    if (mode == 4) {plain.close(); observed.close();}
    TransactionObservation o;
    const auto x = a.sendControl(0, 0, 0, 0, 0), y = b.initialControlObserved(o);
    EXPECT_EQ(x.valid, y.valid); EXPECT_EQ(x.consecutive_timeouts, y.consecutive_timeouts);
    EXPECT_EQ(a.consecutiveTimeouts(), b.consecutiveTimeouts()); EXPECT_EQ(a.state(), b.state());
    EXPECT_EQ(plain.sent_.size(), observed.sent_.size());
    EXPECT_STREQ(o.result, mode == 0 ? "ok" : mode == 3 ? "temperature_fault" :
      mode == 4 ? "send_failed" : "safe_stop");
  }
}

TEST(ActivationObservation, DelayedFeedbackBeforeAfterRetryAndUnknownKernelTime)
{
  // Arrival stamps are injected. Driver send/dequeue clocks are real; order only is checked.
  for (int relation : {-1, 0, 1}) {
    EmulatedBus bus; ASSERT_TRUE(bus.open("fake"));
    auto driver = makeDriver(bus); ASSERT_TRUE(driver.enable().valid);
    bus.respond_ = false; TransactionObservation first, second;
    ASSERT_FALSE(driver.initialControlObserved(first).valid);
    // Inject the delayed frame's kernel stamp; no request tag exists in the protocol.
    const auto rt = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    bus.kernel_realtime_ns_ = relation == 0 ? 0 :
      rt + (relation < 0 ? -1000000000LL : 1000000000LL);
    bus.respond_ = true;
    ASSERT_TRUE(driver.initialControlObserved(second).valid);
    ASSERT_EQ(second.replies.size(), 1U);
    EXPECT_LE(second.tx_call_steady_ns, second.replies[0].rx_dequeue_steady_ns);
    EXPECT_EQ(second.replies[0].rx_kernel_realtime_ns, bus.kernel_realtime_ns_);
    EXPECT_STREQ(kernelVsSend(second.replies[0].rx_kernel_realtime_ns, second.tx_call_realtime_ns),
      relation == 0 ? "NOT_EVALUATED" : relation < 0 ? "before" : "at_or_after");
  }
}

TEST(ActivationPolicy, InjectedWindowsAndSafeStopLimits)
{
  for (int window : {0, 1, 4, 10, 20, 101, 10000}) {
    for (int stop_limit : {1, 2, 3}) {
      int64_t now = 1000000000, enable = now;
      ActivationStartup policy{{[&]() {return now;}, [&](int64_t until) {now = until;}},
        enable + ActivationPolicy::budget_ns, 10000000};
      ASSERT_TRUE(policy.settle(enable));
      bool enabled = true; unsigned sent = 0, timeouts = 0;
      ActuatorFeedback feedback; std::vector<InitialAttempt> records;
      const bool ok = policy.initial([&](TransactionObservation & o) {
        ++sent; ActuatorFeedback f;
        if (now >= enable + window * 1000000LL) {o.result = "ok"; f.valid = true; f.temperature = 30;}
        else {now += 10000000; o.result = "timeout"; if (++timeouts >= unsigned(stop_limit)) {enabled = false; o.result = "safe_stop";}}
        return f;
      }, [&]() {return enabled;}, feedback, records);
      EXPECT_EQ(ok, window <= 10 || (window == 20 && stop_limit > 1));
      EXPECT_LE(sent, 2U); EXPECT_EQ(sent, records.size());
      EXPECT_EQ(sent, window <= 10 || stop_limit == 1 ? 1U : 2U);
      EXPECT_EQ(policy.deadline, enable + ActivationPolicy::budget_ns);
    }
  }
}

TEST(ActivationPolicy, DeadlinesAndTemperatureFailureNeverRetry)
{
  for (int mode = 0; mode < 6; ++mode) {
    int64_t now = 1000000000, enable = now; unsigned sent = 0;
    ActivationStartup policy{{[&]() {return now;}, [&](int64_t until) {
      now = until + (mode == 1 ? 100000000 : 0);
    }}, now + (mode == 0 ? 15000000 : 100000000), 10000000};
    if (mode < 2) {EXPECT_FALSE(policy.settle(enable)); continue;}
    ASSERT_TRUE(policy.settle(enable));
    if (mode == 2) {now = policy.deadline - 5000000;}
    ActuatorFeedback feedback; std::vector<InitialAttempt> records;
    EXPECT_FALSE(policy.initial([&](TransactionObservation & o) {
      ++sent; ActuatorFeedback f;
      if (mode == 3) {now = policy.deadline; o.result = "ok"; f.valid = true; f.temperature = 30;}
      if (mode == 4) {o.result = "temperature_fault"; f.valid = true; f.temperature = std::numeric_limits<double>::quiet_NaN();}
      if (mode == 5) {o.result = "send_failed";}
      return f;
    }, []() {return true;}, feedback, records));
    EXPECT_EQ(sent, mode == 2 ? 0U : 1U);
  }
}

TEST(ActivationPolicy, PreparationTimeIsSubtractedAndExceptionIsNotRetried)
{
  int64_t enable = 1000000000, now = enable + 7000000;
  ActivationStartup policy{{[&]() {return now;}, [&](int64_t until) {now = until;}},
    enable + ActivationPolicy::budget_ns, 10000000};
  ASSERT_TRUE(policy.settle(enable)); EXPECT_EQ(policy.settle_wait, 3000000);
  ActuatorFeedback f; std::vector<InitialAttempt> records; unsigned sent = 0;
  EXPECT_THROW(policy.initial([&](TransactionObservation &) -> ActuatorFeedback {
    ++sent; throw std::runtime_error("injected send exception");
  }, []() {return true;}, f, records), std::runtime_error);
  EXPECT_EQ(sent, 1U); EXPECT_STREQ(policy.reason, "exception");
}

TEST(BxiActuator, ReproducesActivateCommandFeedbackDeactivateLifecycle)
{
  EmulatedBus bus;
  ASSERT_TRUE(bus.open("emulated0"));
  auto driver = makeDriver(bus);

  EXPECT_TRUE(driver.enable().valid);
  EXPECT_EQ(driver.state(), ActuatorState::kEnabled);
  EXPECT_TRUE(driver.setRunMode());
  const auto feedback = driver.sendControl(0.2, 0.1, 20.0, 1.0, 0.5);
  EXPECT_TRUE(feedback.valid);
  EXPECT_NEAR(feedback.position, 0.0, 0.001);
  EXPECT_NEAR(feedback.temperature, 33.53, 0.01);
  EXPECT_FALSE(driver.mosTemperatureSensorFault());
  EXPECT_FALSE(driver.motorTemperatureSensorFault());
  EXPECT_TRUE(driver.disable().valid);
  EXPECT_EQ(driver.state(), ActuatorState::kDisabled);

  ASSERT_EQ(bus.sent_.size(), 3U);
  EXPECT_EQ(bus.sent_[0].data[7], 0xFC);
  EXPECT_EQ(bus.sent_[2].data[7], 0xFD);
  EXPECT_FALSE(bus.sent_[0].extended);
}

TEST(BxiActuator, ExcludesTemperatureSensorsStuckAtTheBottomOfTheRange)
{
  EmulatedBus bus;
  ASSERT_TRUE(bus.open("emulated0"));
  auto driver = makeDriver(bus);
  bus.ntc2_raw_ = 0x02;  // -28.6 degC: the NTC2 FAIL pattern seen on the HIL motor
  const auto one_fault = driver.enable();
  ASSERT_TRUE(one_fault.valid);
  EXPECT_NEAR(one_fault.temperature, 30.0, 1e-9);
  EXPECT_TRUE(std::isnan(one_fault.motor_temperature));
  EXPECT_TRUE(driver.motorTemperatureSensorFault());
  EXPECT_FALSE(driver.mosTemperatureSensorFault());

  bus.ntc1_raw_ = 0x00;
  const auto both_faults = driver.sendControl(0.0, 0.0, 0.0, 0.0, 0.0);
  ASSERT_TRUE(both_faults.valid);
  EXPECT_TRUE(std::isnan(both_faults.temperature));
  EXPECT_TRUE(driver.mosTemperatureSensorFault());

  bus.ntc1_raw_ = 0x55;
  bus.ntc2_raw_ = 0x5A;
  const auto recovered = driver.sendControl(0.0, 0.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(recovered.temperature, 33.53, 0.01);
  EXPECT_FALSE(driver.motorTemperatureSensorFault());
}

TEST(BxiActuator, SaturatesFiniteRuntimeCommandsBeforePacking)
{
  EmulatedBus bus;
  ASSERT_TRUE(bus.open("emulated0"));
  auto driver = makeDriver(bus);
  ASSERT_TRUE(driver.enable().valid);
  EXPECT_TRUE(driver.sendControl(100.0, 100.0, 1000.0, 100.0, 1000.0).valid);
  ASSERT_EQ(bus.sent_.size(), 2U);
  EXPECT_EQ(bus.sent_[1].data[0], 0x8A);  // joint upper=1.0 rad, not protocol p_max
}

TEST(BxiActuator, RejectsNanWithoutTransmitting)
{
  EmulatedBus bus;
  ASSERT_TRUE(bus.open("emulated0"));
  auto driver = makeDriver(bus);
  ASSERT_TRUE(driver.enable().valid);
  const size_t sends_before = bus.sent_.size();
  EXPECT_FALSE(driver.sendControl(
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0, 0.0).valid);
  EXPECT_EQ(bus.sent_.size(), sends_before);
}

TEST(BxiActuator, RejectsMotorCanId16)
{
  EmulatedBus bus;
  ASSERT_TRUE(bus.open("emulated0"));
  BxiSpec spec;
  EXPECT_NO_THROW(BxiActuatorDriver(&bus, 15, 1, 0.0, -1.0, 1.0, 20.0, 1.0, spec));
  EXPECT_THROW(
    BxiActuatorDriver(&bus, 16, 1, 0.0, -1.0, 1.0, 20.0, 1.0, spec),
    std::invalid_argument);
}

TEST(BxiActuator, LatchesSafeStopAndStopsNewCommandsAfterTimeouts)
{
  EmulatedBus bus;
  ASSERT_TRUE(bus.open("emulated0"));
  auto driver = makeDriver(bus, 2);
  ASSERT_TRUE(driver.enable().valid);
  bus.respond_ = false;
  EXPECT_FALSE(driver.sendControl(0.0, 0.0, 0.0, 0.0, 0.0).valid);
  EXPECT_EQ(driver.state(), ActuatorState::kEnabled);
  EXPECT_FALSE(driver.sendControl(0.0, 0.0, 0.0, 0.0, 0.0).valid);
  EXPECT_EQ(driver.state(), ActuatorState::kSafeStopped);
  ASSERT_EQ(bus.sent_.size(), 4U);
  EXPECT_EQ(bus.sent_.back().data8(), protocol::encodeSpecialCommand(protocol::SpecialCommand::kDisable));
  const size_t sends_at_stop = bus.sent_.size();
  EXPECT_FALSE(driver.sendControl(0.0, 0.0, 0.0, 0.0, 0.0).valid);
  EXPECT_EQ(bus.sent_.size(), sends_at_stop);
}

TEST(BxiActuator, TriesDisableWhenControlSendFails)
{
  EmulatedBus bus;
  ASSERT_TRUE(bus.open("emulated0"));
  auto driver = makeDriver(bus);
  ASSERT_TRUE(driver.enable().valid);
  bus.close();
  EXPECT_FALSE(driver.sendControl(0.0, 0.0, 0.0, 0.0, 0.0).valid);
  EXPECT_EQ(driver.state(), ActuatorState::kSafeStopped);
  ASSERT_EQ(bus.sent_.size(), 3U);
  EXPECT_EQ(bus.sent_.back().data8(), protocol::encodeSpecialCommand(protocol::SpecialCommand::kDisable));
  EXPECT_FALSE(driver.sendControl(0.0, 0.0, 0.0, 0.0, 0.0).valid);
  EXPECT_EQ(bus.sent_.size(), 3U);
}

TEST(BxiActuator, RetryingDisableKeepsSafeStopLatched)
{
  EmulatedBus bus;
  ASSERT_TRUE(bus.open("emulated0"));
  auto driver = makeDriver(bus, 1);
  ASSERT_TRUE(driver.enable().valid);
  bus.respond_ = false;
  EXPECT_FALSE(driver.sendControl(0.0, 0.0, 0.0, 0.0, 0.0).valid);
  ASSERT_EQ(driver.state(), ActuatorState::kSafeStopped);
  const size_t sends_before_retry = bus.sent_.size();
  EXPECT_FALSE(driver.disable().valid);
  EXPECT_EQ(bus.sent_.size(), sends_before_retry + 1);
  EXPECT_EQ(bus.sent_.back().data8(), protocol::encodeSpecialCommand(protocol::SpecialCommand::kDisable));
  EXPECT_EQ(driver.state(), ActuatorState::kSafeStopped);
}

TEST(BxiActuator, UnrelatedFramesCannotExtendTransactionDeadline)
{
  EmulatedBus bus;
  ASSERT_TRUE(bus.open("emulated0"));
  auto driver = makeDriver(bus, 1);
  ASSERT_TRUE(driver.enable().valid);
  bus.respond_ = false;
  bus.unrelated_remaining_ = 10;

  EXPECT_FALSE(driver.sendControl(0.0, 0.0, 0.0, 0.0, 0.0).valid);
  EXPECT_LT(bus.unrelated_delivered_, 10);
  EXPECT_EQ(driver.state(), ActuatorState::kSafeStopped);
  EXPECT_EQ(bus.sent_.back().data8(),
    protocol::encodeSpecialCommand(protocol::SpecialCommand::kDisable));
}

TEST(BxiActuator, EncodeControlBlockMatchesTheFrameSentBySendControl)
{
  EmulatedBus bus;
  ASSERT_TRUE(bus.open("emulated0"));
  auto driver = makeDriver(bus);
  ASSERT_TRUE(driver.enable().valid);
  const auto block = driver.encodeControlBlock(100.0, -3.0, 1000.0, 0.5, -2.0);
  ASSERT_TRUE(block.has_value());
  const size_t sends_before = bus.sent_.size();
  ASSERT_TRUE(driver.sendControl(100.0, -3.0, 1000.0, 0.5, -2.0).valid);
  ASSERT_EQ(bus.sent_.size(), sends_before + 1);
  EXPECT_EQ(bus.sent_.back().data8(), *block);
  EXPECT_EQ(bus.sent_.back().dlc, 8U);
}

TEST(BxiActuator, EncodeControlBlockRequiresEnableAndFiniteInputsAndSendsNothing)
{
  EmulatedBus bus;
  ASSERT_TRUE(bus.open("emulated0"));
  auto driver = makeDriver(bus);
  EXPECT_FALSE(driver.encodeControlBlock(0.0, 0.0, 0.0, 0.0, 0.0).has_value());
  ASSERT_TRUE(driver.enable().valid);
  const size_t sends_after_enable = bus.sent_.size();
  EXPECT_TRUE(driver.encodeControlBlock(0.0, 0.0, 0.0, 0.0, 0.0).has_value());
  EXPECT_FALSE(driver.encodeControlBlock(
    0.0, std::numeric_limits<double>::infinity(), 0.0, 0.0, 0.0).has_value());
  EXPECT_EQ(bus.sent_.size(), sends_after_enable);
}

TEST(BxiActuator, DecodeMotorReplyChecksIdLengthAndPayloadMotorId)
{
  EmulatedBus bus;
  const protocol::MitLimits limits;
  const auto reply = bus.feedbackFrame(1);
  const auto decoded = decodeMotorReply(reply, 1, limits);
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->motor_id, 1U);

  EXPECT_FALSE(decodeMotorReply(bus.feedbackFrame(2), 1, limits).has_value());
  auto extended = reply;
  extended.extended = true;
  EXPECT_FALSE(decodeMotorReply(extended, 1, limits).has_value());
  auto long_frame = reply;
  long_frame.dlc = 24;
  EXPECT_FALSE(decodeMotorReply(long_frame, 1, limits).has_value());
  auto wrong_payload_id = reply;
  wrong_payload_id.data[0] = 2;  // reply ID of motor 1, payload naming motor 2
  EXPECT_FALSE(decodeMotorReply(wrong_payload_id, 1, limits).has_value());
}

TEST(BxiActuator, ApplyFeedbackMatchesTheSequentialTransactionResult)
{
  EmulatedBus bus;
  ASSERT_TRUE(bus.open("emulated0"));
  BxiSpec spec;
  spec.t_min = -80.0;
  spec.t_max = 80.0;
  BxiActuatorDriver driver(&bus, 1, -1, 0.3, -1.0, 1.0, 20.0, 1.0, spec, 1, 3);
  ASSERT_TRUE(driver.enable().valid);
  bus.ntc2_raw_ = 0x02;  // NTC2 sensor fault
  const auto sequential = driver.sendControl(0.0, 0.0, 0.0, 0.0, 0.0);
  ASSERT_TRUE(sequential.valid);

  const auto decoded = decodeMotorReply(bus.feedbackFrame(1), 1, driver.limits());
  ASSERT_TRUE(decoded.has_value());
  const auto applied = driver.applyFeedback(*decoded);
  EXPECT_TRUE(applied.valid);
  EXPECT_DOUBLE_EQ(applied.position, sequential.position);
  EXPECT_DOUBLE_EQ(applied.velocity, sequential.velocity);
  EXPECT_DOUBLE_EQ(applied.effort, sequential.effort);
  EXPECT_DOUBLE_EQ(applied.temperature, sequential.temperature);
  EXPECT_TRUE(std::isnan(applied.motor_temperature));
  EXPECT_TRUE(driver.motorTemperatureSensorFault());

  bus.ntc2_raw_ = 0x5A;
  driver.applyFeedback(*decodeMotorReply(bus.feedbackFrame(1), 1, driver.limits()));
  EXPECT_FALSE(driver.motorTemperatureSensorFault());
}

TEST(BxiActuator, CommunicationFaultLatchesSafeStopOnce)
{
  EmulatedBus bus;
  ASSERT_TRUE(bus.open("emulated0"));
  auto driver = makeDriver(bus);
  ASSERT_TRUE(driver.enable().valid);
  const size_t sends_before = bus.sent_.size();
  driver.latchCommunicationFault();
  EXPECT_EQ(driver.state(), ActuatorState::kSafeStopped);
  ASSERT_EQ(bus.sent_.size(), sends_before + 1);
  EXPECT_EQ(bus.sent_.back().data8(),
    protocol::encodeSpecialCommand(protocol::SpecialCommand::kDisable));
  EXPECT_FALSE(driver.encodeControlBlock(0.0, 0.0, 0.0, 0.0, 0.0).has_value());
  driver.latchCommunicationFault();
  EXPECT_EQ(bus.sent_.size(), sends_before + 1);
}

}  // namespace
}  // namespace bxi_hardware
