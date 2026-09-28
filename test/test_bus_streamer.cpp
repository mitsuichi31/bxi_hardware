#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <thread>
#include <tuple>
#include <vector>

#include <pthread.h>
#include <sys/resource.h>

#include <gtest/gtest.h>

#include "bxi_hardware/bus_streamer.hpp"

namespace bxi_hardware
{
namespace
{

using namespace std::chrono_literals;
using clock_type = std::chrono::steady_clock;

CanFrame feedbackFrame(int motor_id)
{
  CanFrame frame;
  frame.id = protocol::defaultMasterId(static_cast<uint32_t>(motor_id));
  frame.extended = false;
  frame.dlc = 8;
  frame.data = {{static_cast<uint8_t>(motor_id), 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 0x55, 0x5A}};
  return frame;
}

// In-memory CAN link with emulated motors. Replies become readable after a configurable delay,
// so the send and receive threads see the same timing structure as on a real bus.
class FakeLink
{
public:
  void onSend(const CanFrame & frame)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sent_.push_back(frame);
    if (!respond_) {
      return;
    }
    std::vector<int> motors;
    if (frame.id == 0x7FFU) {
      for (int block = 0; block < frame.dlc / 8; ++block) {
        motors.push_back(block + 1);
      }
    } else if (frame.dlc == 8) {
      motors.push_back(static_cast<int>(frame.id));
    }
    const auto now = clock_type::now();
    for (const int motor : motors) {
      if (responding_ids_.count(motor) == 0) {
        continue;
      }
      if (drop_next_ > 0) {
        --drop_next_;
        continue;
      }
      const auto delay = delays_[delay_index_++ % delays_.size()];
      pending_.emplace(now + delay, feedbackFrame(motor));
    }
    cv_.notify_all();
  }

  bool recv(CanFrame & out, int timeout_ms)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto deadline = clock_type::now() + std::chrono::milliseconds(timeout_ms);
    while (true) {
      if (flood_) {
        out = CanFrame{};
        out.id = 0x123;
        out.extended = false;
        out.dlc = 8;
        return true;
      }
      const auto now = clock_type::now();
      if (!pending_.empty() && pending_.begin()->first <= now) {
        out = pending_.begin()->second;
        pending_.erase(pending_.begin());
        return true;
      }
      if (now >= deadline) {
        return false;
      }
      auto wake = deadline;
      if (!pending_.empty()) {
        wake = std::min(wake, pending_.begin()->first);
      }
      cv_.wait_until(lock, wake);
    }
  }

  void inject(const CanFrame & frame)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.emplace(clock_type::now(), frame);
    cv_.notify_all();
  }
  void clearPending()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.clear();
  }
  void setRespond(bool respond)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    respond_ = respond;
  }
  void setRespondingIds(std::set<int> ids)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    responding_ids_ = std::move(ids);
  }
  void dropNext(int count)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    drop_next_ = count;
  }
  void setDelays(std::vector<clock_type::duration> delays)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    delays_ = std::move(delays);
    delay_index_ = 0;
  }
  void setFlood(bool flood)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    flood_ = flood;
    cv_.notify_all();
  }
  std::vector<CanFrame> sent()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return sent_;
  }

  std::atomic<bool> send_ok{true};
  // Runs after each accepted send; set before the stream starts.
  std::function<void()> after_send;

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  std::multimap<clock_type::time_point, CanFrame> pending_;
  std::vector<CanFrame> sent_;
  bool respond_{true};
  std::set<int> responding_ids_{1, 2, 3};
  int drop_next_{0};
  std::vector<clock_type::duration> delays_{300us};
  size_t delay_index_{0};
  bool flood_{false};
};

class FakeBus : public CanBus
{
public:
  explicit FakeBus(FakeLink & link, std::string name)
  : link_(link), name_(std::move(name)) {}
  bool open(const std::string &) override {return true;}
  void close() override {}
  bool isOpen() const override {return true;}
  bool send(const CanFrame & frame) override
  {
    if (!link_.send_ok) {
      return false;
    }
    link_.onSend(frame);
    if (link_.after_send) {
      link_.after_send();
    }
    return true;
  }
  bool recv(CanFrame & out, int timeout_ms) override {return link_.recv(out, timeout_ms);}
  void flushRx() override {link_.clearPending();}
  const std::string & name() const override {return name_;}

private:
  FakeLink & link_;
  std::string name_;
};

// Drivers enabled through the sequential path, as on_activate() does before streaming.
class Rig
{
public:
  explicit Rig(std::vector<int> ids)
  : tx(link, "fake_tx"), rx(link, "fake_rx")
  {
    BxiSpec spec;
    for (const int id : ids) {
      drivers.push_back(std::make_unique<BxiActuatorDriver>(
        &tx, id, 1, 0.0, -1.0, 1.0, 20.0, 1.0, spec, 5, 3));
      EXPECT_TRUE(drivers.back()->enable().valid) << "motor " << id;
    }
    link.clearPending();
  }

  std::unique_ptr<BxiBusStreamer> makeStreamer(
    StreamConfig config, BxiBusStreamer::CycleHook hook = nullptr)
  {
    if (!hook) {
      hook = [this](const std::vector<StreamFeedback> &, std::vector<StreamCommand> &) {
          ++hook_calls;
          return true;
        };
    }
    std::vector<BxiActuatorDriver *> raw;
    for (auto & driver : drivers) {
      raw.push_back(driver.get());
    }
    return std::make_unique<BxiBusStreamer>(&tx, &rx, raw, config, hook, &stop_all);
  }

  // Waits until the stream has stopped itself, returning when that was observed.
  std::optional<clock_type::time_point> waitForStop(clock_type::duration timeout)
  {
    const auto deadline = clock_type::now() + timeout;
    while (clock_type::now() < deadline) {
      if (stop_all) {
        return clock_type::now();
      }
      std::this_thread::sleep_for(100us);
    }
    return std::nullopt;
  }

  FakeLink link;
  FakeBus tx;
  FakeBus rx;
  std::vector<std::unique_ptr<BxiActuatorDriver>> drivers;
  std::atomic<bool> stop_all{false};
  std::atomic<int> hook_calls{0};
};

StreamConfig config(std::chrono::nanoseconds period, bool broadcast)
{
  StreamConfig c;
  c.period = period;
  c.max_feedback_age = 3 * period;
  c.broadcast = broadcast;
  return c;
}

double toMs(clock_type::duration duration)
{
  return std::chrono::duration<double, std::milli>(duration).count();
}

double stopDurationMs(BxiBusStreamer & streamer)
{
  const auto begin = clock_type::now();
  streamer.stop();
  return toMs(clock_type::now() - begin);
}

TEST(BusStreamer, BroadcastFramePlacesBlocksByMotorId)
{
  const std::array<uint8_t, 8> b1{{1, 1, 1, 1, 1, 1, 1, 1}};
  const std::array<uint8_t, 8> b2{{2, 2, 2, 2, 2, 2, 2, 2}};
  const std::array<uint8_t, 8> b3{{3, 3, 3, 3, 3, 3, 3, 3}};
  const std::array<uint8_t, 8> pad{{9, 9, 9, 9, 9, 9, 9, 9}};
  const auto frame = BxiBusStreamer::buildBroadcastFrame({{2, b2}, {1, b1}, {3, b3}}, 3, pad);
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->id, 0x7FFU);
  EXPECT_FALSE(frame->extended);
  ASSERT_EQ(frame->dlc, 24U);
  for (size_t i = 0; i < 8; ++i) {
    EXPECT_EQ(frame->data[i], 1U);
    EXPECT_EQ(frame->data[8 + i], 2U);
    EXPECT_EQ(frame->data[16 + i], 3U);
  }

  const auto padded = BxiBusStreamer::buildBroadcastFrame({{1, b1}}, 3, pad);
  ASSERT_TRUE(padded.has_value());
  EXPECT_EQ(padded->data[0], 1U);
  EXPECT_EQ(padded->data[8], 9U);
  EXPECT_EQ(padded->data[23], 9U);

  EXPECT_FALSE(BxiBusStreamer::buildBroadcastFrame({{1, b1}}, 5, pad).has_value());  // 40 B
  EXPECT_FALSE(BxiBusStreamer::buildBroadcastFrame({{4, b1}}, 3, pad).has_value());
}

TEST(BusStreamer, StartRejectsInvalidSetups)
{
  {
    Rig rig({1, 3});
    auto streamer = rig.makeStreamer(config(1ms, true));
    EXPECT_FALSE(streamer->start());
    EXPECT_NE(streamer->error().find("1..N"), std::string::npos);
  }
  {
    Rig rig({1});
    auto c = config(1ms, false);
    c.max_feedback_age = 1500us;
    EXPECT_FALSE(rig.makeStreamer(c)->start());
  }
  {
    Rig rig({1});
    auto c = config(1ms, true);
    c.padded_broadcast_blocks = 3;
    c.forbidden_reply_ids = {1};
    EXPECT_FALSE(rig.makeStreamer(c)->start());
  }
  {
    Rig rig({1});
    auto c = config(1ms, false);
    c.padded_broadcast_blocks = 3;
    EXPECT_FALSE(rig.makeStreamer(c)->start());
  }
}

class HealthyStream : public ::testing::TestWithParam<std::chrono::nanoseconds> {};

TEST_P(HealthyStream, NormalRepliesKeepRunning)
{
  const auto period = GetParam();
  Rig rig({1, 2, 3});
  auto streamer = rig.makeStreamer(config(period, true));
  ASSERT_TRUE(streamer->start());
  EXPECT_TRUE(streamer->waitForFirstFeedback(3 * period + period));
  std::this_thread::sleep_for(60 * period);
  EXPECT_FALSE(rig.stop_all);
  streamer->stop();
  const auto stats = streamer->stats();
  EXPECT_EQ(stats.fault, StreamFault::kNone);
  EXPECT_GE(stats.cycles, 40U);
  EXPECT_EQ(stats.frames_sent, stats.cycles);
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_GE(stats.replies[i] + 1, stats.cycles) << "axis " << i;
    EXPECT_LT(stats.max_age_ms[i], 3.0 * toMs(period));
  }
}

TEST_P(HealthyStream, OneDroppedReplyDoesNotStop)
{
  const auto period = GetParam();
  Rig rig({1, 2, 3});
  auto streamer = rig.makeStreamer(config(period, true));
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(4 * period));
  std::this_thread::sleep_for(10 * period);
  rig.link.dropNext(1);
  std::this_thread::sleep_for(20 * period);
  streamer->stop();
  const auto stats = streamer->stats();
  EXPECT_EQ(stats.fault, StreamFault::kNone);
  EXPECT_GE(stats.cycles_without_new_reply[0], 1U);
}

INSTANTIATE_TEST_SUITE_P(
  Periods, HealthyStream, ::testing::Values(std::chrono::nanoseconds(5ms),
  std::chrono::nanoseconds(1ms)));

TEST(BusStreamer, RepliesCrossingTheCycleBoundaryDoNotStop)
{
  Rig rig({1, 2, 3});
  rig.link.setDelays({900us, 1100us, 1000us, 950us, 1050us});
  auto streamer = rig.makeStreamer(config(1ms, true));
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(4ms));
  std::this_thread::sleep_for(200ms);
  streamer->stop();
  const auto stats = streamer->stats();
  EXPECT_EQ(stats.fault, StreamFault::kNone);
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_GE(stats.replies[i] + 2, stats.cycles) << "axis " << i;
  }
}

class StoppedReplies : public ::testing::TestWithParam<std::chrono::nanoseconds> {};

TEST_P(StoppedReplies, StopWithinMaxAgePlusOnePeriod)
{
  const auto period = GetParam();
  Rig rig({1, 2, 3});
  const auto c = config(period, true);
  auto streamer = rig.makeStreamer(c);
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(4 * period));
  std::this_thread::sleep_for(10 * period);
  // Replies already in flight still arrive, so the last reply is at or before this point plus
  // the emulated delay.
  rig.link.setRespond(false);
  const auto silenced_at = clock_type::now();
  const auto stopped_at = rig.waitForStop(1s);
  ASSERT_TRUE(stopped_at.has_value());
  const double elapsed_ms = toMs(*stopped_at - silenced_at);
  const double bound_ms = toMs(c.max_feedback_age + period);
  // 1 ms margin for thread wake-up in the test environment.
  EXPECT_LE(elapsed_ms, bound_ms + 1.0);
  // The last reply can precede the silencing by up to one period.
  EXPECT_GE(elapsed_ms, toMs(c.max_feedback_age - period) - 1.0);

  const double stop_ms = stopDurationMs(*streamer);
  EXPECT_LT(stop_ms, 10.0);
  const auto stats = streamer->stats();
  EXPECT_EQ(stats.fault, StreamFault::kFeedbackStale);
  ASSERT_GE(stats.fault_axis, 0);
  const auto & faulted = *rig.drivers[static_cast<size_t>(stats.fault_axis)];
  EXPECT_EQ(faulted.state(), ActuatorState::kSafeStopped);
  const auto sent = rig.link.sent();
  ASSERT_FALSE(sent.empty());
  EXPECT_EQ(sent.back().id, static_cast<uint32_t>(faulted.canId()));
  EXPECT_EQ(sent.back().data8(),
    protocol::encodeSpecialCommand(protocol::SpecialCommand::kDisable));
}

INSTANTIATE_TEST_SUITE_P(
  Periods, StoppedReplies, ::testing::Values(std::chrono::nanoseconds(5ms),
  std::chrono::nanoseconds(1ms)));

TEST(BusStreamer, SilentMotorIsTheOneReportedStale)
{
  Rig rig({1, 2});
  rig.link.setRespondingIds({1});
  auto streamer = rig.makeStreamer(config(1ms, false));
  ASSERT_TRUE(streamer->start());
  EXPECT_FALSE(streamer->waitForFirstFeedback(20ms));
  ASSERT_TRUE(rig.waitForStop(100ms).has_value());
  streamer->stop();
  const auto stats = streamer->stats();
  EXPECT_EQ(stats.fault, StreamFault::kFeedbackStale);
  EXPECT_EQ(stats.fault_axis, 1);
  EXPECT_GT(stats.replies[0], 0U);
  EXPECT_EQ(stats.replies[1], 0U);
  EXPECT_EQ(rig.drivers[1]->state(), ActuatorState::kSafeStopped);
  EXPECT_EQ(rig.drivers[0]->state(), ActuatorState::kEnabled);
}

TEST(BusStreamer, UnicastSendsOneFramePerMotorAndBroadcastSendsOne)
{
  for (const bool broadcast : {false, true}) {
    Rig rig({1, 2, 3});
    const size_t enable_frames = rig.link.sent().size();
    auto streamer = rig.makeStreamer(config(1ms, broadcast));
    ASSERT_TRUE(streamer->start());
    std::this_thread::sleep_for(30ms);
    streamer->stop();
    const auto stats = streamer->stats();
    EXPECT_EQ(stats.fault, StreamFault::kNone) << broadcast;
    EXPECT_EQ(stats.frames_sent, stats.cycles * (broadcast ? 1U : 3U)) << broadcast;
    const auto sent = rig.link.sent();
    ASSERT_EQ(sent.size(), enable_frames + stats.frames_sent);
    for (size_t i = enable_frames; i < sent.size(); ++i) {
      const auto & frame = sent[i];
      if (broadcast) {
        EXPECT_EQ(frame.id, 0x7FFU);
        EXPECT_EQ(frame.dlc, 24U);
      } else {
        EXPECT_LE(frame.id, 3U);
        EXPECT_EQ(frame.dlc, 8U);
      }
    }
  }
}

TEST(BusStreamer, PaddedBroadcastStopsOnAReplyFromAnAbsentMotor)
{
  Rig rig({1});
  auto c = config(1ms, true);
  c.padded_broadcast_blocks = 3;
  c.forbidden_reply_ids = {2, 3};
  rig.link.setRespondingIds({1});
  auto streamer = rig.makeStreamer(c);
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(4ms));
  std::this_thread::sleep_for(10ms);
  EXPECT_FALSE(rig.stop_all);
  const auto zero_block =
    protocol::encodeCommand(protocol::MitCommand{}, protocol::MitLimits{}).value();
  const auto sent = rig.link.sent();
  ASSERT_FALSE(sent.empty());
  EXPECT_EQ(sent.back().dlc, 24U);
  for (size_t i = 0; i < 8; ++i) {
    EXPECT_EQ(sent.back().data[8 + i], zero_block[i]);
    EXPECT_EQ(sent.back().data[16 + i], zero_block[i]);
  }

  rig.link.setRespondingIds({1, 3});
  ASSERT_TRUE(rig.waitForStop(20ms).has_value());
  streamer->stop();
  const auto stats = streamer->stats();
  EXPECT_EQ(stats.fault, StreamFault::kForbiddenReply);
  EXPECT_EQ(stats.forbidden_reply_id, 3);
  EXPECT_GE(stats.forbidden_replies, 1U);
}

TEST(BusStreamer, OwnerHookCanRequestAStop)
{
  Rig rig({1});
  const size_t enable_frames = rig.link.sent().size();
  std::atomic<int> calls{0};
  auto streamer = rig.makeStreamer(config(1ms, false),
      [&calls](const std::vector<StreamFeedback> &, std::vector<StreamCommand> &) {
        return ++calls < 10;
      });
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(rig.waitForStop(100ms).has_value());
  streamer->stop();
  const auto stats = streamer->stats();
  EXPECT_EQ(stats.fault, StreamFault::kOwnerRequested);
  EXPECT_EQ(stats.cycles, 9U);
  EXPECT_EQ(rig.link.sent().size(), enable_frames + 9U);
}

TEST(BusStreamer, SendFailureLatchesEveryBroadcastMotor)
{
  Rig rig({1, 2});
  auto streamer = rig.makeStreamer(config(1ms, true));
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(4ms));
  rig.link.send_ok = false;
  ASSERT_TRUE(rig.waitForStop(20ms).has_value());
  streamer->stop();
  EXPECT_EQ(streamer->stats().fault, StreamFault::kSendFailed);
  for (const auto & driver : rig.drivers) {
    EXPECT_EQ(driver->state(), ActuatorState::kSafeStopped);
  }
}

TEST(BusStreamer, ExternalStopEndsSendingWithoutAFault)
{
  Rig rig({1});
  auto streamer = rig.makeStreamer(config(1ms, false));
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(4ms));
  rig.stop_all = true;  // another bus faulted
  std::this_thread::sleep_for(5ms);
  const size_t sent = rig.link.sent().size();
  std::this_thread::sleep_for(5ms);
  EXPECT_EQ(rig.link.sent().size(), sent);
  streamer->stop();
  const auto stats = streamer->stats();
  EXPECT_TRUE(stats.stopped_externally);
  EXPECT_EQ(stats.fault, StreamFault::kNone);
  EXPECT_EQ(rig.drivers[0]->state(), ActuatorState::kEnabled);
}

// The hook can block (for example on the owner's data mutex). A command prepared for a slot
// that has meanwhile expired must not be sent.
TEST(BusStreamer, SlowHookDoesNotSendAnExpiredCommand)
{
  Rig rig({1});
  auto c = config(1ms, false);
  c.max_feedback_age = 50ms;
  c.trace_cycles = 200;
  std::atomic<int> calls{0};
  // Each send records which hook call prepared it. The send thread runs both, so the value read
  // here is the call of the current cycle.
  std::mutex sent_mutex;
  std::vector<int> sent_by_call;
  rig.link.after_send = [&]() {
      std::lock_guard<std::mutex> lock(sent_mutex);
      sent_by_call.push_back(calls.load());
    };
  auto streamer = rig.makeStreamer(c,
      [&calls](const std::vector<StreamFeedback> &, std::vector<StreamCommand> &) {
        if (++calls == 5) {
          std::this_thread::sleep_for(5ms);
        }
        return true;
      });
  ASSERT_TRUE(streamer->start());
  std::this_thread::sleep_for(30ms);
  streamer->stop();
  const auto stats = streamer->stats();
  EXPECT_EQ(stats.fault, StreamFault::kNone);
  EXPECT_GE(stats.skipped_cycles, 1U);
  EXPECT_EQ(stats.frames_sent, stats.cycles);
  // The slow fifth call's command was never sent, and later cycles kept sending. Other calls
  // may also expire on a slow machine (for example under ThreadSanitizer); they are not counted.
  std::lock_guard<std::mutex> lock(sent_mutex);
  EXPECT_EQ(std::count(sent_by_call.begin(), sent_by_call.end(), 5), 0);
  EXPECT_TRUE(std::any_of(sent_by_call.begin(), sent_by_call.end(),
    [](int call) {return call > 5;}));
  // The trace shows the expired cycle and how long its hook took.
  const auto & trace = streamer->trace();
  const auto rows = trace.cycles.begin() +
    static_cast<std::ptrdiff_t>(std::min(trace.cycle_count, trace.cycles.size()));
  const auto expired = std::find_if(trace.cycles.begin(), rows, [](const StreamCycleRecord & row) {
      return row.outcome == StreamCycleOutcome::kExpiredAfterHook;
    });
  ASSERT_NE(expired, rows);
  EXPECT_GE(expired->hook_ns, 4'500'000);
  EXPECT_EQ(expired->frames, 0U);
  EXPECT_EQ(expired->send_lateness_ns, -1);
}

TEST(BusStreamer, StopRequestedDuringTheHookSendsNothing)
{
  Rig rig({1});
  std::atomic<bool> * stop_all = &rig.stop_all;
  std::atomic<int> calls{0};
  auto streamer = rig.makeStreamer(config(1ms, false),
      [&calls, stop_all](const std::vector<StreamFeedback> &, std::vector<StreamCommand> &) {
        if (++calls == 5) {
          stop_all->store(true);  // another bus faults while this hook runs
        }
        return true;
      });
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(rig.waitForStop(100ms).has_value());
  streamer->stop();
  const auto stats = streamer->stats();
  EXPECT_TRUE(stats.stopped_externally);
  EXPECT_EQ(stats.fault, StreamFault::kNone);
  EXPECT_EQ(stats.cycles, 4U);
  EXPECT_EQ(stats.frames_sent, 4U);
}

TEST(BusStreamer, StopBetweenPerMotorFramesSendsNoFurtherFrame)
{
  Rig rig({1, 2, 3});
  std::atomic<int> streamed{0};
  std::atomic<bool> * stop_all = &rig.stop_all;
  rig.link.after_send = [&streamed, stop_all]() {
      if (++streamed == 4) {  // first frame of the second cycle
        stop_all->store(true);
      }
    };
  auto streamer = rig.makeStreamer(config(1ms, false));
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(rig.waitForStop(100ms).has_value());
  std::this_thread::sleep_for(5ms);
  streamer->stop();
  const auto stats = streamer->stats();
  EXPECT_TRUE(stats.stopped_externally);
  EXPECT_EQ(stats.cycles, 1U);
  EXPECT_EQ(stats.frames_sent, 4U);
  EXPECT_EQ(streamed.load(), 4);
}

TEST(BusStreamer, FirstFeedbackIgnoresRepliesReceivedBeforeTheFirstSend)
{
  Rig rig({1});
  auto c = config(10ms, false);
  c.max_feedback_age = 1s;
  // A valid reply is already waiting when the receive socket starts, and the motor does not
  // answer streamed commands. The first hook call holds the first send back by 3 ms so that the
  // receive thread takes the early reply before it.
  rig.link.inject(feedbackFrame(1));
  rig.link.setRespond(false);
  std::atomic<int> calls{0};
  auto streamer = rig.makeStreamer(c,
      [&calls](const std::vector<StreamFeedback> &, std::vector<StreamCommand> &) {
        if (++calls == 1) {
          std::this_thread::sleep_for(3ms);
        }
        return true;
      });
  ASSERT_TRUE(streamer->start());
  EXPECT_FALSE(streamer->waitForFirstFeedback(30ms));
  rig.link.setRespond(true);
  EXPECT_TRUE(streamer->waitForFirstFeedback(30ms));
  streamer->stop();
  EXPECT_GE(streamer->stats().replies[0], 2U);
}

TEST(BusStreamer, TraceRecordsCyclesAgesRepliesAndThreads)
{
  Rig rig({1, 2});
  auto c = config(1ms, true);
  c.trace_cycles = 1000;
  auto streamer = rig.makeStreamer(c);
  ASSERT_TRUE(streamer->start());
  std::this_thread::sleep_for(50ms);
  streamer->stop();
  const auto stats = streamer->stats();
  const auto & trace = streamer->trace();
  ASSERT_EQ(stats.fault, StreamFault::kNone);
  ASSERT_LE(trace.cycle_count, trace.cycles.size());
  EXPECT_EQ(trace.axes, 2U);

  uint64_t sent_rows = 0;
  bool seen_send = false;
  for (size_t r = 0; r < trace.cycle_count; ++r) {
    const auto & row = trace.cycles[r];
    EXPECT_EQ(row.cycle, r);  // no row is skipped while the trace has room
    EXPECT_GE(row.wake_lateness_ns, 0);
    for (size_t a = 0; a < trace.axes; ++a) {
      // Ages are -1 only until the first send, and fresh is a flag.
      EXPECT_EQ(trace.age_ns[r * trace.axes + a] >= 0, seen_send) << r;
      EXPECT_LE(trace.fresh[r * trace.axes + a], 1U);
    }
    if (row.outcome == StreamCycleOutcome::kSent) {
      ++sent_rows;
      seen_send = true;
      EXPECT_EQ(row.frames, 1U);
      EXPECT_GE(row.send_lateness_ns, 0);
      EXPECT_LT(row.send_lateness_ns, 1'000'000);
    }
  }
  EXPECT_EQ(sent_rows, stats.cycles);
  EXPECT_EQ(trace.rx_count, stats.replies[0] + stats.replies[1]);
  for (size_t r = 0; r < trace.rx_count; ++r) {
    EXPECT_LT(trace.rx[r].axis, 2U);
    EXPECT_GE(trace.rx[r].received_ns, 0);
  }
  EXPECT_NE(trace.tx_thread.find("SCHED_"), std::string::npos) << trace.tx_thread;
  EXPECT_NE(trace.tx_thread.find("cpus="), std::string::npos) << trace.tx_thread;
  EXPECT_NE(trace.rx_thread.find("SCHED_"), std::string::npos) << trace.rx_thread;
}

TEST(BusStreamer, FullTraceCountsDroppedRowsAndDisabledTraceStoresNone)
{
  {
    Rig rig({1});
    auto c = config(1ms, false);
    c.trace_cycles = 5;
    auto streamer = rig.makeStreamer(c);
    ASSERT_TRUE(streamer->start());
    std::this_thread::sleep_for(30ms);
    streamer->stop();
    const auto & trace = streamer->trace();
    EXPECT_EQ(trace.cycles.size(), 5U);
    EXPECT_GT(trace.cycle_count, 5U);
    EXPECT_EQ(trace.rx.size(), 10U);
    EXPECT_GT(trace.rx_count, 10U);
    EXPECT_EQ(streamer->stats().fault, StreamFault::kNone);
  }
  {
    Rig rig({1});
    auto streamer = rig.makeStreamer(config(1ms, false));
    ASSERT_TRUE(streamer->start());
    std::this_thread::sleep_for(10ms);
    streamer->stop();
    const auto & trace = streamer->trace();
    EXPECT_TRUE(trace.cycles.empty());
    EXPECT_TRUE(trace.age_ns.empty());
    EXPECT_TRUE(trace.rx.empty());
    EXPECT_GT(trace.cycle_count, 0U);
  }
}

// Lowers the soft RLIMIT_RTPRIO for its scope, so SCHED_FIFO requests are refused unless the
// process holds CAP_SYS_NICE. The hard limit stays, so the soft limit can be restored.
class NoRealtimePriority
{
public:
  NoRealtimePriority()
  {
    lowered_ = ::getrlimit(RLIMIT_RTPRIO, &saved_) == 0;
    if (lowered_) {
      rlimit none = saved_;
      none.rlim_cur = 0;
      lowered_ = ::setrlimit(RLIMIT_RTPRIO, &none) == 0;
    }
  }
  ~NoRealtimePriority()
  {
    if (lowered_) {
      ::setrlimit(RLIMIT_RTPRIO, &saved_);
    }
  }
  bool lowered() const {return lowered_;}

private:
  rlimit saved_{};
  bool lowered_ = false;
};

TEST(BusStreamer, AppliesFifoPriorityAndAffinityToBothThreads)
{
  Rig rig({1});
  auto c = config(1ms, false);
  c.send_priority = 20;
  c.receive_priority = 21;
  c.cpus = {0};
  auto streamer = rig.makeStreamer(c);
  if (!streamer->start()) {
    if (streamer->error().find("Operation not permitted") != std::string::npos) {
      GTEST_SKIP() << "SCHED_FIFO is not allowed here: " << streamer->error();
    }
    FAIL() << streamer->error();
  }
  ASSERT_TRUE(streamer->waitForFirstFeedback(10ms));
  streamer->stop();
  EXPECT_EQ(streamer->trace().tx_thread, "SCHED_FIFO priority=20 cpus=0");
  EXPECT_EQ(streamer->trace().rx_thread, "SCHED_FIFO priority=21 cpus=0");
  EXPECT_EQ(streamer->stats().fault, StreamFault::kNone);
}

// Unset priorities mean SCHED_OTHER even when start() runs on a SCHED_FIFO thread, which the
// stream threads would otherwise inherit.
TEST(BusStreamer, UnsetPriorityGivesSchedOtherEvenFromAFifoCaller)
{
  Rig rig({1});
  auto streamer = rig.makeStreamer(config(1ms, false));
  bool started = false;
  std::string caller_setup;
  std::thread caller([&]() {
      sched_param param{};
      param.sched_priority = 10;
      const int result = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
      if (result != 0) {
        caller_setup = std::strerror(result);
        return;
      }
      started = streamer->start();
    });
  caller.join();
  if (!caller_setup.empty()) {
    GTEST_SKIP() << "SCHED_FIFO is not allowed here: " << caller_setup;
  }
  ASSERT_TRUE(started) << streamer->error();
  ASSERT_TRUE(streamer->waitForFirstFeedback(10ms));
  streamer->stop();
  EXPECT_EQ(streamer->trace().tx_thread.rfind("SCHED_OTHER priority=0", 0), 0U)
    << streamer->trace().tx_thread;
  EXPECT_EQ(streamer->trace().rx_thread.rfind("SCHED_OTHER priority=0", 0), 0U)
    << streamer->trace().rx_thread;
}

TEST(BusStreamer, RefusedFifoPriorityFailsStartAndSendsNothing)
{
  Rig rig({1});
  const size_t enable_frames = rig.link.sent().size();
  auto c = config(1ms, false);
  c.send_priority = 20;
  NoRealtimePriority no_rt;
  if (!no_rt.lowered()) {
    GTEST_SKIP() << "RLIMIT_RTPRIO could not be lowered";
  }
  auto streamer = rig.makeStreamer(c);
  if (streamer->start()) {
    streamer->stop();
    GTEST_SKIP() << "this process may use SCHED_FIFO regardless of RLIMIT_RTPRIO";
  }
  EXPECT_NE(streamer->error().find("send thread: SCHED_FIFO priority 20"), std::string::npos)
    << streamer->error();
  EXPECT_TRUE(streamer->sendFinished());
  EXPECT_EQ(rig.link.sent().size(), enable_frames);
  streamer->stop();  // nothing left to join
  EXPECT_EQ(rig.drivers[0]->state(), ActuatorState::kEnabled);
}

TEST(BusStreamer, RejectsOutOfRangeThreadSettings)
{
  for (const auto & [send, receive, cpu] : {std::tuple{99, 0, 0}, std::tuple{0, -1, 0},
      std::tuple{0, 0, -1}})
  {
    Rig rig({1});
    auto c = config(1ms, false);
    c.send_priority = send;
    c.receive_priority = receive;
    c.cpus = {cpu};
    auto streamer = rig.makeStreamer(c);
    EXPECT_FALSE(streamer->start()) << send << " " << receive << " " << cpu;
    EXPECT_NE(streamer->error().find("thread priority"), std::string::npos);
  }
}

// Stop paths from the plan: an idle bus, a flooded bus, after a fault, and after a failed
// first-feedback wait during activation.
TEST(BusStreamer, StopJoinsWithin10msOnAnIdleBus)
{
  Rig rig({1});
  rig.link.setRespond(false);
  auto c = config(5ms, false);
  c.max_feedback_age = 1s;
  auto streamer = rig.makeStreamer(c);
  ASSERT_TRUE(streamer->start());
  std::this_thread::sleep_for(30ms);
  EXPECT_LT(stopDurationMs(*streamer), 10.0);
}

TEST(BusStreamer, StopJoinsWithin10msOnAFloodedBus)
{
  Rig rig({1});
  auto c = config(5ms, false);
  c.max_feedback_age = 1s;
  auto streamer = rig.makeStreamer(c);
  ASSERT_TRUE(streamer->start());
  rig.link.setFlood(true);
  std::this_thread::sleep_for(30ms);
  EXPECT_LT(stopDurationMs(*streamer), 10.0);
  EXPECT_GT(streamer->stats().rx_ignored, 100U);
}

TEST(BusStreamer, StopJoinsWithin10msAfterAFailedFirstFeedbackWait)
{
  Rig rig({1});
  rig.link.setRespond(false);
  const auto c = config(1ms, false);
  auto streamer = rig.makeStreamer(c);
  ASSERT_TRUE(streamer->start());
  EXPECT_FALSE(streamer->waitForFirstFeedback(c.max_feedback_age + c.period));
  EXPECT_LT(stopDurationMs(*streamer), 10.0);
  streamer->stop();  // idempotent
}

}  // namespace
}  // namespace bxi_hardware
