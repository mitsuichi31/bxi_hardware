#include <algorithm>
#include <cmath>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstring>
#include <functional>
#include <map>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <thread>
#include <tuple>
#include <vector>

#include <pthread.h>
#include <sys/resource.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include "bxi_hardware/bus_streamer.hpp"
#include "bxi_hardware/stream_trace_io.hpp"

namespace bxi_hardware
{
struct AtomicSnapshotTestAccess
{
  template<class T> static void pauseWriter(AtomicSnapshot<T> & value)
  {value.version_.store(value.version_.load() + 1);}
};
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
  // Makes send() fail for the frames it returns true for; set before the stream starts.
  std::function<bool(const CanFrame &)> reject;

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
    if (!link_.send_ok || (link_.reject && link_.reject(frame))) {
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
    StreamConfig config, BxiBusStreamer::CycleHook hook = nullptr,
    std::unique_ptr<WriteStreamStorage> storage = nullptr)
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
    return std::make_unique<BxiBusStreamer>(&tx, &rx, raw, config, hook, &stop_all, std::move(storage));
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

// Periodic write() of the owner's control loop, as BxiSystemInterface::write() does.
class CommandWriter
{
public:
  CommandWriter(std::atomic<int64_t> & stamp, std::chrono::nanoseconds interval)
  : stamp_(stamp), thread_([this, interval]() {
        while (running_) {
          stamp_.store(clock_type::now().time_since_epoch().count());
          last_ = clock_type::now();
          std::this_thread::sleep_for(interval);
        }
      }) {}
  ~CommandWriter() {stop();}
  // Stops writing and returns the time of the last write.
  clock_type::time_point stop()
  {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
    return last_;
  }

private:
  std::atomic<int64_t> & stamp_;
  std::atomic<bool> running_{true};
  clock_type::time_point last_{};
  std::thread thread_;
};

// kp and kd raw fields of the 8-byte MIT command block (12 bits each).
std::pair<uint32_t, uint32_t> rawKpKd(const std::array<uint8_t, 8> & block)
{
  const uint32_t kp = (static_cast<uint32_t>(block[3] & 0x0F) << 8) | block[4];
  const uint32_t kd = (static_cast<uint32_t>(block[5]) << 4) | (block[6] >> 4);
  return {kp, kd};
}

std::array<uint8_t, 8> broadcastBlock(const CanFrame & frame, size_t index)
{
  std::array<uint8_t, 8> block{};
  std::copy_n(frame.data.begin() + static_cast<std::ptrdiff_t>(index * 8), 8, block.begin());
  return block;
}

BxiBusStreamer::CycleHook gainHook()
{
  return [](const std::vector<StreamFeedback> &, std::vector<StreamCommand> & commands) {
           for (auto & command : commands) {
             command = StreamCommand{0.0, 0.0, 10.0, 1.0, 0.0};
           }
           return true;
         };
}

TEST(BusStreamer, CommandAgeIsNotCheckedBeforeTheFirstWrite)
{
  Rig rig({1});
  std::atomic<int64_t> stamp{0};
  auto c = config(1ms, true);
  c.max_command_age = 5ms;
  c.command_written_ns = &stamp;
  auto streamer = rig.makeStreamer(c, gainHook());
  ASSERT_TRUE(streamer->start());
  std::this_thread::sleep_for(50ms);
  EXPECT_FALSE(rig.stop_all);
  streamer->stop();
  const auto stats = streamer->stats();
  EXPECT_EQ(stats.fault, StreamFault::kNone);
  EXPECT_EQ(stats.max_command_age_ms, 0.0);
}

TEST(BusStreamer, RegularWritesKeepRunningBelowTheCommandAgeLimit)
{
  Rig rig({1, 2, 3});
  std::atomic<int64_t> stamp{0};
  auto c = config(1ms, true);
  c.max_command_age = 5ms;
  c.command_written_ns = &stamp;
  c.trace_cycles = 200;
  auto streamer = rig.makeStreamer(c, gainHook());
  // Writes every 3 ms stay below the 5 ms limit even though several cycles see the same write.
  CommandWriter writer(stamp, 3ms);
  ASSERT_TRUE(streamer->start());
  std::this_thread::sleep_for(100ms);
  EXPECT_FALSE(rig.stop_all);
  streamer->stop();
  writer.stop();
  const auto stats = streamer->stats();
  EXPECT_EQ(stats.fault, StreamFault::kNone);
  EXPECT_GT(stats.max_command_age_ms, 1.0);
  EXPECT_LT(stats.max_command_age_ms, 5.0);
  const auto & trace = streamer->trace();
  size_t checked = 0;
  for (size_t r = 0; r < std::min(trace.cycle_count, trace.cycles.size()); ++r) {
    checked += trace.cycles[r].command_age_ns >= 0 ? 1 : 0;
  }
  EXPECT_GT(checked, 50U);
}

TEST(BusStreamer, StaleCommandsSendZeroGainThenStopWhileRepliesContinue)
{
  Rig rig({1, 2, 3});
  const size_t enable_frames = rig.link.sent().size();
  std::atomic<int64_t> stamp{0};
  auto c = config(1ms, true);
  c.max_command_age = 5ms;
  c.command_written_ns = &stamp;
  auto streamer = rig.makeStreamer(c, gainHook());
  CommandWriter writer(stamp, 1ms);
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(10ms));
  std::this_thread::sleep_for(30ms);
  // The control loop stalls; the motors keep answering, so the reply age stays fresh.
  const auto last_write = writer.stop();
  const auto stopped_at = rig.waitForStop(1s);
  ASSERT_TRUE(stopped_at.has_value());
  const double elapsed_ms = toMs(*stopped_at - last_write);
  // Stops in the first cycle whose start sees an age over 5 ms: within one period after that,
  // plus 1 ms for thread wake-up in the test environment.
  EXPECT_GE(elapsed_ms, 5.0);
  EXPECT_LE(elapsed_ms, 5.0 + 1.0 + 1.0);
  streamer->stop();
  const auto stats = streamer->stats();
  EXPECT_EQ(stats.fault, StreamFault::kCommandStale);
  EXPECT_EQ(stats.fault_axis, -1);
  EXPECT_GT(stats.max_command_age_ms, 5.0);
  // Not a communication fault: every motor stays enabled for the owner's normal disable.
  for (const auto & driver : rig.drivers) {
    EXPECT_EQ(driver->state(), ActuatorState::kEnabled);
  }
  const auto sent = rig.link.sent();
  ASSERT_GE(sent.size(), enable_frames + 2U);
  const auto & last = sent.back();
  const auto & before = sent[sent.size() - 2];
  ASSERT_EQ(last.id, 0x7FFU);
  ASSERT_EQ(before.id, 0x7FFU);
  for (size_t block = 0; block < 3; ++block) {
    EXPECT_EQ(rawKpKd(broadcastBlock(last, block)), std::make_pair(0U, 0U)) << block;
    EXPECT_NE(rawKpKd(broadcastBlock(before, block)).first, 0U) << block;
  }
}

TEST(BusStreamer, StaleCycleSendsZeroGainToEveryMotorPastTheSlotDeadline)
{
  Rig rig({1, 2, 3});
  std::atomic<int64_t> stamp{0};
  std::atomic<bool> slow_sends{false};
  // Once the control loop has stalled, each frame takes 1.5 ms, so the 1 ms slot closes after the
  // first per-motor frame.
  rig.link.after_send = [&slow_sends]() {
      if (slow_sends) {
        std::this_thread::sleep_for(1500us);
      }
    };
  auto c = config(1ms, false);
  c.max_feedback_age = 10ms;  // slow sends delay the replies too
  c.max_command_age = 5ms;
  c.command_written_ns = &stamp;
  auto streamer = rig.makeStreamer(c, gainHook());
  CommandWriter writer(stamp, 1ms);
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(10ms));
  std::this_thread::sleep_for(20ms);
  writer.stop();
  slow_sends = true;
  ASSERT_TRUE(rig.waitForStop(1s).has_value());
  streamer->stop();
  const auto stats = streamer->stats();
  ASSERT_EQ(stats.fault, StreamFault::kCommandStale);
  EXPECT_EQ(stats.command_stale_zero_gain_frames, 3U);
  EXPECT_EQ(stats.command_stale_expected_frames, 3U);
  const auto sent = rig.link.sent();
  ASSERT_GE(sent.size(), 3U);
  for (size_t k = 0; k < 3; ++k) {
    const auto & frame = sent[sent.size() - 3 + k];
    EXPECT_EQ(frame.id, static_cast<uint32_t>(k + 1));
    EXPECT_EQ(rawKpKd(frame.data8()), std::make_pair(0U, 0U)) << k;
  }
}

TEST(BusStreamer, StalenessIsDetectedInACycleThatWouldBeSkippedAsLate)
{
  Rig rig({1, 2, 3});
  std::atomic<int64_t> stamp{0};
  std::atomic<bool> stall_hook{false};
  auto c = config(1ms, true);
  c.max_feedback_age = 30ms;  // the stalled hook also delays reply processing
  c.max_command_age = 5ms;
  c.command_written_ns = &stamp;
  c.trace_cycles = 1000;
  auto streamer = rig.makeStreamer(c,
      [&stall_hook](const std::vector<StreamFeedback> &, std::vector<StreamCommand> & commands) {
        for (auto & command : commands) {
          command = StreamCommand{0.0, 0.0, 10.0, 1.0, 0.0};
        }
        if (stall_hook.exchange(false)) {
          std::this_thread::sleep_for(12ms);  // the next slots are already late
        }
        return true;
      });
  CommandWriter writer(stamp, 1ms);
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(10ms));
  std::this_thread::sleep_for(20ms);
  writer.stop();
  stall_hook = true;
  ASSERT_TRUE(rig.waitForStop(1s).has_value());
  streamer->stop();
  const auto stats = streamer->stats();
  ASSERT_EQ(stats.fault, StreamFault::kCommandStale);
  EXPECT_EQ(stats.command_stale_zero_gain_frames, 1U);
  const auto & trace = streamer->trace();
  ASSERT_GT(trace.cycle_count, 0U);
  ASSERT_LE(trace.cycle_count, trace.cycles.size());
  const auto & last = trace.cycles[trace.cycle_count - 1];
  EXPECT_EQ(last.outcome, StreamCycleOutcome::kStopped);
  // The stopping cycle woke at least a full period late: without the check before the skip,
  // it would have been skipped.
  EXPECT_GE(last.wake_lateness_ns, 1'000'000);
  EXPECT_GT(last.command_age_ns, 5'000'000);
  const auto sent = rig.link.sent();
  ASSERT_FALSE(sent.empty());
  for (size_t block = 0; block < 3; ++block) {
    EXPECT_EQ(rawKpKd(broadcastBlock(sent.back(), block)), std::make_pair(0U, 0U)) << block;
  }
}

TEST(BusStreamer, FailedZeroGainSendStopsAsASendFailure)
{
  Rig rig({1});
  std::atomic<int64_t> stamp{0};
  auto c = config(1ms, true);
  c.max_command_age = 5ms;
  c.command_written_ns = &stamp;
  // Only the zero-gain broadcast fails to send; the gain commands before it go out.
  rig.link.reject = [](const CanFrame & frame) {
      return frame.id == 0x7FFU && rawKpKd(broadcastBlock(frame, 0)).first == 0U;
    };
  auto streamer = rig.makeStreamer(c, gainHook());
  CommandWriter writer(stamp, 1ms);
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(10ms));
  std::this_thread::sleep_for(20ms);
  writer.stop();
  ASSERT_TRUE(rig.waitForStop(1s).has_value());
  streamer->stop();
  const auto stats = streamer->stats();
  // The zero-gain send failed in the stale cycle, so it stops as a send failure.
  EXPECT_EQ(stats.fault, StreamFault::kSendFailed);
  EXPECT_GT(stats.max_command_age_ms, 5.0);
  EXPECT_EQ(stats.command_stale_zero_gain_frames, 0U);
  // A send failure is a communication fault: the motor is latched for the best-effort disable.
  EXPECT_EQ(rig.drivers[0]->state(), ActuatorState::kSafeStopped);
}

bool isZeroGainUnicast(const CanFrame & frame, uint32_t id)
{
  return frame.id == id && frame.dlc == 8 && rawKpKd(frame.data8()) == std::make_pair(0U, 0U);
}

TEST(BusStreamer, ZeroGainSendFailurePartWayRecordsTheFramesSent)
{
  Rig rig({1, 2, 3});
  std::atomic<int64_t> stamp{0};
  // Motor 1's zero-gain frame goes out; motor 2's fails.
  rig.link.reject = [](const CanFrame & frame) {return isZeroGainUnicast(frame, 2);};
  auto c = config(1ms, false);
  c.max_command_age = 5ms;
  c.command_written_ns = &stamp;
  auto streamer = rig.makeStreamer(c, gainHook());
  CommandWriter writer(stamp, 1ms);
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(10ms));
  std::this_thread::sleep_for(20ms);
  writer.stop();
  ASSERT_TRUE(rig.waitForStop(1s).has_value());
  streamer->stop();
  const auto stats = streamer->stats();
  EXPECT_EQ(stats.fault, StreamFault::kSendFailed);
  EXPECT_EQ(stats.fault_axis, 1);
  EXPECT_EQ(stats.command_stale_zero_gain_frames, 1U);
  EXPECT_EQ(stats.command_stale_expected_frames, 3U);
  // Motor 2's failed send latches its safe-stop, which sends a disable afterwards, so look at the
  // last frame for motor 1 rather than the last frame overall.
  const auto sent = rig.link.sent();
  const auto last_to_1 = std::find_if(sent.rbegin(), sent.rend(),
      [](const CanFrame & frame) {return frame.id == 1U;});
  ASSERT_NE(last_to_1, sent.rend());
  EXPECT_TRUE(isZeroGainUnicast(*last_to_1, 1));
}

// A stop request that arrives while the zero-gain frames go out owns the shutdown: the stream
// ends as an external stop, not as a command-age fault.
class StopDuringZeroGain : public ::testing::TestWithParam<bool> {};

TEST_P(StopDuringZeroGain, EndsAsAnExternalStop)
{
  const bool other_bus_fault = GetParam();
  Rig rig({1, 2, 3});
  std::atomic<int64_t> stamp{0};
  std::atomic<BxiBusStreamer *> target{nullptr};
  // Motor 1's zero-gain frame goes out, then the stop request arrives before motor 2's frame.
  rig.link.reject = [&](const CanFrame & frame) {
      if (isZeroGainUnicast(frame, 1)) {
        if (other_bus_fault) {
          rig.stop_all = true;
        } else if (auto * streamer = target.load()) {
          streamer->requestStop();
        }
      }
      return false;
    };
  auto c = config(1ms, false);
  c.max_command_age = 5ms;
  c.command_written_ns = &stamp;
  auto streamer = rig.makeStreamer(c, gainHook());
  target = streamer.get();
  CommandWriter writer(stamp, 1ms);
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(10ms));
  std::this_thread::sleep_for(20ms);
  writer.stop();
  const auto deadline = clock_type::now() + 1s;
  while (clock_type::now() < deadline && !streamer->sendFinished()) {
    std::this_thread::sleep_for(100us);
  }
  ASSERT_TRUE(streamer->sendFinished());
  streamer->stop();
  const auto stats = streamer->stats();
  EXPECT_EQ(stats.fault, StreamFault::kNone);
  EXPECT_TRUE(stats.stopped_externally);
  EXPECT_EQ(stats.command_stale_zero_gain_frames, 1U);
  EXPECT_EQ(stats.command_stale_expected_frames, 3U);
  // Only the other bus's fault raised the shared stop flag; this stream did not.
  EXPECT_EQ(rig.stop_all.load(), other_bus_fault);
  EXPECT_TRUE(isZeroGainUnicast(rig.link.sent().back(), 1));
}

INSTANTIATE_TEST_SUITE_P(
  OwnerStopOrOtherBusFault, StopDuringZeroGain, ::testing::Values(false, true));

TEST(BusStreamer, CommandAgeLimitNeedsTwoPeriodsAndAWriteStamp)
{
  std::atomic<int64_t> stamp{0};
  {
    Rig rig({1});
    auto c = config(1ms, false);
    c.max_command_age = 5ms;  // no command_written_ns
    EXPECT_FALSE(rig.makeStreamer(c)->start());
  }
  {
    Rig rig({1});
    auto c = config(1ms, false);
    c.max_command_age = 1500us;
    c.command_written_ns = &stamp;
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
  EXPECT_FALSE(expired->command_valid); EXPECT_FALSE(expired->encoded_valid); // Unicast has no TX detail.
  EXPECT_FALSE(expired->send_attempted);
  EXPECT_EQ(expired->send_before_steady_ns, -1);
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

TEST(BusStreamer, PreparedTraceMovesStorageAndSnapshotsEffectiveCoordinates)
{
  Rig rig({1});
  rig.drivers[0]->setAngleOffset(-6.283185307179586);
  rig.drivers[0]->setAngleRange(-0.2, 0.2);
  const StreamCommand command{0.5, -0.1, 20.0, 1.0, 0.02};
  const auto encoded = rig.drivers[0]->encodeControlBlock(command.position, command.velocity,
    command.kp, command.kd, command.effort);
  ASSERT_TRUE(encoded.has_value());
  auto c = config(1ms, true); c.trace_cycles = 200; c.max_feedback_age = 50ms;
  auto streamer = rig.makeStreamer(c, [command](const auto &, auto & commands) {
      commands[0] = command; return true;
    });
  auto prepared = prepareStreamTrace(200, 1);
  const auto * storage = prepared->cycles.data();
  std::this_thread::sleep_for(5ms);
  const auto before_start = clock_type::now();
  ASSERT_TRUE(streamer->start(std::move(prepared)));
  ASSERT_TRUE(streamer->waitForFirstFeedback(20ms));
  std::this_thread::sleep_for(20ms); streamer->stop();
  const auto & trace = streamer->trace();
  EXPECT_EQ(trace.cycles.data(), storage);
  EXPECT_GE(trace.origin, before_start);
  EXPECT_GE(trace.send_loop_start_steady_ns, trace.origin.time_since_epoch().count());
  EXPECT_GT(trace.stream_stop_steady_ns, trace.send_loop_start_steady_ns);
  ASSERT_EQ(trace.axis_metadata.size(), 1U);
  EXPECT_DOUBLE_EQ(trace.axis_metadata[0].offset_rad_effective, -6.283185307179586);
  EXPECT_DOUBLE_EQ(trace.axis_metadata[0].lower_rad_effective, -0.2);
  EXPECT_DOUBLE_EQ(trace.axis_metadata[0].upper_rad_effective, 0.2);
  size_t sent = 0;
  for (size_t i = 0; i < trace.cycle_count; ++i) {
    const auto & r = trace.cycles[i];
    if (!r.send_attempted) {continue;}
    ++sent;
    EXPECT_TRUE(r.command_valid); EXPECT_TRUE(r.encoded_valid); EXPECT_TRUE(r.send_success);
    EXPECT_DOUBLE_EQ(r.command.position, 0.5);  // Before the encoder clamps it to 0.2.
    EXPECT_EQ(r.command_block, *encoded);
    EXPECT_GE(r.send_before_steady_ns, trace.send_loop_start_steady_ns);
    EXPECT_GE(r.send_after_steady_ns, r.send_before_steady_ns);
  }
  EXPECT_GT(sent, 0U);
  const auto decoded = decodeMotorReply(feedbackFrame(1), 1, rig.drivers[0]->limits());
  ASSERT_TRUE(decoded.has_value()); ASSERT_GT(trace.rx_count, 0U);
  for (size_t i = 0; i < trace.rx_count; ++i) {
    EXPECT_DOUBLE_EQ(trace.rx[i].motor_position, decoded->position);
    EXPECT_DOUBLE_EQ(trace.rx[i].motor_velocity, decoded->velocity);
    EXPECT_DOUBLE_EQ(trace.rx[i].motor_torque, decoded->torque);
  }
}

TEST(BusStreamer, InvalidPreparedShapeDoesNotStartThreadsOrSend)
{
  Rig rig({1});
  auto c = config(1ms, true); c.trace_cycles = 10;
  auto streamer = rig.makeStreamer(c);
  const auto before = rig.link.sent().size();
  EXPECT_FALSE(streamer->start(prepareStreamTrace(11, 1)));
  EXPECT_EQ(rig.link.sent().size(), before);
  EXPECT_EQ(streamer->error(), "prepared trace shape mismatch");
}

TEST(BusStreamer, FailedSendHasOneAttemptWithBothTimestamps)
{
  Rig rig({1});
  auto c = config(1ms, true); c.trace_cycles = 20;
  rig.link.send_ok = false;
  auto streamer = rig.makeStreamer(c, gainHook());
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(rig.waitForStop(100ms).has_value()); streamer->stop();
  const auto & trace = streamer->trace();
  size_t failed = 0;
  for (size_t i = 0; i < trace.cycle_count; ++i) {
    const auto & r = trace.cycles[i];
    if (!r.send_attempted) {continue;}
    ++failed;
    EXPECT_TRUE(r.command_valid); EXPECT_TRUE(r.encoded_valid); EXPECT_FALSE(r.send_success);
    EXPECT_EQ(r.outcome, StreamCycleOutcome::kStopped);
    EXPECT_GE(r.send_after_steady_ns, r.send_before_steady_ns);
  }
  EXPECT_EQ(failed, 1U);
  EXPECT_EQ(streamer->stats().fault, StreamFault::kSendFailed);
}

TEST(BusStreamer, SingleAxisCommandStaleRecordsReplacementAndBlock)
{
  Rig rig({1});
  std::atomic<int64_t> stamp{clock_type::now().time_since_epoch().count() - 100000000};
  auto c = config(1ms, true); c.trace_cycles = 20;
  c.command_written_ns = &stamp; c.max_command_age = 5ms;
  auto streamer = rig.makeStreamer(c, gainHook());
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(rig.waitForStop(100ms).has_value()); streamer->stop();
  ASSERT_EQ(streamer->stats().fault, StreamFault::kCommandStale);
  const auto & trace = streamer->trace(); ASSERT_EQ(trace.cycle_count, 1U);
  const auto & r = trace.cycles[0];
  EXPECT_TRUE(r.command_stale); EXPECT_TRUE(r.command_valid); EXPECT_TRUE(r.encoded_valid);
  EXPECT_TRUE(r.send_attempted); EXPECT_TRUE(r.send_success);
  EXPECT_DOUBLE_EQ(r.command.kp, 0.0); EXPECT_DOUBLE_EQ(r.command.kd, 0.0);
  EXPECT_DOUBLE_EQ(r.command.effort, 0.0);
  EXPECT_EQ(rawKpKd(r.command_block), std::make_pair(0U, 0U));
  EXPECT_EQ(r.outcome, StreamCycleOutcome::kStopped);
}

TEST(BusStreamer, EncodeFailureRecordsCommandButNoBlockOrSend)
{
  Rig rig({1});
  auto c = config(1ms, true); c.trace_cycles = 20;
  auto streamer = rig.makeStreamer(c, [](const auto &, auto & commands) {
      commands[0].position = std::numeric_limits<double>::quiet_NaN(); return true;
    });
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(rig.waitForStop(100ms).has_value()); streamer->stop();
  ASSERT_EQ(streamer->stats().fault, StreamFault::kEncodeFailed);
  const auto & trace = streamer->trace(); ASSERT_GT(trace.cycle_count, 0U);
  const auto & r = trace.cycles[trace.cycle_count - 1];
  EXPECT_TRUE(r.command_valid); EXPECT_FALSE(r.encoded_valid); EXPECT_FALSE(r.send_attempted);
  EXPECT_EQ(r.send_before_steady_ns, -1); EXPECT_EQ(r.send_after_steady_ns, -1);
}

TEST(BusStreamer, BroadcastExpiredCommandKeepsValuesWithoutSendTimestamps)
{
  Rig rig({1});
  auto c = config(1ms, true); c.trace_cycles = 200; c.max_feedback_age = 50ms;
  int calls = 0;
  auto streamer = rig.makeStreamer(c, [&calls](const auto &, auto & commands) {
      commands[0] = {0.1, 0.2, 20.0, 1.0, 0.0};
      if (++calls == 1) {std::this_thread::sleep_for(5ms);}
      return true;
    });
  ASSERT_TRUE(streamer->start());
  std::this_thread::sleep_for(20ms); streamer->stop();
  const auto & trace = streamer->trace();
  const auto end = trace.cycles.begin() + std::min(trace.cycle_count, trace.cycles.size());
  const auto row = std::find_if(trace.cycles.begin(), end, [](const auto & r) {
      return r.outcome == StreamCycleOutcome::kExpiredAfterHook;
    });
  ASSERT_NE(row, end);
  EXPECT_TRUE(row->command_valid); EXPECT_TRUE(row->encoded_valid); EXPECT_FALSE(row->send_attempted);
  EXPECT_DOUBLE_EQ(row->command.position, 0.1);
  EXPECT_EQ(row->send_before_steady_ns, -1); EXPECT_EQ(row->send_after_steady_ns, -1);
}

StreamConfig writeConfig()
{
  StreamConfig c;
  c.write_triggered = true; c.direct_feedback = true;
  c.period = 10ms; c.write_grace = 5ms;
  c.max_feedback_age = 30ms; c.max_command_age = 50ms;
  c.first_write_timeout = 50ms; c.trace_cycles = 200;
  c.broadcast = true;
  return c;
}
void publishBatch(BxiBusStreamer & streamer, size_t axes, uint64_t sequence, double kp = 10)
{
  std::vector<PublishedCommand> batch(axes);
  for (auto & value : batch) {
    value.sequence = sequence;
    value.published_ns = clock_type::now().time_since_epoch().count();
    value.command = {0.1, 0, kp, 1, 0};
  }
  ASSERT_TRUE(streamer.publishCommands(batch.data()));
}
TEST(BusStreamerWrite, ThreeMotorsPublishDirectFeedbackBeforeTheNextTx)
{
  Rig rig({1, 2, 3});
  rig.link.setDelays({100us});
  auto streamer = rig.makeStreamer(writeConfig());
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(100ms));
  // TX's next bootstrap is at 10 ms. RX is visible independently of that cycle.
  for (size_t axis = 0; axis < 3; ++axis) {
    PublishedFeedback feedback;
    ASSERT_TRUE(streamer->readFeedback(axis, feedback));
    EXPECT_TRUE(feedback.feedback.valid);
    EXPECT_GT(feedback.sequence, 0U);
  }
  streamer->armWrites();
  publishBatch(*streamer, 3, 1);
  std::this_thread::sleep_for(8ms);
  streamer->stop();
  EXPECT_EQ(streamer->stats().fault, StreamFault::kNone);
  EXPECT_EQ(streamer->stats().resends, 0U);
  EXPECT_EQ(rig.hook_calls, 0);
}
TEST(BusStreamerWrite, StalledWritesResendWithoutRefreshingCommandAge)
{
  Rig rig({1, 2, 3});
  rig.link.setDelays({100us});
  auto streamer = rig.makeStreamer(writeConfig());
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(100ms));
  streamer->armWrites();
  publishBatch(*streamer, 3, 1);
  ASSERT_TRUE(rig.waitForStop(500ms).has_value());
  streamer->stop();
  const auto stats = streamer->stats();
  EXPECT_EQ(stats.fault, StreamFault::kCommandStale);
  EXPECT_GT(stats.resends, 0U);
  EXPECT_EQ(stats.command_stale_zero_gain_frames, 1U);
  EXPECT_GE(stats.max_command_age_ms, 50.0);
}
TEST(BusStreamerWrite, PausedCommandWriterDoesNotReuseOldCommandsAsNew)
{
  Rig rig({1});
  auto storage = std::make_unique<WriteStreamStorage>(1);
  auto * raw = storage.get();
  auto c = writeConfig(); c.trace_cycles = 1000;
  auto streamer = rig.makeStreamer(c, nullptr, std::move(storage));
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(100ms));
  streamer->armWrites();
  publishBatch(*streamer, 1, 1);
  std::this_thread::sleep_for(8ms);
  // Keep genuine RX feedback coming, to isolate command acquisition from reply freshness.
  std::atomic<bool> injecting{true};
  std::thread injector([&] {
    while (injecting) {
      rig.link.inject(feedbackFrame(1));
      std::this_thread::sleep_for(1ms);
    }
  });
  AtomicSnapshotTestAccess::pauseWriter(raw->commands);
  streamer->notify();
  const auto stopped = rig.waitForStop(500ms);
  streamer->stop();
  injecting = false; injector.join();
  ASSERT_TRUE(stopped.has_value());
  EXPECT_EQ(streamer->stats().fault, StreamFault::kCommandSnapshotUnavailable);
  EXPECT_GT(streamer->stats().command_snapshot_failures, 0U);
  EXPECT_EQ(streamer->stats().resends, 0U);
  EXPECT_EQ(streamer->stats().command_stale_zero_gain_frames, 1U);
}

TEST(BusStreamerWrite, BroadcastFailureLatchesAllMotors)
{
  Rig rig({1, 2, 3});
  rig.link.reject = [](const CanFrame & frame) {return frame.id == 0x7FF;};
  auto streamer = rig.makeStreamer(writeConfig());
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(rig.waitForStop(500ms).has_value());
  streamer->stop();
  EXPECT_EQ(streamer->stats().fault, StreamFault::kSendFailed);
  for (const auto & driver : rig.drivers) {EXPECT_EQ(driver->state(), ActuatorState::kSafeStopped);}
}

TEST(BusStreamerWrite, FirstWriteTimeoutHasASeparateReason)
{
  Rig rig({1});
  auto streamer = rig.makeStreamer(writeConfig());
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(100ms));
  streamer->armWrites();
  ASSERT_TRUE(rig.waitForStop(500ms).has_value());
  streamer->stop();
  EXPECT_EQ(streamer->stats().fault, StreamFault::kStartupTimeout);
}

TEST(BusStreamerWrite, NotificationFailureCannotLeaveTheTxAsleep)
{
  Rig rig({1});
  auto storage = std::make_unique<WriteStreamStorage>(1);
  auto * raw = storage.get();
  auto streamer = rig.makeStreamer(writeConfig(), nullptr, std::move(storage));
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(100ms));
  streamer->armWrites();
  int descriptors[2]; ASSERT_EQ(::pipe(descriptors), 0);
  // Keep the descriptor number immutable, but make writing it fail with EBADF.
  const int replaced = ::dup2(descriptors[0], raw->event_fd);
  ::close(descriptors[0]); ::close(descriptors[1]);
  ASSERT_EQ(replaced, raw->event_fd);
  PublishedCommand packet; packet.sequence = 1;
  packet.published_ns = clock_type::now().time_since_epoch().count();
  EXPECT_FALSE(streamer->publishCommands(&packet));
  const auto stopped = rig.waitForStop(500ms);
  streamer->stop();
  ASSERT_TRUE(stopped.has_value());
  EXPECT_EQ(streamer->stats().fault, StreamFault::kNotificationFailed);
}

TEST(BusStreamerWrite, ForbiddenPaddedMotorReplyIsLatched)
{
  Rig rig({1});
  auto c = writeConfig(); c.padded_broadcast_blocks = 3; c.forbidden_reply_ids = {2, 3};
  auto streamer = rig.makeStreamer(c);
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(rig.waitForStop(500ms).has_value());
  streamer->stop();
  EXPECT_EQ(streamer->stats().fault, StreamFault::kForbiddenReply);
  EXPECT_GT(streamer->stats().forbidden_replies, 0U);
}

TEST(BusStreamerWrite, UnicastFailureRecordsThePartialBatch)
{
  Rig rig({1, 2, 3});
  rig.link.reject = [](const CanFrame & frame) {return frame.id == 2;};
  auto c = writeConfig(); c.broadcast = false;
  auto streamer = rig.makeStreamer(c);
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(rig.waitForStop(500ms).has_value());
  streamer->stop();
  EXPECT_EQ(streamer->stats().fault, StreamFault::kSendFailed);
  EXPECT_EQ(streamer->stats().partial_cycles, 1U);
  EXPECT_EQ(streamer->stats().frames_sent, 1U);
  ASSERT_GT(streamer->trace().cycle_count, 0U);
  EXPECT_EQ(streamer->trace().cycles[0].outcome, StreamCycleOutcome::kPartial);
  EXPECT_TRUE(streamer->trace().cycles[0].send_attempted);
  EXPECT_FALSE(streamer->trace().cycles[0].send_success);
}

TEST(BusStreamerWrite, NewWriteAfterFirstUnicastFrameDoesNotSplitTheBatch)
{
  Rig rig({1, 2, 3});
  std::atomic<bool> inject{false};
  auto c = writeConfig(); c.broadcast = false;
  auto streamer = rig.makeStreamer(c);
  rig.link.after_send = [&] {
    const auto frame = rig.link.sent().back();
    if (frame.id == 1 && !isZeroGainUnicast(frame, 1) && inject.exchange(false)) {
      publishBatch(*streamer, 3, 2, 20);
    }
  };
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(100ms));
  streamer->armWrites(); inject = true;
  publishBatch(*streamer, 3, 1, 10);
  std::this_thread::sleep_for(25ms);
  streamer->stop();
  EXPECT_FALSE(inject.load());
  EXPECT_EQ(streamer->stats().partial_cycles, 0U);
  for (uint64_t sequence : {1U, 2U}) {
    const auto & rows = streamer->trace().cycles;
    const auto found = std::find_if(rows.begin(), rows.end(), [sequence](const auto & row) {
      return row.send_kind == 2 && row.command_sequence == sequence;
    });
    ASSERT_NE(found, rows.end());
    EXPECT_EQ(found->frames, 3U);
    EXPECT_EQ(found->outcome, StreamCycleOutcome::kSent);
  }
}
TEST(BusStreamerWrite, MissingRepliesFaultEvenWhenWritesContinue)
{
  Rig rig({1});
  auto streamer = rig.makeStreamer(writeConfig());
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(100ms));
  streamer->armWrites();
  rig.link.setRespond(false);
  for (uint64_t seq = 1; seq < 50 && !rig.stop_all; ++seq) {
    publishBatch(*streamer, 1, seq);
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_TRUE(rig.waitForStop(500ms).has_value());
  streamer->stop();
  EXPECT_EQ(streamer->stats().fault, StreamFault::kFeedbackStale);
}
TEST(BusStreamerWrite, OvertemperatureCannotBeClearedByANormalReply)
{
  for (bool write_triggered : {true, false}) {
  Rig rig({1});
  auto c = writeConfig(); c.write_triggered = write_triggered;
  if (!write_triggered) {c.max_command_age = 0ns;}
  auto streamer = rig.makeStreamer(c);
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(100ms));
  auto frame = feedbackFrame(1);
  frame.data[6] = 0xff; frame.data[7] = 0xff;
  rig.link.inject(frame);
  rig.link.inject(feedbackFrame(1));
  ASSERT_TRUE(rig.waitForStop(500ms).has_value());
  streamer->stop();
  EXPECT_EQ(streamer->stats().fault, StreamFault::kTemperature);
  EXPECT_EQ(streamer->stats().temperature_fault_axis, 0);
  EXPECT_DOUBLE_EQ(streamer->stats().temperature_fault_c, 150.0);
  EXPECT_GT(streamer->stats().temperature_fault_received_ns, 0);
  }
}
TEST(BusStreamerWrite, BothSensorFailureDetailsSurviveRecoveryFrame)
{
  Rig rig({1});
  auto streamer = rig.makeStreamer(writeConfig());
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(100ms));
  auto frame = feedbackFrame(1); frame.data[6] = frame.data[7] = 0;
  rig.link.inject(frame); rig.link.inject(feedbackFrame(1));
  ASSERT_TRUE(rig.waitForStop(500ms).has_value());
  streamer->stop();
  EXPECT_EQ(streamer->stats().fault, StreamFault::kTemperature);
  EXPECT_EQ(streamer->stats().temperature_fault_axis, 0);
  EXPECT_TRUE(std::isnan(streamer->stats().temperature_fault_c));
  EXPECT_GT(streamer->stats().temperature_fault_received_ns, 0);
}
TEST(BusStreamerWrite, StopWakesAnIdleTxWithoutAwaitingItsWatchdog)
{
  Rig rig({1});
  auto c = writeConfig(); c.period = 100ms; c.write_grace = 50ms;
  c.max_feedback_age = 300ms; c.max_command_age = 500ms;
  auto streamer = rig.makeStreamer(c);
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(100ms));
  const auto elapsed = stopDurationMs(*streamer);
  EXPECT_LT(elapsed, 50.0);
  PublishedCommand packet;
  EXPECT_FALSE(streamer->publishCommands(&packet));
}

TEST(PairPublication, ThreePublishOutcomesAreDistinct)
{
  Rig rig({1, 2});
  auto storage = std::make_unique<WriteStreamStorage>(2);
  auto * raw = storage.get();
  auto streamer = rig.makeStreamer(writeConfig(), nullptr, std::move(storage));
  std::array<PublishedCommand, 2> batch{};
  EXPECT_EQ(streamer->publishCommandBatch(batch.data()), CommandPublishResult::kNotPublished);
  EXPECT_EQ(raw->commands.version(), 0U);
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(streamer->waitForFirstFeedback(100ms));
  streamer->armWrites();
  for (auto & packet : batch) {
    packet.sequence = 1; packet.published_ns = clock_type::now().time_since_epoch().count();
  }
  EXPECT_EQ(streamer->publishCommandBatch(batch.data()), CommandPublishResult::kSnapshotPublished);
  const auto version = raw->commands.version();
  int descriptors[2]; ASSERT_EQ(::pipe(descriptors), 0);
  ASSERT_EQ(::dup2(descriptors[0], raw->event_fd), raw->event_fd);
  ::close(descriptors[0]); ::close(descriptors[1]);
  for (auto & packet : batch) {packet.sequence = 2;}
  EXPECT_EQ(streamer->publishCommandBatch(batch.data()), CommandPublishResult::kSnapshotPublishedNotifyFailed);
  EXPECT_GT(raw->commands.version(), version);
  std::array<PublishedCommand, 2> read{}; ASSERT_TRUE(raw->commands.tryRead(read.data()));
  EXPECT_EQ(read[0].sequence, 2U); EXPECT_EQ(read[1].sequence, 2U);
  streamer->stop();
}
TEST(PairPublication, ArmUsesFixedActivationOrigin)
{
  Rig rig({1, 2}); auto config = writeConfig();
  config.activation_origin_ns = clock_type::now().time_since_epoch().count() - 1000000000;
  auto streamer = rig.makeStreamer(config);
  ASSERT_TRUE(streamer->start());
  ASSERT_TRUE(rig.waitForStop(500ms).has_value());  // G expires even before armWrites.
  streamer->armWrites(); streamer->stop();
  EXPECT_EQ(streamer->trace().writes_armed_steady_ns, config.activation_origin_ns);
  EXPECT_EQ(streamer->stats().fault, StreamFault::kStartupTimeout);
}

TEST(PairPublication, BootstrapUsesPreparedFeedbackNotUncommittedStage)
{
  Rig rig({1, 2}); auto config = writeConfig();
  auto storage = std::make_unique<WriteStreamStorage>(2);
  storage->tx_feedback[0].feedback.position = .25;
  storage->tx_feedback[0].feedback.valid = true;
  storage->tx_feedback[1].feedback.position = -.25;
  storage->tx_feedback[1].feedback.valid = true;
  auto streamer = rig.makeStreamer(config, nullptr, std::move(storage));
  const auto expected1 = rig.drivers[0]->encodeControlBlock(.25, 0, 0, 0, 0);
  const auto expected2 = rig.drivers[1]->encodeControlBlock(-.25, 0, 0, 0, 0);
  ASSERT_TRUE(expected1); ASSERT_TRUE(expected2);
  ASSERT_TRUE(streamer->start()); ASSERT_TRUE(streamer->waitForFirstFeedback(100ms));
  streamer->stop();
  const auto sent = rig.link.sent();
  const auto frame = std::find_if(sent.begin(), sent.end(), [](const auto & f) {return f.id == 0x7FF;});
  ASSERT_NE(frame, sent.end());
  EXPECT_TRUE(std::equal(expected1->begin(), expected1->end(), frame->data.begin()));
  EXPECT_TRUE(std::equal(expected2->begin(), expected2->end(), frame->data.begin() + 8));
}

// In-memory transports only. These tests make no SocketCAN calls.
TEST(PairN3Publication, BootstrapUsesAllThreePreparedPositions)
{
  Rig rig({1, 2, 3}); auto c = writeConfig();
  auto storage = std::make_unique<WriteStreamStorage>(3);
  const std::array<double, 3> positions{.25, -.25, .125};
  for (size_t axis = 0; axis < 3; ++axis) {
    storage->tx_feedback[axis].feedback.position = positions[axis];
    storage->tx_feedback[axis].feedback.valid = true;
  }
  auto streamer = rig.makeStreamer(c, nullptr, std::move(storage));
  ASSERT_TRUE(streamer->start()); ASSERT_TRUE(streamer->waitForFirstFeedback(100ms));
  streamer->stop();
  const auto sent = rig.link.sent();
  auto frame = std::find_if(sent.begin(), sent.end(), [](const auto & f) {return f.id == 0x7FF;});
  ASSERT_NE(frame, sent.end()); ASSERT_EQ(frame->dlc, 24);
  for (size_t axis = 0; axis < 3; ++axis) {
    const auto block = rig.drivers[axis]->encodeControlBlock(positions[axis], 0, 0, 0, 0);
    ASSERT_TRUE(block); EXPECT_TRUE(std::equal(block->begin(), block->end(), frame->data.begin() + 8 * axis));
  }
}
TEST(PairN3Publication, SnapshotEncodesAllThreeBlocksInOneFrame)
{
  Rig rig({1, 2, 3}); auto c = writeConfig();
  auto storage = std::make_unique<WriteStreamStorage>(3); auto * raw = storage.get();
  auto streamer = rig.makeStreamer(c, nullptr, std::move(storage));
  ASSERT_TRUE(streamer->start()); ASSERT_TRUE(streamer->waitForFirstFeedback(100ms));
  streamer->armWrites();
  std::array<PublishedCommand, 3> batch{};
  const auto published = clock_type::now().time_since_epoch().count();
  for (size_t axis = 0; axis < 3; ++axis) {
    batch[axis].command = {.1 * (axis + 1), 0, 10. + axis, 1, 0};
    batch[axis].sequence = 7; batch[axis].published_ns = published;
  }
  ASSERT_EQ(streamer->publishCommandBatch(batch.data()), CommandPublishResult::kSnapshotPublished);
  std::array<PublishedCommand, 3> adopted{}; ASSERT_TRUE(raw->commands.tryRead(adopted.data()));
  for (const auto & packet : adopted) {EXPECT_EQ(packet.sequence, 7U); EXPECT_EQ(packet.published_ns, published);}
  std::this_thread::sleep_for(20ms); streamer->stop();
  const auto frames = rig.link.sent(); bool found = false;
  for (const auto & frame : frames) {
    if (frame.id != 0x7FF || frame.dlc != 24) {continue;}
    bool matches = true;
    for (size_t axis = 0; axis < 3; ++axis) {
      const auto block = rig.drivers[axis]->encodeControlBlock(.1 * (axis + 1), 0, 10. + axis, 1, 0);
      ASSERT_TRUE(block); matches &= std::equal(block->begin(), block->end(), frame.data.begin() + axis * 8);
    }
    found |= matches;
  }
  EXPECT_TRUE(found);
  const auto & rows = streamer->trace().cycles;
  auto row = std::find_if(rows.begin(), rows.end(), [](const auto & r) {
      return r.send_kind == 2 && r.command_sequence == 7 && r.send_success;
    });
  ASSERT_NE(row, rows.end()); EXPECT_EQ(row->frames, 1U); EXPECT_EQ(row->command_published_ns, published);
}
TEST(PairN3Publication, ThirdAxisStaleUsesExistingFaultPath)
{
  Rig rig({1, 2, 3}); auto c = writeConfig();
  c.period = 1ms; c.write_grace = 200us; c.max_feedback_age = 3ms;
  c.max_command_age = 5ms; c.first_write_timeout = 1000ms;
  auto streamer = rig.makeStreamer(c);
  ASSERT_TRUE(streamer->start()); ASSERT_TRUE(streamer->waitForFirstFeedback(100ms));
  streamer->armWrites(); rig.link.setRespondingIds({1, 2});
  for (uint64_t generation = 1; generation < 200 && !rig.stop_all; ++generation) {
    // The streamer may fault between the loop condition and publication; that rejection is expected.
    std::array<PublishedCommand, 3> batch{};
    const auto stamp = clock_type::now().time_since_epoch().count();
    for (auto & packet : batch) {packet.sequence = generation; packet.published_ns = stamp;}
    streamer->publishCommandBatch(batch.data());
    std::this_thread::sleep_for(200us);
  }
  ASSERT_TRUE(rig.waitForStop(100ms).has_value()); streamer->stop();
  EXPECT_EQ(streamer->stats().fault, StreamFault::kFeedbackStale);
  EXPECT_EQ(streamer->stats().fault_axis, 2);
  EXPECT_EQ(rig.drivers[2]->state(), ActuatorState::kSafeStopped);
  // All-axis ERROR/safety delivery is outside b-1; do not assert it as implemented here.
}
}  // namespace
}  // namespace bxi_hardware
