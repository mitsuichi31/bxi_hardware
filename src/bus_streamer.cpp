#include "bxi_hardware/bus_streamer.hpp"
#include "bxi_hardware/stream_trace_io.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <set>
#include <utility>
#include <cmath>
#include <cerrno>
#include <stdexcept>
#include <sys/eventfd.h>
#include <sys/timerfd.h>
#include <unistd.h>
#include <poll.h>
#include "bxi_hardware/write_schedule.hpp"

#include <pthread.h>
#include <sched.h>

#include <rclcpp/rclcpp.hpp>

namespace bxi_hardware
{
namespace
{

rclcpp::Logger logger() {return rclcpp::get_logger("BxiBusStreamer");}

using clock_type = std::chrono::steady_clock;

double toMilliseconds(clock_type::duration duration)
{
  return std::chrono::duration<double, std::milli>(duration).count();
}

int64_t toNanoseconds(clock_type::duration duration)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

// Applies CPU affinity, then the scheduling policy, to the calling thread: SCHED_FIFO for a
// priority above 0, otherwise SCHED_OTHER explicitly, since a new thread inherits the policy of
// the thread that created it. Returns an empty string on success and the refused setting
// otherwise.
std::string applyThreadScheduling(int fifo_priority, const std::vector<int> & cpus)
{
  if (!cpus.empty()) {
    cpu_set_t set;
    CPU_ZERO(&set);
    std::string list;
    for (const int cpu : cpus) {
      CPU_SET(cpu, &set);
      list += (list.empty() ? "" : ",") + std::to_string(cpu);
    }
    const int result = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    if (result != 0) {
      return "CPU affinity " + list + ": " + std::strerror(result);
    }
  }
  sched_param param{};
  param.sched_priority = fifo_priority;
  const int policy = fifo_priority > 0 ? SCHED_FIFO : SCHED_OTHER;
  const int result = pthread_setschedparam(pthread_self(), policy, &param);
  if (result != 0) {
    return std::string(fifo_priority > 0 ? "SCHED_FIFO" : "SCHED_OTHER") + " priority " +
           std::to_string(fifo_priority) + ": " + std::strerror(result);
  }
  return "";
}

// Names the calling thread (at most 15 characters) and describes its scheduling, for example
// "SCHED_FIFO priority=60 cpus=2-3". Called once at thread start, never inside the cycle.
std::string nameAndDescribeThread(const std::string & name)
{
  pthread_setname_np(pthread_self(), name.substr(0, 15).c_str());
  int policy = 0;
  sched_param param{};
  std::string text = "unknown";
  if (pthread_getschedparam(pthread_self(), &policy, &param) == 0) {
    text = policy == SCHED_FIFO ? "SCHED_FIFO" : policy == SCHED_RR ? "SCHED_RR" :
      policy == SCHED_OTHER ? "SCHED_OTHER" : "policy=" + std::to_string(policy);
    text += " priority=" + std::to_string(param.sched_priority);
  }
  cpu_set_t cpus;
  CPU_ZERO(&cpus);
  if (pthread_getaffinity_np(pthread_self(), sizeof(cpus), &cpus) == 0) {
    std::string list;
    for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
      if (!CPU_ISSET(cpu, &cpus)) {
        continue;
      }
      int last = cpu;
      while (last + 1 < CPU_SETSIZE && CPU_ISSET(last + 1, &cpus)) {
        ++last;
      }
      list += (list.empty() ? "" : ",") + std::to_string(cpu) +
        (last > cpu ? "-" + std::to_string(last) : "");
      cpu = last;
    }
    text += " cpus=" + list;
  }
  return text;
}

}  // namespace

const char * streamCycleOutcomeName(StreamCycleOutcome outcome)
{
  switch (outcome) {
    case StreamCycleOutcome::kSent: return "sent";
    case StreamCycleOutcome::kSkippedLate: return "skipped_late";
    case StreamCycleOutcome::kSkippedBurst: return "skipped_burst";
    case StreamCycleOutcome::kExpiredAfterHook: return "expired_after_hook";
    case StreamCycleOutcome::kPartial: return "partial";
    case StreamCycleOutcome::kStopped: return "stopped";
  }
  return "unknown";
}

const char * streamFaultName(StreamFault fault)
{
  switch (fault) {
    case StreamFault::kNone: return "none";
    case StreamFault::kFeedbackStale: return "feedback stale";
    case StreamFault::kSendFailed: return "send failed";
    case StreamFault::kEncodeFailed: return "command not encodable";
    case StreamFault::kForbiddenReply: return "reply from a motor that must be absent";
    case StreamFault::kOwnerRequested: return "owner requested stop";
    case StreamFault::kCommandStale: return "command stale";
    case StreamFault::kCommandSnapshotUnavailable: return "command snapshot unavailable";
    case StreamFault::kStartupTimeout: return "first write timeout";
    case StreamFault::kTemperature: return "temperature fault";
    case StreamFault::kNotificationFailed: return "notification failed";
  }
  return "unknown";
}

BxiBusStreamer::BxiBusStreamer(
  CanBus * tx_bus, CanBus * rx_bus, std::vector<BxiActuatorDriver *> drivers,
  StreamConfig config, CycleHook hook, std::atomic<bool> * stop_all,
  std::unique_ptr<WriteStreamStorage> storage)
: tx_bus_(tx_bus), rx_bus_(rx_bus), drivers_(std::move(drivers)), config_(std::move(config)),
  hook_(std::move(hook)), stop_all_(stop_all), write_storage_(std::move(storage))
{
}

BxiBusStreamer::~BxiBusStreamer()
{
  stop();
}

std::optional<CanFrame> BxiBusStreamer::buildBroadcastFrame(
  const std::vector<std::pair<int, std::array<uint8_t, 8>>> & blocks, size_t block_count,
  const std::array<uint8_t, 8> & padding)
{
  const size_t length = block_count * 8;
  if (block_count == 0 || length > kCanFdMaxDataLength || !isValidCanFdDataLength(length)) {
    return std::nullopt;
  }
  CanFrame frame;
  frame.id = 0x7FFU;
  frame.extended = false;
  frame.dlc = static_cast<uint8_t>(length);
  for (size_t block = 0; block < block_count; ++block) {
    std::copy(padding.begin(), padding.end(), frame.data.begin() + block * 8);
  }
  for (const auto & [motor_id, bytes] : blocks) {
    if (motor_id < 1 || static_cast<size_t>(motor_id) > block_count) {
      return std::nullopt;
    }
    std::copy(bytes.begin(), bytes.end(), frame.data.begin() + (motor_id - 1) * 8);
  }
  return frame;
}

bool BxiBusStreamer::validate()
{
  if (tx_bus_ == nullptr || rx_bus_ == nullptr || stop_all_ == nullptr || !hook_ ||
    drivers_.empty() || config_.period <= std::chrono::nanoseconds::zero() ||
    config_.rx_poll_ms <= 0)
  {
    error_ = "incomplete stream setup";
    return false;
  }
  if (config_.trace_extended && (!config_.broadcast || drivers_.size() != 1)) {
    error_ = "expanded trace requires 1-axis broadcast";
    return false;
  }
  // Two missing or late replies in a row must not stop a healthy bus.
  const auto valid_priority = [](int priority) {return priority >= 0 && priority <= 98;};
  if (!valid_priority(config_.send_priority) || !valid_priority(config_.receive_priority) ||
    std::any_of(config_.cpus.begin(), config_.cpus.end(),
    [](int cpu) {return cpu < 0 || cpu >= CPU_SETSIZE;}))
  {
    error_ = "thread priority must be 0-98 and CPU indices must be valid";
    return false;
  }
  if (config_.max_feedback_age < 2 * config_.period) {
    error_ = "max feedback age must be at least two periods";
    return false;
  }
  if (config_.max_command_age.count() != 0 &&
    (config_.max_command_age < 2 * config_.period ||
      (!config_.write_triggered && config_.command_written_ns == nullptr)))
  {
    error_ = "max_command_age must be 0 (off), or at least two periods with command_written_ns";
    return false;
  }
  if (config_.write_triggered &&
    (config_.write_grace <= std::chrono::nanoseconds::zero() ||
    config_.period + config_.write_grace >= config_.max_feedback_age ||
    config_.period + config_.write_grace >= config_.max_command_age ||
    config_.first_write_timeout <= std::chrono::nanoseconds::zero()))
  {
    error_ = "write trigger needs positive grace/startup timeout and freshness margin";
    return false;
  }
  std::set<int> ids;
  for (const auto * driver : drivers_) {
    if (driver == nullptr || !ids.insert(driver->canId()).second) {
      error_ = "missing driver or duplicate motor ID";
      return false;
    }
  }
  for (const int id : config_.forbidden_reply_ids) {
    if (ids.count(id) != 0) {
      error_ = "a configured motor ID is also listed as forbidden";
      return false;
    }
  }
  if (!config_.broadcast) {
    if (config_.padded_broadcast_blocks != 0) {
      error_ = "padding requires broadcast";
      return false;
    }
    return true;
  }
  block_count_ = config_.padded_broadcast_blocks != 0 ?
    config_.padded_broadcast_blocks : drivers_.size();
  // Without padding every block position must belong to a configured motor (IDs 1..N).
  if (config_.padded_broadcast_blocks == 0 &&
    (*ids.begin() != 1 || static_cast<size_t>(*ids.rbegin()) != drivers_.size()))
  {
    error_ = "broadcast needs motor IDs 1..N on the bus";
    return false;
  }
  if (static_cast<size_t>(*ids.rbegin()) > block_count_ ||
    !isValidCanFdDataLength(block_count_ * 8) || block_count_ * 8 > kCanFdMaxDataLength)
  {
    error_ = "broadcast block count does not fit a CAN FD frame";
    return false;
  }
  // The padding matches the bxi_probe single-motor check: zero gains, zero torque.
  padding_block_ =
    protocol::encodeCommand(protocol::MitCommand{}, protocol::MitLimits{}).value();
  return true;
}

bool BxiBusStreamer::start(std::unique_ptr<StreamTrace> prepared)
{
  if (rx_thread_.joinable() || tx_thread_.joinable()) {
    error_ = "already started";
    return false;
  }
  if (!validate()) {
    RCLCPP_ERROR(logger(), "[%s] %s", tx_bus_ ? tx_bus_->name().c_str() : "?", error_.c_str());
    return false;
  }
  const size_t n = drivers_.size();
  can_ids_.clear();
  reply_ids_.clear();
  limits_.clear();
  for (const auto * driver : drivers_) {
    can_ids_.push_back(driver->canId());
    reply_ids_.push_back(protocol::defaultMasterId(static_cast<uint32_t>(driver->canId())));
    limits_.push_back(driver->limits());
  }
  if ((config_.write_triggered || config_.direct_feedback) && !write_storage_) {
    write_storage_ = std::make_unique<WriteStreamStorage>(n);
  }
  if (write_storage_ && write_storage_->feedback.size() != n) {
    error_ = "write storage shape mismatch";
    return false;
  }
  if (write_storage_) {
    for (size_t i = 0; i < n; ++i) {
      write_storage_->transforms[i] = drivers_[i]->feedbackTransform();
    }
  }
  writes_armed_ns_ = 0;
  rx_fault_ = StreamFault::kNone;
  rx_temperature_fault_c_ = 0.0; rx_temperature_fault_received_ns_ = 0;
  notification_failed_ = false;
  slots_.assign(n, ReplySlot{});
  forbidden_replies_ = 0;
  forbidden_reply_id_ = 0;
  rx_frames_ = 0;
  rx_ignored_ = 0;
  tx_stats_ = StreamStats{};
  tx_stats_.replies.assign(n, 0);
  tx_stats_.cycles_without_new_reply.assign(n, 0);
  tx_stats_.max_age_ms.assign(n, 0.0);
  try {
    if (!prepared) {
      prepared = prepareStreamTrace(config_.trace_cycles, n);
    }
    if (prepared->axes != n || prepared->cycles.size() != config_.trace_cycles ||
      prepared->age_ns.size() != config_.trace_cycles * n ||
      prepared->fresh.size() != config_.trace_cycles * n ||
      prepared->rx.size() != 2 * config_.trace_cycles * n ||
      prepared->cycle_count != 0 || prepared->rx_count != 0 ||
      prepared->axis_metadata.size() != n)
    {
      error_ = "prepared trace shape mismatch";
      return false;
    }
    trace_ = std::move(*prepared);
    for (size_t i = 0; i < n; ++i) {
      const auto * driver = drivers_[i];
      const auto range = driver->effectiveAngleRange();
      trace_.axis_metadata[i] = {driver->canId(), driver->motorDirection(),
        driver->effectiveAngleOffset(), range.first, range.second, driver->limits()};
    }
  } catch (const std::exception & e) {
    error_ = std::string("trace preparation: ") + e.what();
    return false;
  }
  // Allocation and effective-coordinate snapshot are complete before the receive clock begins.
  trace_.origin = clock_type::now();

  rx_state_ = kStarting;
  tx_state_ = kStarting;
  rx_setup_error_.clear();
  tx_setup_error_.clear();
  rx_running_ = true;
  rx_thread_ = std::thread(&BxiBusStreamer::receiveLoop, this);
  if (!waitForThread(rx_state_)) {
    // The setup error is read only after join, which orders it after the thread's write.
    stop();
    error_ = "receive thread: " + (rx_setup_error_.empty() ? "did not start in time" :
      rx_setup_error_);
    RCLCPP_ERROR(logger(), "[%s] %s", tx_bus_->name().c_str(), error_.c_str());
    return false;
  }
  tx_done_ = false;
  first_send_ns_ = 0;
  tx_running_ = true;
  tx_thread_ = std::thread(config_.write_triggered ?
    &BxiBusStreamer::sendWriteLoop : &BxiBusStreamer::sendLoop, this);
  if (!waitForThread(tx_state_)) {
    stop();
    error_ = "send thread: " + (tx_setup_error_.empty() ? "did not start in time" :
      tx_setup_error_);
    RCLCPP_ERROR(logger(), "[%s] %s", tx_bus_->name().c_str(), error_.c_str());
    return false;
  }
  return true;
}

bool BxiBusStreamer::waitForThread(const std::atomic<int> & state)
{
  const auto deadline = clock_type::now() + config_.rx_ready_timeout;
  while (state == kStarting && clock_type::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  return state == kReady;
}

bool BxiBusStreamer::waitForFirstFeedback(std::chrono::nanoseconds timeout)
{
  const auto deadline = clock_type::now() + timeout;
  while (clock_type::now() < deadline && !tx_done_) {
    const auto first_send_ns = first_send_ns_.load();
    if (first_send_ns != 0) {
      // A reply that arrived before the first streamed send does not count.
      const clock_type::time_point first_send{clock_type::duration(first_send_ns)};
      std::lock_guard<std::mutex> lock(slot_mutex_);
      if (std::all_of(slots_.begin(), slots_.end(),
        [first_send](const ReplySlot & slot) {
          return slot.sequence > 0 && slot.received_at >= first_send;
        }))
      {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  return false;
}

void BxiBusStreamer::stop()
{
  requestStop();
  if (tx_thread_.joinable()) {
    tx_thread_.join();
  }
  rx_running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  if (trace_.stream_stop_steady_ns < 0) {
    trace_.stream_stop_steady_ns = toNanoseconds(clock_type::now().time_since_epoch());
  }
}

StreamStats BxiBusStreamer::stats() const
{
  StreamStats stats = tx_stats_;
  stats.rx_frames = rx_frames_;
  stats.rx_ignored = rx_ignored_;
  stats.forbidden_replies = forbidden_replies_;
  stats.forbidden_reply_id = forbidden_reply_id_;
  if (rx_fault_.load() == StreamFault::kTemperature) {
    stats.temperature_fault_axis = rx_fault_axis_.load();
    stats.temperature_fault_c = rx_temperature_fault_c_;
    stats.temperature_fault_received_ns = rx_temperature_fault_received_ns_;
  }
  return stats;
}

void BxiBusStreamer::receiveLoop()
{
  const auto setup_error = applyThreadScheduling(config_.receive_priority, config_.cpus);
  if (!setup_error.empty()) {
    rx_setup_error_ = setup_error;  // read by start() only after join
    rx_state_ = kFailed;
    return;
  }
  trace_.rx_thread = nameAndDescribeThread("bxrx_" + rx_bus_->name());
  RCLCPP_INFO(logger(), "[%s] receive thread: %s", rx_bus_->name().c_str(),
    trace_.rx_thread.c_str());
  rx_state_ = kReady;
  CanFrame frame;
  // recv() returns within rx_poll_ms when idle and after each frame on a busy bus, so the stop
  // flag is seen without closing the socket from another thread.
  while (rx_running_) {
    if (!rx_bus_->recv(frame, config_.rx_poll_ms)) {
      continue;
    }
    ++rx_frames_;
    const auto received_at = clock_type::now();
    bool used = false;
    for (size_t i = 0; i < reply_ids_.size(); ++i) {
      if (frame.extended || frame.id != reply_ids_[i]) {
        continue;
      }
      const auto decoded = decodeMotorReply(frame, can_ids_[i], limits_[i]);
      if (decoded.has_value()) {
        {
        std::lock_guard<std::mutex> lock(slot_mutex_);
        auto & slot = slots_[i];
        slot.decoded = *decoded;
        slot.received_at = received_at;
        slot.rx_kernel_realtime_ns = frame.rx_kernel_realtime_ns;
        slot.rx_userspace_realtime_ns = frame.rx_userspace_realtime_ns;
        ++slot.sequence;
        }
        if (write_storage_) {
          auto & published = write_storage_->rx_cache[i];
          published.feedback = convertFeedback(*decoded, write_storage_->transforms[i]);
          published.feedback.rx_kernel_realtime_ns = frame.rx_kernel_realtime_ns;
          published.feedback.rx_userspace_realtime_ns = frame.rx_userspace_realtime_ns;
          ++published.sequence;
          published.received_ns = received_at.time_since_epoch().count();
          published.published_ns = clock_type::now().time_since_epoch().count();
          write_storage_->feedback[i]->publish(&published);
          write_storage_->received_ns[i].store(published.received_ns);
          if (!std::isfinite(published.feedback.temperature) ||
            published.feedback.temperature > 80.0)
          {
            if (rx_fault_.load() == StreamFault::kNone) {
              rx_temperature_fault_c_ = published.feedback.temperature;
              rx_temperature_fault_received_ns_ = published.received_ns;
              rx_fault_axis_ = static_cast<int>(i);
              rx_fault_ = StreamFault::kTemperature;
            }
            if (config_.write_triggered) {notify();}
          }
        }
        used = true;
      }
      if (decoded.has_value()) {
        if (trace_.rx_count < trace_.rx.size()) {
          auto & row = trace_.rx[trace_.rx_count];
          row.axis = static_cast<uint16_t>(i);
          row.received_ns = toNanoseconds(received_at - trace_.origin);
          row.rx_kernel_realtime_ns = frame.rx_kernel_realtime_ns;
          row.rx_userspace_realtime_ns = frame.rx_userspace_realtime_ns;
          row.rx_hardware_raw_ns = frame.rx_hardware_raw_ns;
          row.sequence = write_storage_ ? write_storage_->rx_cache[i].sequence : slots_[i].sequence;
          row.published_ns = write_storage_ ? write_storage_->rx_cache[i].published_ns :
            clock_type::now().time_since_epoch().count();
          row.motor_position = decoded->position;
          row.motor_velocity = decoded->velocity;
          row.motor_torque = decoded->torque;
        }
        ++trace_.rx_count;
      }
      break;
    }
    if (!used) {
      for (const int id : config_.forbidden_reply_ids) {
        if (decodeMotorReply(frame, id, protocol::MitLimits{}).has_value()) {
          std::lock_guard<std::mutex> lock(slot_mutex_);
          ++forbidden_replies_;
          forbidden_reply_id_ = id;
          if (rx_fault_.load() == StreamFault::kNone) {
            rx_fault_axis_ = -1;
            rx_fault_ = StreamFault::kForbiddenReply;
          }
          if (config_.write_triggered) {notify();}
          used = true;
          break;
        }
      }
    }
    if (!used) {
      ++rx_ignored_;
    }
  }
}

void BxiBusStreamer::fault(StreamFault kind, int axis)
{
  tx_stats_.fault = kind;
  tx_stats_.fault_axis = axis;
  stop_all_->store(true);
  const uint64_t one = 1;
  for (int fd : config_.stop_wake_fds) {
    const auto notified = ::write(fd, &one, sizeof(one));
    if (notified < 0 && errno != EAGAIN) {notification_failed_ = true;}
  }
  if (config_.write_triggered) {return;}  // Non-RT monitor reports the latched reason after disable.
  if (axis >= 0) {
    RCLCPP_ERROR(logger(), "[%s] motor id=%d: %s; stopping all motors",
      tx_bus_->name().c_str(), can_ids_[static_cast<size_t>(axis)], streamFaultName(kind));
  } else {
    RCLCPP_ERROR(logger(), "[%s] %s; stopping all motors", tx_bus_->name().c_str(),
      streamFaultName(kind));
  }
}

void BxiBusStreamer::sendLoop()
{
  const auto setup_error = applyThreadScheduling(config_.send_priority, config_.cpus);
  if (!setup_error.empty()) {
    tx_setup_error_ = setup_error;  // read by start() only after join
    tx_state_ = kFailed;
    tx_done_ = true;
    return;
  }
  trace_.tx_thread = nameAndDescribeThread("bxtx_" + tx_bus_->name());
  RCLCPP_INFO(logger(), "[%s] send thread: %s", tx_bus_->name().c_str(),
    trace_.tx_thread.c_str());
  tx_state_ = kReady;
  const size_t n = drivers_.size();
  const auto period = config_.period;
  std::vector<StreamFeedback> feedback(n);
  std::vector<StreamCommand> commands(n);
  std::vector<std::pair<int, std::array<uint8_t, 8>>> blocks(n);
  std::vector<ReplySlot> latest(n);
  std::vector<uint64_t> seen_sequence(n, 0);
  std::vector<bool> fresh(n, false);
  std::vector<clock_type::time_point> last_reply(n);
  std::vector<bool> has_reply(n, false);
  uint64_t forbidden_seen = 0;
  int forbidden_id = 0;
  clock_type::time_point first_send{};
  clock_type::time_point last_send{};
  bool sent_any = false;
  bool sent_previous_cycle = false;

  const auto start = clock_type::now();
  trace_.send_loop_start_steady_ns = toNanoseconds(start.time_since_epoch());
  for (uint64_t k = 0; tx_running_; ++k) {
    const auto target = start + k * period;
    std::this_thread::sleep_until(target);
    if (!tx_running_) {
      break;
    }
    if (stop_all_->load()) {
      tx_stats_.stopped_externally = true;
      break;
    }
    const auto now = clock_type::now();
    // Trace row for this cycle, or nullptr when tracing is off or the trace is full.
    const size_t row_index = trace_.cycle_count++;
    StreamCycleRecord * row =
      row_index < trace_.cycles.size() ? &trace_.cycles[row_index] : nullptr;
    if (row != nullptr) {
      *row = StreamCycleRecord{};
      row->cycle = k;
      row->wake_lateness_ns = toNanoseconds(now - target);
    }

    {
      std::lock_guard<std::mutex> lock(slot_mutex_);
      for (size_t i = 0; i < n; ++i) {
        fresh[i] = slots_[i].sequence != seen_sequence[i];
        if (fresh[i]) {
          latest[i] = slots_[i];
          seen_sequence[i] = slots_[i].sequence;
        }
      }
      forbidden_seen = forbidden_replies_;
      forbidden_id = forbidden_reply_id_;
    }

    bool faulted = false;
    for (size_t i = 0; i < n; ++i) {
      feedback[i].updated = fresh[i];
      if (fresh[i]) {
        auto applied = drivers_[i]->applyFeedback(latest[i].decoded);
        applied.rx_kernel_realtime_ns = latest[i].rx_kernel_realtime_ns;
        applied.rx_userspace_realtime_ns = latest[i].rx_userspace_realtime_ns;
        feedback[i].feedback = applied;
        last_reply[i] = latest[i].received_at;
        has_reply[i] = true;
        tx_stats_.replies[i] = seen_sequence[i];
        feedback[i].sequence = seen_sequence[i];
        feedback[i].received_ns = latest[i].received_at.time_since_epoch().count();
      } else if (sent_previous_cycle) {
        ++tx_stats_.cycles_without_new_reply[i];
      }
      if (row != nullptr) {
        trace_.fresh[row_index * n + i] = fresh[i] ? 1 : 0;
        trace_.age_ns[row_index * n + i] = -1;
      }
      if (!sent_any) {
        continue;
      }
      const auto age = now - (has_reply[i] ? last_reply[i] : first_send);
      if (row != nullptr) {
        trace_.age_ns[row_index * n + i] = toNanoseconds(age);
      }
      feedback[i].age_ms = toMilliseconds(age);
      tx_stats_.max_age_ms[i] = std::max(tx_stats_.max_age_ms[i], feedback[i].age_ms);
      if (!faulted && age > config_.max_feedback_age) {
        drivers_[i]->latchCommunicationFault();
        fault(StreamFault::kFeedbackStale, static_cast<int>(i));
        faulted = true;
      }
    }
    if (!faulted && config_.direct_feedback && rx_fault_.load() == StreamFault::kTemperature) {
      fault(StreamFault::kTemperature, rx_fault_axis_.load());
      faulted = true;
    }
    if (!faulted && forbidden_seen > 0) {
      tx_stats_.forbidden_replies = forbidden_seen;
      tx_stats_.forbidden_reply_id = forbidden_id;
      fault(StreamFault::kForbiddenReply, -1);
      faulted = true;
    }
    const auto set_outcome = [row](StreamCycleOutcome outcome) {
        if (row != nullptr) {
          row->outcome = outcome;
        }
      };
    if (faulted) {
      set_outcome(StreamCycleOutcome::kStopped);
      break;
    }

    // A stalled owner loop leaves its last commands in place while replies keep arriving. When
    // the latest write is too old, send zero-gain, zero-torque commands in this cycle instead of
    // asking the hook, then stop. Checked in every cycle, including those that would be skipped
    // below, and the zero-gain frames ignore the slot and burst rules: a late zero-gain command is
    // still safe to send.
    bool command_stale = false;
    const int64_t last_command_ns = config_.max_command_age.count() > 0 ?
      config_.command_written_ns->load(std::memory_order_relaxed) : 0;
    if (last_command_ns != 0) {
      const auto command_age = now - clock_type::time_point(clock_type::duration(last_command_ns));
      if (row != nullptr) {
        row->command_age_ns = toNanoseconds(command_age);
      }
      tx_stats_.max_command_age_ms =
        std::max(tx_stats_.max_command_age_ms, toMilliseconds(command_age));
      command_stale = command_age > config_.max_command_age;
    }

    // Send nothing for a slot that is already a full period late, and never two frames within
    // half a period, so a stalled thread does not catch up in a burst.
    const bool late = now >= target + period;
    if (!command_stale && (late || (sent_any && now - last_send < period / 2))) {
      set_outcome(late ? StreamCycleOutcome::kSkippedLate : StreamCycleOutcome::kSkippedBurst);
      ++tx_stats_.skipped_cycles;
      sent_previous_cycle = false;
      continue;
    }

    if (command_stale) {
      for (size_t i = 0; i < n; ++i) {
        commands[i] = StreamCommand{feedback[i].feedback.position, 0.0, 0.0, 0.0, 0.0};
      }
    } else {
      const auto hook_start = clock_type::now();
      const bool hook_ok = hook_(feedback, commands);
      if (row != nullptr) {
        row->hook_ns = toNanoseconds(clock_type::now() - hook_start);
        row->command_sequence = commands[0].sequence;
        row->command_published_ns = commands[0].published_ns;
        row->send_kind = 2;
      }
      if (!hook_ok) {
        set_outcome(StreamCycleOutcome::kStopped);
        fault(StreamFault::kOwnerRequested, -1);
        break;
      }
    }
    if (row != nullptr && config_.broadcast && n == 1) {
      row->command = {commands[0].position, commands[0].velocity, commands[0].kp,
        commands[0].kd, commands[0].effort};
      row->command_valid = true;
      row->command_stale = command_stale;
    }
    for (size_t i = 0; i < n && !faulted; ++i) {
      const auto & c = commands[i];
      const auto block =
        drivers_[i]->encodeControlBlock(c.position, c.velocity, c.kp, c.kd, c.effort);
      if (!block.has_value()) {
        fault(StreamFault::kEncodeFailed, static_cast<int>(i));
        faulted = true;
        break;
      }
      blocks[i] = {can_ids_[i], *block};
      if (row != nullptr && config_.broadcast && n == 1) {
        row->command_block = *block;
        row->encoded_valid = true;
      }
    }
    if (faulted) {
      set_outcome(StreamCycleOutcome::kStopped);
      break;
    }

    // The hook may block, for example on the owner's data mutex. Check again before every frame
    // so that no command leaves after its slot has expired or after a stop was requested.
    const auto may_send = [&]() {
        return tx_running_ && !stop_all_->load() &&
               (command_stale || clock_type::now() < target + period);
      };
    size_t frames_this_cycle = 0;
    bool window_closed = false;
    clock_type::time_point send_time{};
    if (config_.broadcast) {
      if (!may_send()) {
        window_closed = true;
      } else {
        send_time = clock_type::now();
        const auto frame = buildBroadcastFrame(blocks, block_count_, padding_block_);
        bool success = false;
        if (frame.has_value()) {
          if (row != nullptr && (n == 1 || config_.control_trace)) {
            row->send_attempted = true;
            row->send_before_steady_ns = toNanoseconds(clock_type::now().time_since_epoch());
          }
          success = tx_bus_->send(*frame);
          if (row != nullptr && (n == 1 || config_.control_trace)) {
            row->send_after_steady_ns = toNanoseconds(clock_type::now().time_since_epoch());
            row->send_success = success;
          }
        }
        if (!success) {
          for (auto * driver : drivers_) {
            driver->latchCommunicationFault();
          }
          fault(StreamFault::kSendFailed, -1);
          faulted = true;
        } else {
          ++frames_this_cycle;
        }
      }
    } else {
      for (size_t i = 0; i < n; ++i) {
        if (!may_send()) {
          window_closed = true;
          break;
        }
        if (frames_this_cycle == 0) {
          send_time = clock_type::now();
        }
        CanFrame frame;
        frame.id = static_cast<uint32_t>(can_ids_[i]);
        frame.extended = false;
        frame.setData(blocks[i].second);
        if (!tx_bus_->send(frame)) {
          drivers_[i]->latchCommunicationFault();
          fault(StreamFault::kSendFailed, static_cast<int>(i));
          faulted = true;
          break;
        }
        ++frames_this_cycle;
      }
    }
    tx_stats_.frames_sent += frames_this_cycle;
    if (command_stale) {
      // Recorded before any exit below, including a send failure part-way through the motors.
      tx_stats_.command_stale_zero_gain_frames = frames_this_cycle;
      tx_stats_.command_stale_expected_frames = config_.broadcast ? 1U : n;
    }
    if (row != nullptr) {
      row->frames = static_cast<uint8_t>(frames_this_cycle);
      if (frames_this_cycle > 0) {
        row->send_lateness_ns = toNanoseconds(send_time - target);
      }
    }
    if (frames_this_cycle > 0) {
      if (!sent_any) {
        first_send = send_time;
        sent_any = true;
        first_send_ns_ = send_time.time_since_epoch().count();
      }
      last_send = send_time;
    }
    if (faulted) {
      set_outcome(StreamCycleOutcome::kStopped);
      break;
    }
    if (command_stale) {
      set_outcome(StreamCycleOutcome::kStopped);
      if (window_closed) {
        // A stale cycle ignores the slot deadline, so only a stop request (the owner's stop, or
        // another bus's fault) closes it. That stop owns the shutdown: end as an external stop
        // without raising a fault of our own.
        tx_stats_.stopped_externally = true;
      } else {
        fault(StreamFault::kCommandStale, -1);
      }
      break;
    }
    if (window_closed) {
      if (stop_all_->load()) {
        set_outcome(StreamCycleOutcome::kStopped);
        tx_stats_.stopped_externally = true;
        break;
      }
      if (frames_this_cycle == 0) {
        set_outcome(StreamCycleOutcome::kExpiredAfterHook);
        ++tx_stats_.skipped_cycles;
      } else {
        set_outcome(StreamCycleOutcome::kPartial);
        ++tx_stats_.partial_cycles;
      }
      sent_previous_cycle = frames_this_cycle > 0;
      continue;
    }
    sent_previous_cycle = true;
    ++tx_stats_.cycles;
  }
  {
    std::lock_guard<std::mutex> lock(slot_mutex_);
    for (size_t i = 0; i < n; ++i) {
      tx_stats_.replies[i] = slots_[i].sequence;
    }
  }
  tx_done_ = true;
}

WriteStreamStorage::WriteStreamStorage(size_t axes)
: commands(axes), received_ns(std::make_unique<std::atomic<int64_t>[]>(axes)),
  control_unavailable(std::make_unique<std::atomic<bool>[]>(axes)), tx_seen_feedback(axes),
  tx_cache(axes), tx_scratch(axes), rx_cache(axes), tx_feedback(axes), transforms(axes),
  tx_commands(axes), tx_blocks(axes)
{
  static_assert(clock_type::period::num == 1 && clock_type::period::den == 1000000000,
    "write stream requires nanosecond steady_clock ticks");
  // timerfd absolute deadlines and the published timestamps must share an epoch.
  const auto before = clock_type::now().time_since_epoch().count();
  timespec monotonic{};
  const int clock_result = ::clock_gettime(CLOCK_MONOTONIC, &monotonic);
  const auto after = clock_type::now().time_since_epoch().count();
  const int64_t clock_ns = monotonic.tv_sec * 1000000000LL + monotonic.tv_nsec;
  if (clock_result != 0 || clock_ns < before || clock_ns > after) {
    throw std::runtime_error("steady_clock and CLOCK_MONOTONIC have different epochs");
  }
  if (!commands.lockFree() || !received_ns[0].is_lock_free() ||
    !control_unavailable[0].is_lock_free()) {
    throw std::runtime_error("stream atomics are not lock free");
  }
  for (size_t i = 0; i < axes; ++i) {
    feedback.push_back(std::make_unique<AtomicSnapshot<PublishedFeedback>>());
    if (!feedback.back()->lockFree()) {throw std::runtime_error("feedback atomics not lock free");}
    received_ns[i].store(0);
    control_unavailable[i].store(false);
  }
  event_fd = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  timer_fd = ::timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
  if (event_fd < 0 || timer_fd < 0) {
    if (event_fd >= 0) {::close(event_fd);}
    if (timer_fd >= 0) {::close(timer_fd);}
    throw std::runtime_error("stream eventfd/timerfd creation failed");
  }
}
WriteStreamStorage::~WriteStreamStorage()
{
  if (event_fd >= 0) {::close(event_fd);}
  if (timer_fd >= 0) {::close(timer_fd);}
}
void BxiBusStreamer::notify()
{
  if (!write_storage_) {return;}
  const uint64_t one = 1;
  if (::write(write_storage_->event_fd, &one, sizeof(one)) != sizeof(one) && errno != EAGAIN) {
    notification_failed_ = true;
  }
}
void BxiBusStreamer::requestStop()
{
  tx_running_ = false;
  notify();
}
void BxiBusStreamer::armWrites()
{
  trace_.writes_armed_steady_ns = clock_type::now().time_since_epoch().count();
  writes_armed_ns_ = trace_.writes_armed_steady_ns;
  notify();
}
bool BxiBusStreamer::publishCommands(const PublishedCommand * commands)
{
  if (!config_.write_triggered || !writes_armed_ns_.load() ||
    !tx_running_ || stop_all_->load()) {return false;}
  write_storage_->commands.publish(commands);
  notify();
  return !notification_failed_.load();
}
bool BxiBusStreamer::readFeedback(size_t axis, PublishedFeedback & scratch,
  unsigned * attempts) const
{
  return write_storage_ && axis < write_storage_->feedback.size() &&
         write_storage_->feedback[axis]->tryRead(&scratch, attempts);
}

void BxiBusStreamer::setControlUnavailable(size_t axis, bool unavailable)
{
  if (write_storage_ && axis < write_storage_->feedback.size()) {
    write_storage_->control_unavailable[axis].store(unavailable);
  }
}
bool BxiBusStreamer::controlUnavailable(size_t axis) const
{
  return write_storage_ && axis < write_storage_->feedback.size() &&
         write_storage_->control_unavailable[axis].load();
}

void BxiBusStreamer::sendWriteLoop()
{
  const auto setup = applyThreadScheduling(config_.send_priority, config_.cpus);
  if (!setup.empty()) {
    tx_setup_error_ = setup; tx_state_ = kFailed; tx_done_ = true; return;
  }
  trace_.tx_thread = nameAndDescribeThread("bxtx_" + tx_bus_->name());
  auto & storage = *write_storage_;
  const size_t n = drivers_.size();
  auto & commands = storage.tx_commands;
  auto & feedback = storage.tx_feedback;
  auto & blocks = storage.tx_blocks;
  WriteSchedule schedule{config_.period.count(), config_.write_grace.count()};
  const auto ns_now = [] {return clock_type::now().time_since_epoch().count();};
  trace_.send_loop_start_steady_ns = ns_now();
  int64_t bootstrap_due = ns_now(), first_send = 0, retry_due = 0, hold_start = 0;
  uint64_t observed_version = 0;
  int last_send_kind = 0;
  bool snapshot_failed = false;
  tx_state_ = kReady;
  while (tx_running_) {
    int64_t now = ns_now();
    if (stop_all_->load()) {tx_stats_.stopped_externally = true; break;}
    if (notification_failed_) {fault(StreamFault::kNotificationFailed, -1); break;}
    // Take a whole command batch, never extend freshness with a torn/failed read.
    const auto version = storage.commands.version();
    if (version != observed_version || snapshot_failed) {
      snapshot_failed = !storage.commands.tryRead(storage.tx_scratch.data());
      if (!snapshot_failed) {
        const auto & packet = storage.tx_scratch[0];
        if (packet.sequence != schedule.sequence) {
          if (schedule.last_was_resend && packet.published_ns >= schedule.last_send_ns &&
            packet.published_ns < schedule.earliestSend()) {++tx_stats_.writes_after_resend;}
          if (schedule.sequence && packet.sequence > schedule.sequence + 1) {
            tx_stats_.command_coalesced += packet.sequence - schedule.sequence - 1;
          }
          storage.tx_cache.swap(storage.tx_scratch);
          schedule.published(packet.sequence, packet.published_ns);
        }
        observed_version = version;
        retry_due = 0;
      } else {
        ++tx_stats_.command_snapshot_failures;
        retry_due = now + std::max(int64_t{1}, config_.period.count() / 20);
      }
    }
    const bool active = schedule.sequence != 0;
    const auto armed = writes_armed_ns_.load();
    const bool startup_expired = !active && armed &&
      now >= armed + config_.first_write_timeout.count();
    const bool stale = active && now >= schedule.published_ns + config_.max_command_age.count();
    // Safe-stop independent of write notifications. Command expiry takes precedence.
    StreamFault stop_reason = stale ? (snapshot_failed ? StreamFault::kCommandSnapshotUnavailable :
      StreamFault::kCommandStale) : StreamFault::kNone;
    int stop_axis = -1;
    if (!stale && startup_expired) {stop_reason = StreamFault::kStartupTimeout;}
    if (!stale && rx_fault_.load() != StreamFault::kNone) {
      stop_reason = rx_fault_.load(); stop_axis = rx_fault_axis_.load();
    }
    int64_t feedback_due = now + config_.max_feedback_age.count();
    for (size_t i = 0; i < n; ++i) {
      PublishedFeedback scratch;
      if (storage.feedback[i]->tryRead(&scratch)) {feedback[i] = scratch;}
      const auto received = storage.received_ns[i].load();
      const auto base = received ? received : first_send;
      if (base) {
        feedback_due = std::min(feedback_due, base + config_.max_feedback_age.count());
        tx_stats_.max_age_ms[i] = std::max(tx_stats_.max_age_ms[i], (now - base) / 1.e6);
        if (now >= base + config_.max_feedback_age.count()) {
          tx_stats_.feedback_stale_at_stop = true;
          if (stop_reason == StreamFault::kNone) {
            stop_reason = StreamFault::kFeedbackStale; stop_axis = static_cast<int>(i);
          }
        }
      }
      tx_stats_.replies[i] = feedback[i].sequence;
    }
    if (active) {
      tx_stats_.max_command_age_ms = std::max(tx_stats_.max_command_age_ms,
        (now - schedule.published_ns) / 1.e6);
    }
    const bool zero_stop = stop_reason == StreamFault::kCommandStale ||
      stop_reason == StreamFault::kCommandSnapshotUnavailable ||
      stop_reason == StreamFault::kStartupTimeout;
    if (stop_reason != StreamFault::kNone) {tx_stats_.received_fault_at_stop = rx_fault_.load();}
    if (stop_reason != StreamFault::kNone && !zero_stop) {
      if (stop_reason == StreamFault::kFeedbackStale && stop_axis >= 0) {
        drivers_[stop_axis]->latchCommunicationFault(false);
      }
      fault(stop_reason, stop_axis); break;
    }
    bool resend = active && !snapshot_failed && schedule.resendDue(now);
    bool due = zero_stop || (!active && now >= bootstrap_due) ||
      (active && !snapshot_failed && (schedule.pending() || resend));
    if (due && !zero_stop && schedule.held(now)) {
      if (!hold_start) {
        hold_start = now; ++tx_stats_.minimum_interval_holds;
        if (active && schedule.pending()) {
          if (last_send_kind == 3) {++tx_stats_.resend_command_holds;}
          else if (last_send_kind == 2) {++tx_stats_.new_command_holds;}
          else {++tx_stats_.bootstrap_command_holds;}
        }
      }
      due = false;
    }
    if (due) {
      const auto row_index = trace_.cycle_count++;
      auto * row = row_index < trace_.cycles.size() ? &trace_.cycles[row_index] : nullptr;
      if (row) {
        *row = StreamCycleRecord{};
        row->cycle = row_index; row->wake_lateness_ns = -1; row->send_lateness_ns = -1;
        row->wake_steady_ns = now; row->command_sequence = schedule.sequence;
        row->command_published_ns = schedule.published_ns; row->resend_due_ns = schedule.resend_ns;
        row->command_age_ns = active ? now - schedule.published_ns : -1;
        row->send_kind = zero_stop ? 4 : !active ? 1 : resend ? 3 : 2;
        row->snapshot_failed = snapshot_failed;
        row->hold_ns = hold_start ? now - hold_start : 0;
        for (size_t i = 0; i < n; ++i) {
          trace_.age_ns[row_index * n + i] = feedback[i].received_ns ?
            now - feedback[i].received_ns : -1;
          trace_.fresh[row_index * n + i] =
            feedback[i].sequence != storage.tx_seen_feedback[i];
        }
      }
      for (size_t i = 0; i < n; ++i) {
        if (schedule.last_send_ns && storage.tx_seen_feedback[i] == feedback[i].sequence) {
          ++tx_stats_.cycles_without_new_reply[i];
        }
        storage.tx_seen_feedback[i] = feedback[i].sequence;
      }
      if (hold_start) {tx_stats_.max_hold_ns = std::max(tx_stats_.max_hold_ns, now - hold_start);}
      hold_start = 0;
      bool encoded = true;
      for (size_t i = 0; i < n; ++i) {
        commands[i] = active && !zero_stop ? storage.tx_cache[i].command :
          StreamCommand{feedback[i].feedback.position, 0, 0, 0, 0};
        const auto & c = commands[i];
        const auto block = drivers_[i]->encodeControlBlock(c.position, c.velocity, c.kp, c.kd, c.effort);
        if (!block) {encoded = false; fault(StreamFault::kEncodeFailed, static_cast<int>(i)); break;}
        blocks[i] = {can_ids_[i], *block};
      }
      if (!encoded) {break;}
      if (row && n == 1) {
        row->command = {commands[0].position, commands[0].velocity, commands[0].kp,
        commands[0].kd, commands[0].effort}; row->command_valid = true; row->command_stale = zero_stop;
        row->command_block = blocks[0].second; row->encoded_valid = true;
      }
      size_t sent = 0;
      int64_t send_at = 0;
      bool failed = false;
      const size_t frames = config_.broadcast ? 1 : n;
      for (size_t i = 0; i < frames; ++i) {
        if (!tx_running_ || stop_all_->load()) {break;}
        // Recheck at the send boundary too: a publication during encoding cancels the old
        // batch before its first frame. Once started, finish this immutable batch; safety
        // stops and freshness limits are still checked before EVERY frame.
        if (!zero_stop && active && sent == 0 &&
          storage.commands.version() != observed_version) {break;}
        if (!zero_stop && rx_fault_.load() != StreamFault::kNone) {break;}
        if (!zero_stop && active && ns_now() >= schedule.published_ns + config_.max_command_age.count()) {
          break;  // Re-enter watchdog immediately; never emit a stale normal frame.
        }
        CanFrame frame;
        if (config_.broadcast) {
          frame = *buildBroadcastFrame(blocks, block_count_, padding_block_);
        } else {
          frame.id = static_cast<uint32_t>(can_ids_[i]); frame.setData(blocks[i].second);
        }
        const auto before = ns_now();
        if (!zero_stop && active && before >= schedule.published_ns + config_.max_command_age.count()) {
          break;
        }
        if (!send_at) {send_at = before;}
        if (row) {
          if (!row->send_attempted) {row->send_before_steady_ns = before;}
          row->send_attempted = true;
        }
        const bool success = tx_bus_->send(frame);
        if (row) {row->send_after_steady_ns = ns_now(); row->send_success = success;}
        if (!success) {
          if (config_.broadcast) {
            for (auto * driver : drivers_) {driver->latchCommunicationFault(false);}
          } else {drivers_[i]->latchCommunicationFault(false);}
          fault(StreamFault::kSendFailed, config_.broadcast ? -1 : static_cast<int>(i));
          failed = true; break;
        }
        ++sent;
      }
      tx_stats_.frames_sent += sent;
      if (row) {
        row->send_success = sent == frames;
        row->frames = static_cast<uint8_t>(sent);
        row->outcome = sent == frames ? StreamCycleOutcome::kSent : StreamCycleOutcome::kPartial;
      }
      if (sent) {
        if (!first_send) {first_send = send_at; first_send_ns_ = send_at;}
        schedule.sent(send_at, resend);
        last_send_kind = zero_stop ? 4 : !active ? 1 : resend ? 3 : 2;
        bootstrap_due = send_at + config_.period.count();
        if (resend) {++tx_stats_.resends;}
        if (sent == frames) {++tx_stats_.cycles;} else {++tx_stats_.partial_cycles;}
      }
      if (zero_stop) {
        tx_stats_.command_stale_zero_gain_frames = sent;
        tx_stats_.command_stale_expected_frames = frames;
        if (row) {row->outcome = StreamCycleOutcome::kStopped;}
        if (!failed && tx_running_ && !stop_all_->load()) {fault(stop_reason, -1);}
        else if (!failed) {tx_stats_.stopped_externally = true;}
        break;
      }
      if (failed) {break;}
      continue;
    }
    int64_t deadline = feedback_due;
    if (active) {
      deadline = std::min(deadline, schedule.published_ns + config_.max_command_age.count());
      if (snapshot_failed) {deadline = std::min(deadline, retry_due);}
      else if (schedule.pending() || resend) {deadline = std::min(deadline, schedule.earliestSend());}
      else {deadline = std::min(deadline, schedule.resend_ns);}
    } else {
      deadline = std::min(deadline, bootstrap_due);
      if (snapshot_failed) {deadline = std::min(deadline, retry_due);}
      if (armed) {deadline = std::min(deadline, armed + config_.first_write_timeout.count());}
    }
    deadline = std::max(deadline, ns_now() + 1);
    itimerspec timer{};
    timer.it_value.tv_sec = deadline / 1000000000;
    timer.it_value.tv_nsec = deadline % 1000000000;
    if (::timerfd_settime(storage.timer_fd, TFD_TIMER_ABSTIME, &timer, nullptr) != 0) {
      fault(StreamFault::kNotificationFailed, -1); break;
    }
    pollfd fds[2]{{storage.event_fd, POLLIN, 0}, {storage.timer_fd, POLLIN, 0}};
    const int result = ::poll(fds, 2, -1);
    if (result < 0) {
      if (errno == EINTR) {continue;}
      fault(StreamFault::kNotificationFailed, -1); break;
    }
    uint64_t count;
    for (auto & fd : fds) {
      if (fd.revents & (POLLERR | POLLHUP | POLLNVAL)) {notification_failed_ = true;}
      if (fd.revents & POLLIN) {
        const auto consumed = ::read(fd.fd, &count, sizeof(count));
        if (consumed < 0 && errno != EAGAIN && errno != EINTR) {notification_failed_ = true;}
      }
    }
  }
  tx_done_ = true;
}

}  // namespace bxi_hardware
