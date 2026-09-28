#include "bxi_hardware/bus_streamer.hpp"

#include <algorithm>
#include <set>
#include <utility>

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

}  // namespace

const char * streamFaultName(StreamFault fault)
{
  switch (fault) {
    case StreamFault::kNone: return "none";
    case StreamFault::kFeedbackStale: return "feedback stale";
    case StreamFault::kSendFailed: return "send failed";
    case StreamFault::kEncodeFailed: return "command not encodable";
    case StreamFault::kForbiddenReply: return "reply from a motor that must be absent";
    case StreamFault::kOwnerRequested: return "owner requested stop";
  }
  return "unknown";
}

BxiBusStreamer::BxiBusStreamer(
  CanBus * tx_bus, CanBus * rx_bus, std::vector<BxiActuatorDriver *> drivers,
  StreamConfig config, CycleHook hook, std::atomic<bool> * stop_all)
: tx_bus_(tx_bus), rx_bus_(rx_bus), drivers_(std::move(drivers)), config_(std::move(config)),
  hook_(std::move(hook)), stop_all_(stop_all)
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
  // Two missing or late replies in a row must not stop a healthy bus.
  if (config_.max_feedback_age < 2 * config_.period) {
    error_ = "max feedback age must be at least two periods";
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

bool BxiBusStreamer::start()
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
  slots_.assign(n, ReplySlot{});
  forbidden_replies_ = 0;
  forbidden_reply_id_ = 0;
  rx_frames_ = 0;
  rx_ignored_ = 0;
  tx_stats_ = StreamStats{};
  tx_stats_.replies.assign(n, 0);
  tx_stats_.cycles_without_new_reply.assign(n, 0);
  tx_stats_.max_age_ms.assign(n, 0.0);

  rx_ready_ = false;
  rx_running_ = true;
  rx_thread_ = std::thread(&BxiBusStreamer::receiveLoop, this);
  const auto deadline = clock_type::now() + config_.rx_ready_timeout;
  while (!rx_ready_ && clock_type::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  if (!rx_ready_) {
    error_ = "receive thread did not start in time";
    RCLCPP_ERROR(logger(), "[%s] %s", tx_bus_->name().c_str(), error_.c_str());
    stop();
    return false;
  }
  tx_done_ = false;
  first_send_ns_ = 0;
  tx_running_ = true;
  tx_thread_ = std::thread(&BxiBusStreamer::sendLoop, this);
  return true;
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
  tx_running_ = false;
  if (tx_thread_.joinable()) {
    tx_thread_.join();
  }
  rx_running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
}

StreamStats BxiBusStreamer::stats() const
{
  StreamStats stats = tx_stats_;
  stats.rx_frames = rx_frames_;
  stats.rx_ignored = rx_ignored_;
  return stats;
}

void BxiBusStreamer::receiveLoop()
{
  rx_ready_ = true;
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
        std::lock_guard<std::mutex> lock(slot_mutex_);
        auto & slot = slots_[i];
        slot.decoded = *decoded;
        slot.received_at = received_at;
        slot.rx_kernel_realtime_ns = frame.rx_kernel_realtime_ns;
        slot.rx_userspace_realtime_ns = frame.rx_userspace_realtime_ns;
        ++slot.sequence;
        used = true;
      }
      break;
    }
    if (!used) {
      for (const int id : config_.forbidden_reply_ids) {
        if (decodeMotorReply(frame, id, protocol::MitLimits{}).has_value()) {
          std::lock_guard<std::mutex> lock(slot_mutex_);
          ++forbidden_replies_;
          forbidden_reply_id_ = id;
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
      } else if (sent_previous_cycle) {
        ++tx_stats_.cycles_without_new_reply[i];
      }
      if (!sent_any) {
        continue;
      }
      const auto age = now - (has_reply[i] ? last_reply[i] : first_send);
      feedback[i].age_ms = toMilliseconds(age);
      tx_stats_.max_age_ms[i] = std::max(tx_stats_.max_age_ms[i], feedback[i].age_ms);
      if (!faulted && age > config_.max_feedback_age) {
        drivers_[i]->latchCommunicationFault();
        fault(StreamFault::kFeedbackStale, static_cast<int>(i));
        faulted = true;
      }
    }
    if (!faulted && forbidden_seen > 0) {
      tx_stats_.forbidden_replies = forbidden_seen;
      tx_stats_.forbidden_reply_id = forbidden_id;
      fault(StreamFault::kForbiddenReply, -1);
      faulted = true;
    }
    if (faulted) {
      break;
    }

    // Send nothing for a slot that is already a full period late, and never two frames within
    // half a period, so a stalled thread does not catch up in a burst.
    if (now >= target + period || (sent_any && now - last_send < period / 2)) {
      ++tx_stats_.skipped_cycles;
      sent_previous_cycle = false;
      continue;
    }

    if (!hook_(feedback, commands)) {
      fault(StreamFault::kOwnerRequested, -1);
      break;
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
    }
    if (faulted) {
      break;
    }

    // The hook may block, for example on the owner's data mutex. Check again before every frame
    // so that no command leaves after its slot has expired or after a stop was requested.
    const auto may_send = [&]() {
        return tx_running_ && !stop_all_->load() && clock_type::now() < target + period;
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
        if (!frame.has_value() || !tx_bus_->send(*frame)) {
          for (auto * driver : drivers_) {
            driver->latchCommunicationFault();
          }
          fault(StreamFault::kSendFailed, -1);
          break;
        }
        ++frames_this_cycle;
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
    if (frames_this_cycle > 0) {
      if (!sent_any) {
        first_send = send_time;
        sent_any = true;
        first_send_ns_ = send_time.time_since_epoch().count();
      }
      last_send = send_time;
    }
    if (faulted) {
      break;
    }
    if (window_closed) {
      if (stop_all_->load()) {
        tx_stats_.stopped_externally = true;
        break;
      }
      if (frames_this_cycle == 0) {
        ++tx_stats_.skipped_cycles;
      } else {
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

}  // namespace bxi_hardware
