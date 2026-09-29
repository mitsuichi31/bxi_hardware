// 送受信分離: 1本のCANバスを送信スレッドと受信スレッドで周期運転する。
// 計画: docs/plans/20260928_bxi_worker_stream_broadcast_plan.md
#ifndef BXI_HARDWARE__BUS_STREAMER_HPP_
#define BXI_HARDWARE__BUS_STREAMER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "bxi_hardware/actuator/bxi_actuator.hpp"
#include "bxi_hardware/can/can_bus.hpp"

namespace bxi_hardware
{

struct StreamCommand
{
  double position = 0.0;
  double velocity = 0.0;
  double kp = 0.0;
  double kd = 0.0;
  double effort = 0.0;
};

struct StreamFeedback
{
  // Latest feedback in joint units; valid once any streamed reply has arrived.
  ActuatorFeedback feedback;
  // A new reply was applied at the start of this cycle.
  bool updated = false;
  // Since the latest reply, or since the first streamed send before any reply.
  double age_ms = 0.0;
};

enum class StreamFault
{
  kNone,
  kFeedbackStale,
  kSendFailed,
  kEncodeFailed,
  kForbiddenReply,
  kOwnerRequested,
  kCommandStale,
};

const char * streamFaultName(StreamFault fault);

struct StreamConfig
{
  std::chrono::nanoseconds period{std::chrono::milliseconds(1)};
  // Safe-stop when a reply is older than this at the start of a cycle.
  std::chrono::nanoseconds max_feedback_age{std::chrono::milliseconds(3)};
  // Safe-stop when the owner's latest command write, stored by the owner in *command_written_ns
  // as steady_clock nanoseconds since its epoch, is older than this at the start of a cycle
  // (0 = off). Replies can keep arriving while the owner's control loop is stalled, so the reply
  // age alone does not detect it. Checked in every cycle, including late ones; the stale cycle
  // sends zero-gain, zero-torque commands to every motor regardless of the slot deadline, then
  // stops with kCommandStale. If a stop request comes first (the owner's stop, another bus's
  // fault), the remaining frames are not sent and the stream ends as an external stop instead;
  // if a send fails, it ends as kSendFailed. StreamStats::command_stale_zero_gain_frames counts
  // what was sent either way. Not checked while the stored value is 0 (no write yet). The atomic
  // must outlive the streamer.
  std::chrono::nanoseconds max_command_age{0};
  const std::atomic<int64_t> * command_written_ns = nullptr;
  // One FD frame on ID 0x7FF with 8-byte blocks at (id - 1) * 8, instead of one frame per motor.
  bool broadcast = false;
  // Single-motor HIL only: pad the broadcast to this many blocks with zero-gain, zero-torque
  // commands (0 = off). The motors behind the padded blocks must be absent.
  size_t padded_broadcast_blocks = 0;
  // Motor IDs that must not answer; one valid reply from any of them stops the stream.
  std::vector<int> forbidden_reply_ids;
  // The receive thread checks its stop flag at least this often.
  int rx_poll_ms = 2;
  // Each thread must report that it started (scheduling applied) within this time.
  std::chrono::milliseconds rx_ready_timeout{100};
  // Cycles to record in the preallocated trace (0 = no trace). Replies get twice as many rows
  // per axis.
  size_t trace_cycles = 0;
  // SCHED_FIFO priority (1-98) of each thread; 0 selects SCHED_OTHER even if the thread calling
  // start() runs under SCHED_FIFO. Each thread applies its own setting as it starts, and start()
  // fails if that is refused.
  int send_priority = 0;
  int receive_priority = 0;
  // CPU affinity of both threads; empty keeps the inherited affinity.
  std::vector<int> cpus;
};

enum class StreamCycleOutcome : uint8_t
{
  kSent,              // every command of the cycle was sent
  kSkippedLate,       // woke a full period late; nothing sent
  kSkippedBurst,      // within half a period of the previous send; nothing sent
  kExpiredAfterHook,  // the slot expired while the hook ran; nothing sent
  kPartial,           // the slot expired between per-motor frames
  kStopped,           // a fault or stop request ended the stream in this cycle
};

const char * streamCycleOutcomeName(StreamCycleOutcome outcome);

struct StreamCycleRecord
{
  uint64_t cycle = 0;
  int64_t wake_lateness_ns = 0;  // wake-up after the scheduled slot start
  int64_t hook_ns = 0;           // time in the owner hook, including its lock waits
  int64_t send_lateness_ns = -1;  // first frame after the slot start; -1 if nothing was sent
  // Age of the owner's latest command at the start of the cycle; -1 when not checked.
  int64_t command_age_ns = -1;
  uint8_t frames = 0;
  StreamCycleOutcome outcome = StreamCycleOutcome::kSent;
};

struct StreamRxRecord
{
  uint16_t axis = 0;
  int64_t received_ns = 0;  // steady clock, relative to StreamTrace::origin
  int64_t rx_kernel_realtime_ns = 0;
  int64_t rx_userspace_realtime_ns = 0;
  int64_t rx_hardware_raw_ns = 0;
};

// Filled only while running and read only after stop(). All storage is allocated in start().
struct StreamTrace
{
  std::chrono::steady_clock::time_point origin{};
  size_t axes = 0;
  std::vector<StreamCycleRecord> cycles;
  size_t cycle_count = 0;  // may exceed cycles.size(); the excess is dropped
  // cycles.size() x axes, row-major. Age at the start of the cycle; -1 before the first send.
  std::vector<int64_t> age_ns;
  std::vector<uint8_t> fresh;  // a new reply was applied at the start of the cycle
  std::vector<StreamRxRecord> rx;
  size_t rx_count = 0;  // may exceed rx.size(); the excess is dropped
  // Scheduling of each thread as it started, e.g. "SCHED_OTHER priority=0 cpus=0-15".
  std::string tx_thread;
  std::string rx_thread;
};

struct StreamStats
{
  uint64_t cycles = 0;          // cycles in which every command was sent
  uint64_t skipped_cycles = 0;  // late cycles that sent nothing
  uint64_t partial_cycles = 0;  // per-motor frames stopped mid-cycle because the slot expired
  uint64_t frames_sent = 0;
  std::vector<uint64_t> replies;                   // per axis
  std::vector<uint64_t> cycles_without_new_reply;  // per axis, statistics only
  std::vector<double> max_age_ms;                  // per axis
  double max_command_age_ms = 0.0;  // largest checked command age; 0 while unchecked
  // On a command-age stop: zero-gain frames sent in the stopping cycle, out of the expected count.
  size_t command_stale_zero_gain_frames = 0;
  size_t command_stale_expected_frames = 0;
  uint64_t rx_frames = 0;
  uint64_t rx_ignored = 0;
  uint64_t forbidden_replies = 0;
  int forbidden_reply_id = 0;
  bool stopped_externally = false;
  StreamFault fault = StreamFault::kNone;
  int fault_axis = -1;  // -1 when the fault is not tied to one axis
};

// Runs one bus. While running, the send thread is the only user of the drivers and of tx_bus,
// and the receive thread is the only user of rx_bus. The owner configures both sockets (receive
// filter on rx_bus, receive disabled on tx_bus) before start() and restores them after stop().
class BxiBusStreamer
{
public:
  // Called on the send thread once per sending cycle with the feedback applied at its start.
  // Must fill one command per axis. Returning false requests a safe stop.
  using CycleHook =
    std::function<bool(const std::vector<StreamFeedback> &, std::vector<StreamCommand> &)>;

  BxiBusStreamer(
    CanBus * tx_bus, CanBus * rx_bus, std::vector<BxiActuatorDriver *> drivers,
    StreamConfig config, CycleHook hook, std::atomic<bool> * stop_all);
  ~BxiBusStreamer();
  BxiBusStreamer(const BxiBusStreamer &) = delete;
  BxiBusStreamer & operator=(const BxiBusStreamer &) = delete;

  // Starts the receive thread, waits until it is receiving, then starts the send thread.
  // Returns false, with no thread left running, on an invalid setup or a receive-start timeout.
  bool start();
  // True once every axis has a reply received after the first streamed send; false on timeout
  // or fault.
  bool waitForFirstFeedback(std::chrono::nanoseconds timeout);
  // Asks the send thread to stop sending, without waiting. No frame leaves after the request is
  // seen. stop() still has to join. Lets an owner stop several buses before joining any of them.
  void requestStop() {tx_running_ = false;}
  // Joins the send thread, then the receive thread. Safe to call more than once.
  void stop();

  // True once the send thread has ended: after a fault, an external stop, or stop().
  bool sendFinished() const {return tx_done_;}
  const std::string & error() const {return error_;}
  // Complete only after stop().
  StreamStats stats() const;
  const StreamTrace & trace() const {return trace_;}
  const StreamConfig & config() const {return config_;}
  const std::vector<int> & canIds() const {return can_ids_;}

  // The 0x7FF frame for blocks indexed by motor ID - 1. nullopt if the length is not a valid
  // CAN FD length or a block position is out of range.
  static std::optional<CanFrame> buildBroadcastFrame(
    const std::vector<std::pair<int, std::array<uint8_t, 8>>> & blocks, size_t block_count,
    const std::array<uint8_t, 8> & padding);

private:
  struct ReplySlot
  {
    protocol::MitFeedback decoded;
    std::chrono::steady_clock::time_point received_at{};
    int64_t rx_kernel_realtime_ns = 0;
    int64_t rx_userspace_realtime_ns = 0;
    uint64_t sequence = 0;
  };

  bool validate();
  void receiveLoop();
  void sendLoop();
  void fault(StreamFault kind, int axis);

  CanBus * tx_bus_;
  CanBus * rx_bus_;
  std::vector<BxiActuatorDriver *> drivers_;
  StreamConfig config_;
  CycleHook hook_;
  std::atomic<bool> * stop_all_;
  std::string error_;

  // Immutable while running; the receive thread uses only these copies.
  std::vector<int> can_ids_;
  std::vector<uint32_t> reply_ids_;
  std::vector<protocol::MitLimits> limits_;
  size_t block_count_ = 0;
  std::array<uint8_t, 8> padding_block_{};

  std::mutex slot_mutex_;
  std::vector<ReplySlot> slots_;
  uint64_t forbidden_replies_ = 0;
  int forbidden_reply_id_ = 0;

  // Start-up handshake of each thread: scheduling applied, or refused. The reason is left in
  // *_setup_error_ and read only after the thread has been joined.
  enum ThreadState : int {kStarting, kReady, kFailed};
  bool waitForThread(const std::atomic<int> & state);

  std::atomic<bool> rx_running_{false};
  std::atomic<int> rx_state_{kStarting};
  std::atomic<int> tx_state_{kStarting};
  std::string rx_setup_error_;
  std::string tx_setup_error_;
  std::atomic<bool> tx_running_{false};
  std::atomic<bool> tx_done_{false};
  // steady_clock ticks of the first streamed send; 0 until then.
  std::atomic<int64_t> first_send_ns_{0};

  std::thread rx_thread_;
  std::thread tx_thread_;

  // Written by the receive thread, read after join.
  uint64_t rx_frames_ = 0;
  uint64_t rx_ignored_ = 0;
  // Written by the send thread, read after join.
  StreamStats tx_stats_;
  // Cycle rows and tx_thread by the send thread, rx rows and rx_thread by the receive thread.
  StreamTrace trace_;
};

}  // namespace bxi_hardware

#endif  // BXI_HARDWARE__BUS_STREAMER_HPP_
