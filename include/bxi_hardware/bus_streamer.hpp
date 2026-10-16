// 送受信分離: 1本のCANバスを送信スレッドと受信スレッドで周期運転する。
// 計画: docs/plans/20260928_bxi_worker_stream_broadcast_plan.md
#ifndef BXI_HARDWARE__BUS_STREAMER_HPP_
#define BXI_HARDWARE__BUS_STREAMER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "bxi_hardware/actuator/bxi_actuator.hpp"
#include "bxi_hardware/atomic_snapshot.hpp"
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
  uint64_t sequence = 0;
  int64_t published_ns = 0;
};

struct PublishedCommand
{
  StreamCommand command;
  uint64_t sequence = 0;
  int64_t published_ns = 0;
};
struct PublishedFeedback
{
  ActuatorFeedback feedback;
  uint64_t sequence = 0;
  int64_t received_ns = 0;
  int64_t published_ns = 0;
};
// Prepared before enable; immutable lifetime until read/write have stopped and threads joined.
struct WriteStreamStorage
{
  explicit WriteStreamStorage(size_t axes);
  ~WriteStreamStorage();
  WriteStreamStorage(const WriteStreamStorage &) = delete;
  WriteStreamStorage & operator=(const WriteStreamStorage &) = delete;
  AtomicSnapshot<PublishedCommand> commands;
  std::vector<std::unique_ptr<AtomicSnapshot<PublishedFeedback>>> feedback;
  std::unique_ptr<std::atomic<int64_t>[]> received_ns;
  std::unique_ptr<std::atomic<bool>[]> control_unavailable;
  std::vector<uint64_t> tx_seen_feedback;
  std::vector<PublishedCommand> tx_cache, tx_scratch;
  std::vector<PublishedFeedback> rx_cache, tx_feedback;
  std::vector<FeedbackTransform> transforms;
  std::vector<StreamCommand> tx_commands;
  std::vector<std::pair<int, std::array<uint8_t, 8>>> tx_blocks;
  // TX-only last valid material. Snapshot read failure never authorizes fallback.
  std::array<PublishedFeedback, 3> last_valid{};
  std::array<int64_t, 3> material_acquired_ns{};
  std::array<bool, 3> has_last_valid{}, material_read_failed{};
  int64_t first_stream_send_ns = 0;
  int event_fd = -1, timer_fd = -1;
};

struct StreamFeedback
{
  // Latest feedback in joint units; valid once any streamed reply has arrived.
  ActuatorFeedback feedback;
  // A new reply was applied at the start of this cycle.
  bool updated = false;
  // Since the latest reply, or since the first streamed send before any reply.
  double age_ms = 0.0;
  uint64_t sequence = 0;
  int64_t received_ns = 0;
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
  kCommandSnapshotUnavailable,
  kStartupTimeout,
  kTemperature,
  kNotificationFailed,
};

const char * streamFaultName(StreamFault fault);

enum class CommandPublishResult {kNotPublished, kSnapshotPublished, kSnapshotPublishedNotifyFailed};

// One activation-owned request, independent of the normal command snapshot.
// Metadata is atomic because the control thread, error callback and TX may interleave.
enum class SafetySource : uint32_t {kHwFault = 1, kModeStop = 2, kWatchdog = 4, kFeedbackStale = 8};
enum class SafetyResult : uint32_t {kUnattempted, kProcessing, kCannotGenerate, kSendFailed, kSucceeded, kIncomplete};
enum class SafetyFailure : uint32_t {kNone, kFeedbackRead, kInvalidFeedback, kPosition, kDriver, kEncode, kFrame, kNoErrorObservation, kDrainTimeout};
// Immutable captures are TX-owned and saved only after join. Unknown is explicit.
struct FeedbackCapture
{
  int64_t evaluated_ns = 0;
  uint32_t stale_mask = 0, missing_mask = 0, unknown_mask = 0;
  std::array<int64_t, 3> received_ns{}, age_ns{}, wait_ns{};
  std::array<double, 3> position{};
  std::array<uint64_t, 3> sequence{};
  std::array<int64_t, 3> material_received_ns{}, acquired_ns{}, material_age_ns{};
  std::array<uint8_t, 3> material{}, read_failed{}, material_unknown{};
  std::array<uint8_t, 3> base{};  // 0=not started, 1=RX, 2=first-send wait, 3=unknown
};
struct FeedbackStaleRecord
{
  size_t capacity = 1;
  bool detected = false, attempted = false;
  FeedbackCapture detection{}, attempt{};
  int primary_axis = -1;
  std::array<int64_t, 3> acquired_ns{}, material_age_ns{{-1, -1, -1}};
  std::array<uint8_t, 3> material_unknown{{1, 1, 1}}, read_failed{};
};
struct GroupSafety
{
  static_assert(std::atomic<int64_t>::is_always_lock_free && std::atomic<uint32_t>::is_always_lock_free &&
    std::atomic<uint64_t>::is_always_lock_free && std::atomic<bool>::is_always_lock_free, "G safety requires lock-free atomics");
  uint64_t epoch = 0;
  size_t axes = 0, record_capacity = 1;
  std::atomic<uint32_t> sources{0};
  std::atomic<int64_t> detected_ns{0}, registered_ns{0}, error_observed_ns{0};
  std::atomic<int64_t> attempted_ns{0}, completed_ns{0}, mode_stop_observed_ns{0};
  std::atomic<SafetyResult> result{SafetyResult::kUnattempted};
  std::atomic<bool> abandoned{false}, drain_timed_out{false};
  std::atomic<uint64_t> requests{0};
  // TX owns these fields; inspect/save only after TX join.
  std::array<double, 3> position{};
  std::array<uint8_t, 3> material{};  // 1=prepared, 2=snapshot, 4=valid, 8=finite
  std::array<uint64_t, 3> feedback_sequence{};
  std::array<int64_t, 3> feedback_received_ns{};
  SafetyFailure failure = SafetyFailure::kNone;
  int failure_axis = -1;
  bool send_attempted = false, send_success = false, notification_failed = false;
  StreamFault terminal_fault = StreamFault::kOwnerRequested;
  // TX writes the cause record; RT writers only observe this admission latch.
  std::atomic<bool> feedback_stopped{false};
  FeedbackStaleRecord feedback_stale;
  bool saveFeedbackStale(const std::string & path) const;
  uint64_t watchdog_sequence = 0;
  int64_t watchdog_published_ns = 0;
  void reset(uint64_t new_epoch, size_t count) noexcept {
    epoch = new_epoch; axes = count;
    sources = 0; detected_ns = 0; registered_ns = 0; error_observed_ns = 0;
    attempted_ns = 0; completed_ns = 0; mode_stop_observed_ns = 0; result = SafetyResult::kUnattempted;
    abandoned = false; drain_timed_out = false; requests = 0;
    position = {}; material = {}; feedback_sequence = {}; feedback_received_ns = {};
    failure = SafetyFailure::kNone; failure_axis = -1;
    send_attempted = send_success = notification_failed = false;
    feedback_stopped = false;
    const auto capacity = feedback_stale.capacity;
    feedback_stale = {}; feedback_stale.capacity = capacity;
    terminal_fault = StreamFault::kOwnerRequested; watchdog_sequence = 0; watchdog_published_ns = 0;
  }
  bool request(uint64_t request_epoch, SafetySource source, int64_t detected, int64_t registered) noexcept {
    if (request_epoch != epoch) {return false;}
    ++requests;
    int64_t zero = 0; detected_ns.compare_exchange_strong(zero, detected);
    zero = 0; registered_ns.compare_exchange_strong(zero, registered);
    sources.fetch_or(static_cast<uint32_t>(source));
    return true;
  }
  void observeError(int64_t now) noexcept {
    int64_t zero = 0; error_observed_ns.compare_exchange_strong(zero, now);
  }
  bool requested() const noexcept {return sources.load() != 0;}
  bool ready() const noexcept {
    return requested() && (!(sources.load() & static_cast<uint32_t>(SafetySource::kHwFault)) ||
      error_observed_ns.load() != 0);
  }
  bool finished() const noexcept {
    const auto value = result.load();
    return value != SafetyResult::kUnattempted && value != SafetyResult::kProcessing;
  }
  bool save(const std::string & path) const;  // Non-RT, after join. False for missing/truncated/failed output.
};

struct StreamConfig
{
  GroupSafety * group_safety = nullptr;  // G only; owner outlives TX/RX join.
  std::chrono::milliseconds safety_drain_timeout{100};
  int64_t activation_origin_ns = 0;  // G only; zero retains legacy armWrites origin.
  bool control_trace = false;
  bool write_triggered = false;
  bool direct_feedback = false;  // periodic TX + direct RX is supported for comparisons.
  std::chrono::nanoseconds write_grace{0};
  std::chrono::nanoseconds first_write_timeout{std::chrono::milliseconds(20)};
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
  // All prepared write-stream event FDs; immutable until every TX/RX has joined.
  std::vector<int> stop_wake_fds;
  // Each thread must report that it started (scheduling applied) within this time.
  std::chrono::milliseconds rx_ready_timeout{100};
  // Cycles to record in the preallocated trace (0 = no trace). Replies get twice as many rows
  // per axis.
  size_t trace_cycles = 0;
  bool trace_extended = false;
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

// Trace values only: sequence/publication are recorded once in the enclosing row.
struct TraceCommand
{
  double position = 0.0, velocity = 0.0, kp = 0.0, kd = 0.0, effort = 0.0;
};
struct StreamCycleRecord
{
  uint64_t cycle = 0;
  int64_t wake_lateness_ns = 0, hook_ns = 0, send_lateness_ns = -1;
  int64_t command_age_ns = -1;
  TraceCommand command{};
  std::array<uint8_t, 8> command_block{};
  int64_t send_before_steady_ns = -1, send_after_steady_ns = -1;
  uint64_t command_sequence = 0;
  int64_t command_published_ns = 0, wake_steady_ns = 0, resend_due_ns = 0, hold_ns = 0;
  // Group small fields to avoid per-row alignment padding. CSV columns are unchanged.
  uint8_t frames = 0;
  StreamCycleOutcome outcome = StreamCycleOutcome::kSent;
  bool command_valid = false, encoded_valid = false, send_attempted = false;
  bool send_success = false, command_stale = false;
  uint8_t send_kind = 0;  // 0 periodic, 1 bootstrap, 2 new write, 3 resend, 4 stale zero gain.
  bool snapshot_failed = false;
};

struct StreamRxRecord
{
  uint16_t axis = 0;
  int64_t received_ns = 0;  // steady clock, relative to StreamTrace::origin
  int64_t rx_kernel_realtime_ns = 0;
  int64_t rx_userspace_realtime_ns = 0;
  int64_t rx_hardware_raw_ns = 0;
  double motor_position = 0.0;
  double motor_velocity = 0.0;
  double motor_torque = 0.0;
  uint64_t sequence = 0;
  int64_t published_ns = 0;
};

struct StreamAxisMetadata
{
  int can_id = 0;
  int motor_dir = 1;
  double offset_rad_effective = 0.0;
  double lower_rad_effective = 0.0;
  double upper_rad_effective = 0.0;
  protocol::MitLimits limits{};
};

// Filled while running and read after stop(). HWIF prepares storage before enable; start() moves it.
struct StreamTrace
{
  bool synthetic = false;  // Test-only maximum-capacity output; never physical CAN observations.
  std::chrono::steady_clock::time_point origin{};
  size_t axes = 0;
  int64_t send_loop_start_steady_ns = -1;
  int64_t writes_armed_steady_ns = 0;
  int64_t stream_stop_steady_ns = -1;
  int64_t allocation_ns = 0;
  size_t payload_bytes = 0;
  std::vector<StreamAxisMetadata> axis_metadata;
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
  uint64_t resends = 0;
  uint64_t command_snapshot_failures = 0;
  uint64_t command_coalesced = 0;
  uint64_t minimum_interval_holds = 0;
  uint64_t writes_after_resend = 0;
  uint64_t resend_command_holds = 0;
  uint64_t new_command_holds = 0;
  uint64_t bootstrap_command_holds = 0;
  int64_t max_hold_ns = 0;
  bool feedback_stale_at_stop = false;
  StreamFault received_fault_at_stop = StreamFault::kNone;
  int temperature_fault_axis = -1;
  double temperature_fault_c = 0.0;
  int64_t temperature_fault_received_ns = 0;
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
    StreamConfig config, CycleHook hook, std::atomic<bool> * stop_all,
    std::unique_ptr<WriteStreamStorage> storage = nullptr);
  virtual ~BxiBusStreamer();
  BxiBusStreamer(const BxiBusStreamer &) = delete;
  BxiBusStreamer & operator=(const BxiBusStreamer &) = delete;

  // Starts the receive thread, waits until it is receiving, then starts the send thread.
  // Returns false, with no thread left running, on an invalid setup or a receive-start timeout.
  // HWIF must supply storage prepared before any enable. Legacy standalone callers may omit it.
  bool start(std::unique_ptr<StreamTrace> prepared = nullptr);
  // True once every axis has a reply received after the first streamed send; false on timeout
  // or fault.
  bool waitForFirstFeedback(std::chrono::nanoseconds timeout);
  // Asks the send thread to stop sending, without waiting. No frame leaves after the request is
  // seen. stop() still has to join. Lets an owner stop several buses before joining any of them.
  void requestStop();
  // Single control-thread writer. Scratch values supplied by HWIF, already allocated.
  bool publishCommands(const PublishedCommand * commands);
  CommandPublishResult publishCommandBatch(const PublishedCommand * commands);
  bool readFeedback(size_t axis, PublishedFeedback & scratch, unsigned * attempts = nullptr) const;
  void setControlUnavailable(size_t axis, bool unavailable);
  bool controlUnavailable(size_t axis) const;
  // Called after on_activate's bootstrap feedback handshake, before controller write begins.
  void armWrites();
  void notify();
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
  size_t broadcastBlockCount() const {return block_count_;}
  const std::array<uint8_t, 8> & paddingBlock() const {return padding_block_;}

  // The 0x7FF frame for blocks indexed by motor ID - 1. nullopt if the length is not a valid
  // CAN FD length or a block position is out of range.
  static std::optional<CanFrame> buildBroadcastFrame(
    const std::vector<std::pair<int, std::array<uint8_t, 8>>> & blocks, size_t block_count,
    const std::array<uint8_t, 8> & padding);

protected:
  virtual std::optional<std::array<uint8_t, 8>> encodeSafety(size_t axis, double position) const {
    return drivers_[axis]->encodeControlBlock(position, 0, 0, 0, 0);
  }
private:
  friend struct GroupSafetyTestAccess;
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
  void sendWriteLoop();
  void performGroupSafety();
  FeedbackCapture captureFeedback(int64_t evaluated_ns = 0);
  void collectFeedbackStale(int64_t evaluated_ns = 0);
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
  std::unique_ptr<WriteStreamStorage> write_storage_;
  std::atomic<int64_t> writes_armed_ns_{0};
  std::atomic<StreamFault> rx_fault_{StreamFault::kNone};
  std::atomic<int> rx_fault_axis_{-1};
  // First thermal fault only; RX writes, stats() reads after RX join.
  double rx_temperature_fault_c_ = 0.0;
  int64_t rx_temperature_fault_received_ns_ = 0;
  std::atomic<bool> notification_failed_{false};
};

}  // namespace bxi_hardware

#endif  // BXI_HARDWARE__BUS_STREAMER_HPP_
