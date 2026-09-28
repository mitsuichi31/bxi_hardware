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
};

const char * streamFaultName(StreamFault fault);

struct StreamConfig
{
  std::chrono::nanoseconds period{std::chrono::milliseconds(1)};
  // Safe-stop when a reply is older than this at the start of a cycle.
  std::chrono::nanoseconds max_feedback_age{std::chrono::milliseconds(3)};
  // One FD frame on ID 0x7FF with 8-byte blocks at (id - 1) * 8, instead of one frame per motor.
  bool broadcast = false;
  // Single-motor HIL only: pad the broadcast to this many blocks with zero-gain, zero-torque
  // commands (0 = off). The motors behind the padded blocks must be absent.
  size_t padded_broadcast_blocks = 0;
  // Motor IDs that must not answer; one valid reply from any of them stops the stream.
  std::vector<int> forbidden_reply_ids;
  // The receive thread checks its stop flag at least this often.
  int rx_poll_ms = 2;
  std::chrono::milliseconds rx_ready_timeout{100};
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
  // Joins the send thread, then the receive thread. Safe to call more than once.
  void stop();

  const std::string & error() const {return error_;}
  // Complete only after stop().
  StreamStats stats() const;

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

  std::atomic<bool> rx_running_{false};
  std::atomic<bool> rx_ready_{false};
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
};

}  // namespace bxi_hardware

#endif  // BXI_HARDWARE__BUS_STREAMER_HPP_
