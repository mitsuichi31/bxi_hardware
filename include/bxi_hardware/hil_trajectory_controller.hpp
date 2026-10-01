// HilTrajectoryController: computes the single-joint HIL ramp / sine targets inside
// controller_manager's update() every cycle, so that every cycle writes the trajectory's value at
// that time (no topic, no other process's period in between).
//
// Real-time side (update()): no allocation, no blocking lock (try_lock only), no exception, no
// logging. Requests, keepalives and status cross over through try-locked mailboxes and atomics.
// Plan: docs/plans/20260929_bxi_rt_trajectory_controller_plan.md (workspace).
#ifndef BXI_HARDWARE__HIL_TRAJECTORY_CONTROLLER_HPP_
#define BXI_HARDWARE__HIL_TRAJECTORY_CONTROLLER_HPP_

#include <atomic>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "bxi_hardware/hil_trajectory.hpp"
#include "bxi_hardware/msg/hil_trajectory_status.hpp"
#include "bxi_hardware/srv/start_hil_trajectory.hpp"

namespace bxi_hardware
{

class HilTrajectoryController : public controller_interface::ControllerInterface
{
public:
  enum class Kind : uint8_t {kHold = 0, kRamp = 1, kSine = 2};
  enum class State : uint8_t {kIdle = 0, kArmed, kRunning, kHolding, kReleased, kAborted};
  enum class EndReason : uint8_t
  {
    kNone,
    kRelease,         // release: true, after the trajectory and hold
    kFinish,          // ~/finish while holding
    kStop,            // ~/stop
    kNoKeepalive,     // no matching keepalive while armed (never moved)
    kLease,           // keepalive of the running request stopped for longer than its lease
    kTorque,          // Kp |target - position| + |feedforward| > max_torque
    kVelocity,        // |target velocity| > max_velocity
    kStateNotFinite,  // position, velocity or effort not finite
    kPeriod,          // period not finite, <= 0 or > max_period
    kStartRange,      // sine: center +- amplitude outside the joint range at the start
    kDuration,        // ramp: total time over the limit at the start
    kDeactivated,     // the controller was deactivated during a run
    kWriteFailed,     // a command interface refused a value while the outputs had gains
  };
  static const char * endReasonName(EndReason reason);

  // A validated start request, as handed to update(). Plain data.
  struct StartRequest
  {
    uint32_t id = 0;
    Kind kind = Kind::kHold;
    double target = 0.0;
    double speed = 0.0;
    double amplitude = 0.0;
    double frequency = 0.0;
    double cycles = 0.0;
    double ramp_cycles = 0.0;
    double friction_kinetic = 0.0;
    double friction_static = -1.0;
    double friction_velocity = 0.05;
    double stribeck_velocity = 0.2;
    double kp = 0.0;
    double kd = 0.0;
    double hold = 0.0;
    bool release = false;
    int64_t lease_ns = 0;
    int64_t accepted_ns = 0;  // steadyNowNs() when accepted; keepalives must be later
  };

  struct StartResult
  {
    bool accepted = false;
    uint32_t id = 0;
    std::string message;
  };

  // Status, copied from update() at the end of each cycle (try-lock; a busy cycle skips it).
  struct Snapshot
  {
    State state = State::kIdle;
    uint32_t applied_id = 0;
    uint32_t applied_stop_count = 0;
    EndReason end_reason = EndReason::kNone;
    double elapsed = 0.0;
    double target_position = std::numeric_limits<double>::quiet_NaN();
    double position = std::numeric_limits<double>::quiet_NaN();
    bool recording = false;
    uint64_t recorded_cycles = 0;
    uint64_t dropped_cycles = 0;
    uint64_t write_failures = 0;
    bool release_write_pending = false;  // no complete released write has succeeded for this run
  };

  // One row of the per-cycle record.
  struct Row
  {
    int64_t steady_ns = 0;      // steadyNowNs() at the start of update()
    double ros_time_s = 0.0;    // update()'s time argument (not used for the trajectory)
    double period_s = 0.0;
    double elapsed_s = 0.0;     // accumulated period since the trajectory started
    double target_position = 0.0;
    double target_velocity = 0.0;
    double kp = 0.0;
    double kd = 0.0;
    double feedforward = 0.0;
    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;
    int64_t update_ns = 0;      // time spent in update() before writing this row
    uint8_t state = 0;
    uint8_t end_reason = 0;
    uint8_t write_ok = 1;       // every command interface took the value this cycle
  };

  // Writes a record completed by the last deactivation (see the .cpp).
  ~HilTrajectoryController() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Non-real-time entry points (the services and the keepalive subscription call these; tests
  // call them directly).
  StartResult requestStart(const srv::StartHilTrajectory::Request & request);
  bool requestFinish(std::string * message);
  void requestStop();
  void onKeepalive(uint32_t id);
  Snapshot snapshot() const;
  // Writes a completed record to CSV if there is one. Returns true when a file was written.
  bool writeCompletedRecord();
  std::string csvPath() const;

protected:
  // steady_clock nanoseconds; tests override it to control the lease and arming timeouts.
  virtual int64_t steadyNowNs() const;
  // Writes one command interface (0 position, 1 velocity, 2 kp, 3 kd, 4 effort). false when the
  // interface refused the value (set_value() cannot take its lock). Tests override it.
  virtual bool writeCommand(size_t index, double value);
  // Test seam: holds the snapshot lock, so that update() cannot refresh the snapshot.
  std::unique_lock<std::mutex> holdSnapshotLockForTest()
  {
    return std::unique_lock<std::mutex>(snapshot_mutex_);
  }

private:
  // Parameters
  std::string joint_;
  double lower_ = -0.5;
  double upper_ = 0.5;
  double max_kp_ = 20.0;
  double max_torque_ = 5.0;
  double max_velocity_ = 2.0;
  double max_amplitude_ = 0.5;
  bool fixed_sine_center_ = false;
  double sine_center_ = 0.0;
  double max_period_s_ = 0.005;
  double arm_timeout_s_ = 1.0;
  double record_tail_s_ = 0.5;
  double record_seconds_ = 120.0;
  double max_run_seconds_ = 60.0;
  std::string output_dir_;
  // Test only (vcan fault paths): each service callback sleeps this long before it handles the
  // request, so the request is neither applied nor answered meanwhile. The controller node's
  // callbacks share one mutually exclusive group, so keepalives and ~/status wait as well.
  // Non-zero values are refused unless BXI_ALLOW_TEST_FAULTS=1 (never set on the HIL runners).
  double test_start_delay_s_ = 0.0;
  double test_stop_delay_s_ = 0.0;
  double test_finish_delay_s_ = 0.0;

  std::string validate(const srv::StartHilTrajectory::Request & r) const;
  void endRun(EndReason reason, int64_t now_ns);
  // true when all five values were taken.
  bool writeOutputs(double position, double velocity, double kp, double kd, double feedforward);
  bool writeReleased(double position);
  // Stores state_ and run_.id where the non-RT side reads them (every cycle, never skipped).
  void publishRtState();

  // --- shared with the non-real-time side ---
  mutable std::mutex start_mutex_;
  std::optional<StartRequest> pending_start_;
  uint32_t next_id_ = 1;  // non-RT only
  // ~/stop clears a pending start under start_mutex_ (the lock update() takes the start with), so
  // a stop always cancels a start accepted before it; update() then ends any run on the new count.
  std::atomic<uint32_t> stop_count_{0};
  // Always updated by update() (atomics, not the try-locked snapshot), so the services decide on
  // the current state:
  //   busy_: set with a start taken from pending_start_ (under start_mutex_), cleared by the
  //   non-RT side only after that run's record was handed over (CSV written, or no output_dir).
  std::atomic<bool> busy_{false};
  std::atomic<uint32_t> rt_applied_stop_count_{0};
  std::atomic<uint64_t> rt_state_id_{0};  // state << 32 | request id
  std::atomic<uint32_t> finish_for_id_{0};
  mutable std::mutex keepalive_mutex_;
  uint32_t keepalive_id_ = 0;
  int64_t keepalive_ns_ = 0;
  mutable std::mutex snapshot_mutex_;
  Snapshot snapshot_;
  std::vector<Row> rows_;  // sized in on_configure()
  std::atomic<bool> record_complete_{false};
  std::atomic<uint64_t> record_count_{0};
  std::atomic<uint64_t> record_dropped_{0};
  std::atomic<uint32_t> record_id_{0};
  mutable std::mutex csv_mutex_;
  std::string csv_path_;
  bool csv_failed_ = false;
  int64_t last_csv_attempt_ns_ = 0;  // status timer only
  // Written by update() while recording; read by the non-RT side only after record_complete_
  // (release/acquire). A new record starts only after the previous CSV was written.
  StartRequest record_run_;
  EndReason record_end_reason_ = EndReason::kNone;

  // --- real-time state (update() only) ---
  State state_ = State::kIdle;
  EndReason end_reason_ = EndReason::kNone;
  StartRequest run_;
  uint32_t applied_stop_count_ = 0;
  int64_t armed_ns_ = 0;
  int64_t end_ns_ = 0;
  int64_t last_keepalive_ns_ = 0;  // for run_.id
  double elapsed_ = 0.0;
  bool first_running_cycle_ = false;
  double start_position_ = 0.0;
  double run_duration_ = 0.0;       // trajectory + hold
  std::optional<hil::RampTrajectory> ramp_;
  std::optional<hil::SineTrajectory> sine_;
  hil::Setpoint final_setpoint_;
  double last_target_ = std::numeric_limits<double>::quiet_NaN();
  bool recording_ = false;
  uint64_t rows_written_ = 0;
  uint64_t rows_dropped_ = 0;
  uint64_t write_failures_ = 0;
  bool release_write_pending_ = false;

  rclcpp::Service<srv::StartHilTrajectory>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr finish_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr keepalive_sub_;
  rclcpp::Publisher<msg::HilTrajectoryStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace bxi_hardware

#endif  // BXI_HARDWARE__HIL_TRAJECTORY_CONTROLLER_HPP_
