#ifndef BXI_HARDWARE__N_AXIS_TRAJECTORY_CONTROLLER_HPP_
#define BXI_HARDWARE__N_AXIS_TRAJECTORY_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "bxi_hardware/atomic_snapshot.hpp"
#include "bxi_hardware/hil_trajectory.hpp"

namespace bxi_hardware
{
// G only. Internal non-RT entry points intentionally have no ROS transport (G1-4).
class NAxisTrajectoryController : public controller_interface::ControllerInterface
{
public:
  static constexpr size_t kAxes = 3, kCommands = 16, kStates = 10;
  static constexpr uint64_t kMaxGeneration = 9007199254740991ULL;
  static constexpr int64_t kLeaseNs = 500000000, kArmNs = 1000000000;
  static constexpr double kPi = 3.14159265358979323846;
  enum class State : uint8_t {kBoot, kIdle, kArmed, kRunning, kHolding, kReleased, kStopped, kAborted};
  enum class Reason : uint8_t {kNone, kRelease, kFinish, kStop, kState, kPeriod, kLease,
    kArmTimeout, kStartRange, kTrajectory, kTorque, kWrite, kCapacity, kUpperLimit, kDeactivated};
  enum class Accepted : uint8_t {kInitial, kMatch, kMismatch, kUnavailable, kNonfinite};
  struct Request {
    uint64_t nonce = 0, id = 0;
    int64_t sent_ns = 0, received_ns = 0;  // receipt is diagnostic only, set by requestStart
    size_t axes = 0;
    double amplitude = kPi / 6., frequency = .2, cycles = 5., ramp_cycles = 1., hold = 1.;
    std::array<double, 3> kp{{20., 20., 20.}}, kd{{1., 1., 1.}};
    bool release = true;
  };
  struct Heartbeat {uint64_t nonce = 0, id = 0, sequence = 0; int64_t sent_ns = 0;};
  struct Attempt {
    uint32_t write_attempted = 0, write_ok = 0, read_attempted = 0, read_ok = 0;
    uint32_t unavailable = 0, mismatch = 0;
    std::array<double, 16> values{}, readback{};
    int64_t commit_set_ns = 0, commit_read_ns = 0;
  };
  struct Row {
    uint64_t activation = 0, cycle = 0, nonce = 0, request_id = 0;
    uint64_t generation = 0, previous_generation = 0, holds = 0, max_holds = 0, failures = 0;
    int64_t start_ns = 0, end_ns = 0, error_ns = 0, detected_ns = 0, accepted_ns = 0;
    int64_t heartbeat_sent_ns = 0, deadline_ns = 0, request_sent_ns = 0, request_received_ns = 0;
    double accepted = 0., elapsed = 0., acceleration = 0.;
    std::array<double, 9> states{};
    uint32_t state_ok = 0, material_missing = 0;
    Attempt normal{}, released{};
    State before = State::kBoot, after = State::kBoot;
    Reason reason = Reason::kNone;
    Accepted accepted_result = Accepted::kInitial;
    uint8_t axes = 0;
    bool advanced = false, held = false, released_attempted = false;
  };
  struct Timing {int64_t configure_in = 0, configure_out = 0, activate_in = 0, activate_out = 0, first_update = 0;};
  struct Budget {size_t calls, attempts, yields;};
  static constexpr Budget apiBudget(size_t n, unsigned tries = 1) {
    return {4 * (5 * n + 1) + 3 * n + 1,
      (4 * (5 * n + 1) + 3 * n + 1) * tries,
      2 * (5 * n + 1) * (tries - 1) + (2 * (5 * n + 1) + 3 * n + 1) * tries};
  }
  // Provisional supplement-1 policy, isolated so a later decision can replace it.
  static bool mayAdvance(bool previous_complete, uint64_t previous, const std::optional<double> & accepted) noexcept;
  static bool leaseExpired(int64_t now, int64_t sent) noexcept;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

  bool requestStart(const Request &);  // non-RT; single snapshot writer under input_mutex_
  bool keepalive(const Heartbeat &);   // non-RT; validates sender time, never refreshes by receipt time
  bool requestFinish();
  void requestStop() noexcept {stop_.store(true);}
  bool saveRecord(const std::string & path);  // non-RT, only after record_complete
  bool recordComplete() const noexcept {return complete_.load();}
  const std::vector<Row> & rowsAfterStop() const noexcept {return rows_;}
  const Row & finalRowAfterStop() const noexcept {return final_row_;}
  size_t rowCountAfterStop() const noexcept {return row_count_;}
  uint64_t droppedAfterStop() const noexcept {return dropped_;}
  Timing timingAfterStop() const noexcept {return timing_;}

protected:
  virtual int64_t steadyNow() const noexcept;
  virtual bool setCommand(size_t logical, double value);
  virtual std::optional<double> getCommand(size_t logical);
  virtual std::optional<double> getState(size_t logical);

private:
  friend struct NAxisTestAccess;
  bool bindInterfaces();
  bool validRequest(const Request &, int64_t now) const;
  bool attempt(const std::array<double, 16> &, uint64_t generation, Attempt &);
  void abort(Reason, Row &) noexcept;
  void append(Row &);
  size_t n_ = 0;
  unsigned tries_ = 1;  // provisional; configure-time parameter, never zero
  std::array<std::string, 16> command_keys_{};
  std::array<std::string, 10> state_keys_{};
  std::array<size_t, 16> command_index_{};
  std::array<size_t, 10> state_index_{};
  std::array<double, 3> lower_{{-kPi/6., -kPi/6., -kPi/6.}}, upper_{{kPi/6., kPi/6., kPi/6.}};
  double max_period_ = .005, state_limit_ = .60, max_torque_ = 5.;
  AtomicSnapshot<Request> requests_;
  AtomicSnapshot<Heartbeat> hearts_;
  std::mutex input_mutex_, save_mutex_;  // non-RT only
  Request input_request_{}, run_{};
  Heartbeat input_heart_{}, heart_{};
  std::atomic<bool> busy_{false}, terminal_{false}, active_{false}, complete_{false};
  std::atomic<bool> stop_{false}, finish_{false}, save_failed_{false};
  std::atomic<State> published_state_{State::kBoot};
  bool activated_once_ = false, previous_complete_ = false;
  State state_ = State::kBoot;
  Reason reason_ = Reason::kNone;
  uint64_t activation_ = 0, cycle_ = 0, generation_ = 0, holds_ = 0, max_holds_ = 0, failures_ = 0;
  int64_t armed_ns_ = 0;
  double elapsed_ = 0.;
  std::optional<hil::SineTrajectory> sine_;
  std::vector<Row> rows_;
  size_t row_count_ = 0;
  uint64_t dropped_ = 0;
  Row final_row_{};  // emergency final observation even if the bounded buffer is full
  Timing timing_{};
};
}  // namespace bxi_hardware
#endif
