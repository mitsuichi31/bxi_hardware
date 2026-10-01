#include "bxi_hardware/hil_trajectory_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <thread>

#include "pluginlib/class_list_macros.hpp"

namespace bxi_hardware
{

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kMinLeaseS = 0.1;
constexpr double kMaxLeaseS = 2.0;
constexpr double kMaxKd = 2.0;
constexpr double kMaxHoldS = 10.0;
constexpr double kMaxAmplitude = 1.5707963267948966;
constexpr double kMaxFrequency = 5.0;
constexpr double kMaxCycles = 20.0;
constexpr double kMinRampCycles = 0.5;
constexpr double kMaxRampCycles = 5.0;
constexpr double kMaxFriction = 1.0;
constexpr double kMaxStribeckVelocity = 2.0;
constexpr double kMinFrictionVelocity = 0.005;
constexpr double kMaxTestDelayS = 30.0;

// The test-only service delays are accepted only in a process started with this set.
bool testFaultsAllowed()
{
  const char * value = std::getenv("BXI_ALLOW_TEST_FAULTS");
  return value != nullptr && std::string(value) == "1";
}

// The log line tells a vcan test that the callback (and so the node's callbacks) is blocked.
void sleepForTest(double seconds, const rclcpp::Logger & logger, const char * service)
{
  if (seconds > 0.0) {
    RCLCPP_WARN(logger, "試験用の遅延：~/%s の処理を %.3f 秒止める", service, seconds);
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
  }
}
constexpr double kMaxFrictionVelocity = 1.0;
constexpr double kMaxKpLimit = 30.0;

using State = HilTrajectoryController::State;
using EndReason = HilTrajectoryController::EndReason;

bool isActive(State state)
{
  return state == State::kArmed || state == State::kRunning || state == State::kHolding;
}

int64_t realSteadyNs()
{
  return std::chrono::steady_clock::now().time_since_epoch().count();
}
}  // namespace

const char * HilTrajectoryController::endReasonName(EndReason reason)
{
  switch (reason) {
    case EndReason::kNone: return "none";
    case EndReason::kRelease: return "release";
    case EndReason::kFinish: return "finish";
    case EndReason::kStop: return "stop";
    case EndReason::kNoKeepalive: return "no keepalive";
    case EndReason::kLease: return "lease expired";
    case EndReason::kTorque: return "torque limit";
    case EndReason::kVelocity: return "velocity limit";
    case EndReason::kStateNotFinite: return "state not finite";
    case EndReason::kPeriod: return "period out of range";
    case EndReason::kStartRange: return "start out of range";
    case EndReason::kDuration: return "duration over limit";
    case EndReason::kDeactivated: return "deactivated";
    case EndReason::kWriteFailed: return "write failed";
  }
  return "unknown";
}

HilTrajectoryController::~HilTrajectoryController()
{
  // When the launch is stopped during or right after a run (as the HIL runner does on an NG), the
  // deactivation completes the record but the status timer never runs again. controller_manager
  // destroys the controllers only after it has shut the hardware down, so writing the file here
  // does not delay the HWIF's disable (on_deactivate() and on_shutdown() run before it).
  try {
    writeCompletedRecord();
  } catch (...) {
  }
}

controller_interface::CallbackReturn HilTrajectoryController::on_init()
{
  auto_declare<std::string>("joint", "joint1");
  auto_declare<double>("lower", -0.5);
  auto_declare<double>("upper", 0.5);
  auto_declare<double>("max_kp", 20.0);
  auto_declare<double>("max_torque", 5.0);
  auto_declare<double>("max_velocity", 2.0);
  auto_declare<double>("max_amplitude", 0.5);
  auto_declare<bool>("fixed_sine_center", false);
  auto_declare<double>("sine_center", 0.0);
  auto_declare<double>("max_period", 0.005);
  auto_declare<double>("arm_timeout", 1.0);
  auto_declare<double>("record_tail", 0.5);
  auto_declare<double>("record_seconds", 120.0);
  auto_declare<double>("max_run_seconds", 60.0);
  auto_declare<std::string>("output_dir", "");
  auto_declare<double>("test_fault.start_delay", 0.0);
  auto_declare<double>("test_fault.stop_delay", 0.0);
  auto_declare<double>("test_fault.finish_delay", 0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
HilTrajectoryController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
    {joint_ + "/position", joint_ + "/velocity", joint_ + "/kp", joint_ + "/kd",
      joint_ + "/effort"}};
}

controller_interface::InterfaceConfiguration
HilTrajectoryController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
    {joint_ + "/position", joint_ + "/velocity", joint_ + "/effort"}};
}

controller_interface::CallbackReturn HilTrajectoryController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  joint_ = node->get_parameter("joint").as_string();
  lower_ = node->get_parameter("lower").as_double();
  upper_ = node->get_parameter("upper").as_double();
  max_kp_ = node->get_parameter("max_kp").as_double();
  max_torque_ = node->get_parameter("max_torque").as_double();
  max_velocity_ = node->get_parameter("max_velocity").as_double();
  max_amplitude_ = node->get_parameter("max_amplitude").as_double();
  fixed_sine_center_ = node->get_parameter("fixed_sine_center").as_bool();
  sine_center_ = node->get_parameter("sine_center").as_double();
  max_period_s_ = node->get_parameter("max_period").as_double();
  arm_timeout_s_ = node->get_parameter("arm_timeout").as_double();
  record_tail_s_ = node->get_parameter("record_tail").as_double();
  record_seconds_ = node->get_parameter("record_seconds").as_double();
  max_run_seconds_ = node->get_parameter("max_run_seconds").as_double();
  output_dir_ = node->get_parameter("output_dir").as_string();
  test_start_delay_s_ = node->get_parameter("test_fault.start_delay").as_double();
  test_stop_delay_s_ = node->get_parameter("test_fault.stop_delay").as_double();
  test_finish_delay_s_ = node->get_parameter("test_fault.finish_delay").as_double();
  const double test_delays[] = {test_start_delay_s_, test_stop_delay_s_, test_finish_delay_s_};
  bool test_faults = false;
  for (const double delay : test_delays) {
    if (!std::isfinite(delay) || delay < 0.0 || delay > kMaxTestDelayS) {
      RCLCPP_ERROR(node->get_logger(), "test_fault の遅延は 0〜30 秒");
      return controller_interface::CallbackReturn::ERROR;
    }
    test_faults = test_faults || delay > 0.0;
  }
  if (test_faults) {
    if (!testFaultsAllowed()) {
      RCLCPP_ERROR(node->get_logger(),
        "test_fault は vcan の故障経路の試験専用で、BXI_ALLOW_TEST_FAULTS=1 のときだけ使える");
      return controller_interface::CallbackReturn::ERROR;
    }
    RCLCPP_WARN(node->get_logger(),
      "試験用のサービスの遅延が有効（start %.3f s、stop %.3f s、finish %.3f s）。実機では使わない",
      test_start_delay_s_, test_stop_delay_s_, test_finish_delay_s_);
  }
  if (output_dir_.empty()) {
    if (const char * dir = std::getenv("BXI_WORKER_TIMING_DIR")) {
      output_dir_ = dir;
    }
  }
  const auto finite_positive = [](double v) {return std::isfinite(v) && v > 0.0;};
  if (joint_.empty() || !std::isfinite(lower_) || !std::isfinite(upper_) || lower_ >= upper_ ||
    !finite_positive(max_kp_) || max_kp_ > kMaxKpLimit || !finite_positive(max_torque_) ||
    !finite_positive(max_velocity_) || !finite_positive(max_amplitude_) ||
    max_amplitude_ > kMaxAmplitude || !std::isfinite(sine_center_) ||
    (fixed_sine_center_ && (sine_center_ < lower_ || sine_center_ > upper_)) ||
    !finite_positive(max_period_s_) ||
    !finite_positive(arm_timeout_s_) || !std::isfinite(record_tail_s_) || record_tail_s_ < 0.0 ||
    !finite_positive(record_seconds_) || !finite_positive(max_run_seconds_))
  {
    RCLCPP_ERROR(node->get_logger(), "パラメータが範囲外（joint、lower < upper、0 < max_kp ≤ 30 など）");
    return controller_interface::CallbackReturn::ERROR;
  }
  const double rate = get_update_rate() > 0 ? static_cast<double>(get_update_rate()) : 1000.0;
  rows_.assign(static_cast<size_t>(std::ceil(record_seconds_ * rate)), Row{});

  start_service_ = node->create_service<srv::StartHilTrajectory>("~/start",
      [this](const std::shared_ptr<srv::StartHilTrajectory::Request> request,
      std::shared_ptr<srv::StartHilTrajectory::Response> response) {
        sleepForTest(test_start_delay_s_, get_node()->get_logger(), "start");
        const auto result = requestStart(*request);
        response->accepted = result.accepted;
        response->request_id = result.id;
        response->message = result.message;
      });
  finish_service_ = node->create_service<std_srvs::srv::Trigger>("~/finish",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        sleepForTest(test_finish_delay_s_, get_node()->get_logger(), "finish");
        response->success = requestFinish(&response->message);
      });
  stop_service_ = node->create_service<std_srvs::srv::Trigger>("~/stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        sleepForTest(test_stop_delay_s_, get_node()->get_logger(), "stop");
        requestStop();
        response->success = true;
        response->message = "受け付けた（適用は ~/status の applied_stop_count で確かめる）";
      });
  keepalive_sub_ = node->create_subscription<std_msgs::msg::UInt32>("~/keepalive", 10,
      [this](const std_msgs::msg::UInt32::SharedPtr msg) {onKeepalive(msg->data);});
  status_pub_ = node->create_publisher<msg::HilTrajectoryStatus>("~/status", 10);
  status_timer_ = node->create_wall_timer(std::chrono::milliseconds(20), [this]() {
        // After a failed CSV write, retry once a second (the record and busy_ stay meanwhile).
        const int64_t now = realSteadyNs();
        bool failed = false;
        {
          std::lock_guard<std::mutex> lock(csv_mutex_);
          failed = csv_failed_;
        }
        if (!failed || now - last_csv_attempt_ns_ >= 1'000'000'000) {
          last_csv_attempt_ns_ = now;
          writeCompletedRecord();
        }
        const auto s = snapshot();
        msg::HilTrajectoryStatus status;
        status.state = static_cast<uint8_t>(s.state);
        status.applied_request_id = s.applied_id;
        status.applied_stop_count = s.applied_stop_count;
        status.end_reason = endReasonName(s.end_reason);
        status.elapsed = s.elapsed;
        status.target_position = s.target_position;
        status.position = s.position;
        status.recorded_cycles = s.recorded_cycles;
        status.dropped_cycles = s.dropped_cycles;
        status.write_failures = s.write_failures;
        status.release_write_pending = s.release_write_pending;
        {
          std::lock_guard<std::mutex> lock(csv_mutex_);
          status.csv_path = csv_path_;
          status.record_state = s.recording ? msg::HilTrajectoryStatus::RECORD_RECORDING :
          (csv_failed_ ? msg::HilTrajectoryStatus::RECORD_FAILED :
          (csv_path_.empty() ? msg::HilTrajectoryStatus::RECORD_NONE :
          msg::HilTrajectoryStatus::RECORD_WRITTEN));
        }
        status_pub_->publish(status);
      });
  RCLCPP_INFO(node->get_logger(), "HilTrajectoryController: %s、記録 %zu 周期、出力先 %s",
    joint_.c_str(), rows_.size(), output_dir_.empty() ? "（なし）" : output_dir_.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HilTrajectoryController::on_activate(
  const rclcpp_lifecycle::State &)
{
  state_ = State::kIdle;
  end_reason_ = EndReason::kNone;
  release_write_pending_ = false;
  // Stops accepted while inactive do not carry over to the next run.
  applied_stop_count_ = stop_count_.load(std::memory_order_acquire);
  rt_applied_stop_count_.store(applied_stop_count_, std::memory_order_release);
  publishRtState();
  if (!writeOutputs(kNaN, kNaN, kNaN, kNaN, kNaN)) {
    RCLCPP_ERROR(get_node()->get_logger(), "有効化で command interface に書き込めない");
    return controller_interface::CallbackReturn::ERROR;
  }
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  snapshot_ = Snapshot{};
  snapshot_.applied_stop_count = applied_stop_count_;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HilTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // NaN: HWIF writes Kp = Kd = 0 at its next write(). On a shutdown, that write may never come;
  // the HWIF's disable is what confirms the motor is limp.
  if (!writeOutputs(kNaN, kNaN, kNaN, kNaN, kNaN)) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "無効化で command interface に書き込めない（力が抜けたことは HWIF の disable で確かめる）");
  }
  const int64_t now = steadyNowNs();
  if (isActive(state_)) {
    endRun(EndReason::kDeactivated, now);
  }
  publishRtState();
  if (recording_) {
    recording_ = false;
    record_end_reason_ = end_reason_;
    record_complete_.store(true, std::memory_order_release);
  }
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  snapshot_.state = state_;
  snapshot_.end_reason = end_reason_;
  snapshot_.recording = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

bool HilTrajectoryController::writeCommand(size_t index, double value)
{
  return command_interfaces_[index].set_value(value);
}

bool HilTrajectoryController::writeOutputs(
  double position, double velocity, double kp, double kd, double feedforward)
{
  // Every value is attempted even after a failure, so that as much as possible is current.
  bool ok = writeCommand(0, position);
  ok = writeCommand(1, velocity) && ok;
  ok = writeCommand(2, kp) && ok;
  ok = writeCommand(3, kd) && ok;
  ok = writeCommand(4, feedforward) && ok;
  return ok;
}

bool HilTrajectoryController::writeReleased(double position)
{
  // Zero gains and torque at the current position; NaN (HWIF also zero gains) if unknown.
  return writeOutputs(std::isfinite(position) ? position : kNaN, 0.0, 0.0, 0.0, 0.0);
}

void HilTrajectoryController::publishRtState()
{
  const uint64_t id = state_ == State::kIdle ? 0U : run_.id;
  rt_state_id_.store((static_cast<uint64_t>(state_) << 32) | id, std::memory_order_release);
}

void HilTrajectoryController::endRun(EndReason reason, int64_t now_ns)
{
  end_reason_ = reason;
  state_ = (reason == EndReason::kRelease || reason == EndReason::kFinish ||
    reason == EndReason::kStop) ? State::kReleased : State::kAborted;
  end_ns_ = now_ns;
  release_write_pending_ = true;  // cleared only after a complete released write succeeds
}

controller_interface::return_type HilTrajectoryController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const auto update_start = std::chrono::steady_clock::now();
  const int64_t now = steadyNowNs();
  const double dt = period.seconds();
  const double position = state_interfaces_[0].get_optional().value_or(kNaN);
  const double velocity = state_interfaces_[1].get_optional().value_or(kNaN);
  const double effort = state_interfaces_[2].get_optional().value_or(kNaN);
  const bool state_finite =
    std::isfinite(position) && std::isfinite(velocity) && std::isfinite(effort);

  // 1. Stop first: it cancels a pending start and ends any run.
  const uint32_t stops = stop_count_.load(std::memory_order_acquire);
  if (stops != applied_stop_count_) {
    applied_stop_count_ = stops;
    if (isActive(state_)) {
      endRun(EndReason::kStop, now);
    }
    rt_applied_stop_count_.store(stops, std::memory_order_release);
  }

  // 2. A new start, only when idle and the previous record has been handed over.
  if (!isActive(state_) && !recording_ && !record_complete_.load(std::memory_order_acquire)) {
    std::unique_lock<std::mutex> lock(start_mutex_, std::try_to_lock);
    if (lock.owns_lock() && pending_start_.has_value()) {
      const StartRequest request = *pending_start_;
      pending_start_.reset();
      // Before the lock is released: ~/start checks busy_ under the same lock, so it can never
      // see neither the pending request nor the run it became.
      busy_.store(true, std::memory_order_release);
      run_ = request;
      state_ = State::kArmed;
      publishRtState();
      lock.unlock();
      end_reason_ = EndReason::kNone;
      armed_ns_ = now;
      last_keepalive_ns_ = 0;
      elapsed_ = 0.0;
      ramp_.reset();
      sine_.reset();
      last_target_ = kNaN;
      recording_ = true;
      rows_written_ = 0;
      rows_dropped_ = 0;
      release_write_pending_ = false;
      record_run_ = request;
      record_id_.store(request.id, std::memory_order_relaxed);
    }
  }

  // Keepalives count only for the applied request and only after it was accepted.
  if (isActive(state_)) {
    std::unique_lock<std::mutex> lock(keepalive_mutex_, std::try_to_lock);
    if (lock.owns_lock() && keepalive_id_ == run_.id && keepalive_ns_ >= run_.accepted_ns) {
      last_keepalive_ns_ = std::max(last_keepalive_ns_, keepalive_ns_);
    }
  }

  // 3. The trajectory advances by the accumulated period; an implausible period ends the run.
  const bool period_ok = std::isfinite(dt) && dt > 0.0 && dt <= max_period_s_;
  if ((state_ == State::kRunning || state_ == State::kHolding) && !period_ok) {
    endRun(EndReason::kPeriod, now);
  }

  // 4. Armed: start on the first matching keepalive, or give up without moving.
  if (state_ == State::kArmed) {
    if (last_keepalive_ns_ > 0) {
      if (!state_finite) {
        endRun(EndReason::kStateNotFinite, now);
      } else {
        start_position_ = position;
        switch (run_.kind) {
          case Kind::kRamp:
            ramp_.emplace(position, run_.target, run_.speed);
            run_duration_ = ramp_->rampTime() + run_.hold;
            break;
          case Kind::kSine:
            sine_.emplace(fixed_sine_center_ ? sine_center_ : position, run_.amplitude, run_.frequency, run_.cycles, run_.ramp_cycles);
            run_duration_ = sine_->duration() + run_.hold;
            break;
          case Kind::kHold:
            run_duration_ = run_.hold;
            break;
        }
        if (run_.kind == Kind::kSine &&
          ((fixed_sine_center_ ? sine_center_ : position) - run_.amplitude < lower_ ||
          (fixed_sine_center_ ? sine_center_ : position) + run_.amplitude > upper_))
        {
          endRun(EndReason::kStartRange, now);
        } else if (run_duration_ > max_run_seconds_) {
          endRun(EndReason::kDuration, now);
        } else {
          state_ = State::kRunning;
          elapsed_ = 0.0;
          first_running_cycle_ = true;
        }
      }
    } else if (static_cast<double>(now - armed_ns_) * 1e-9 > arm_timeout_s_) {
      endRun(EndReason::kNoKeepalive, now);
    }
  }

  const double lease_s = static_cast<double>(run_.lease_ns) * 1e-9;
  const auto lease_expired = [&]() {
      return static_cast<double>(now - last_keepalive_ns_) * 1e-9 > lease_s;
    };

  // 5. Running.
  hil::Setpoint setpoint{kNaN, kNaN};
  double feedforward = 0.0;
  if (state_ == State::kRunning) {
    if (!first_running_cycle_) {
      elapsed_ += dt;
    }
    first_running_cycle_ = false;
    switch (run_.kind) {
      case Kind::kRamp:
        setpoint = ramp_->at(elapsed_);
        break;
      case Kind::kSine:
        setpoint = sine_->at(elapsed_);
        feedforward = hil::frictionTorque(setpoint.velocity, run_.friction_kinetic,
            run_.friction_velocity, run_.friction_static, run_.stribeck_velocity);
        break;
      case Kind::kHold:
        setpoint = {start_position_, 0.0};
        break;
    }
    if (lease_expired()) {
      endRun(EndReason::kLease, now);
    } else if (!state_finite) {
      endRun(EndReason::kStateNotFinite, now);
    } else if (std::abs(setpoint.velocity) > max_velocity_) {
      endRun(EndReason::kVelocity, now);
    } else if (run_.kp * std::abs(setpoint.position - position) + std::abs(feedforward) >
      max_torque_)
    {
      endRun(EndReason::kTorque, now);
    } else if (elapsed_ >= run_duration_) {
      final_setpoint_ = {setpoint.position, 0.0};
      if (run_.release) {
        endRun(EndReason::kRelease, now);
      } else {
        state_ = State::kHolding;
      }
    }
  }

  // 6. Holding: the final target until finish, stop, a lease expiry or a limit.
  if (state_ == State::kHolding) {
    if (lease_expired()) {
      endRun(EndReason::kLease, now);
    } else if (!state_finite) {
      endRun(EndReason::kStateNotFinite, now);
    } else if (run_.kp * std::abs(final_setpoint_.position - position) > max_torque_) {
      endRun(EndReason::kTorque, now);
    } else if (finish_for_id_.load(std::memory_order_acquire) == run_.id) {
      endRun(EndReason::kFinish, now);
    }
  }

  // Outputs. If an interface refuses a value while the outputs carry gains, the cycle's command
  // is not what was intended: end the run and write the released outputs at once. Released
  // outputs that are refused are written again every cycle (the state stays released/aborted).
  bool wrote = true;
  switch (state_) {
    case State::kIdle:
    case State::kArmed:
      wrote = writeOutputs(kNaN, kNaN, kNaN, kNaN, kNaN);
      last_target_ = kNaN;
      break;
    case State::kRunning:
      wrote = writeOutputs(setpoint.position, setpoint.velocity, run_.kp, run_.kd, feedforward);
      last_target_ = setpoint.position;
      break;
    case State::kHolding:
      feedforward = 0.0;
      wrote = writeOutputs(final_setpoint_.position, 0.0, run_.kp, run_.kd, 0.0);
      last_target_ = final_setpoint_.position;
      break;
    case State::kReleased:
    case State::kAborted:
      wrote = writeReleased(position);
      last_target_ = kNaN;
      break;
  }
  if (!wrote && (state_ == State::kRunning || state_ == State::kHolding)) {
    endRun(EndReason::kWriteFailed, now);
    // Keep the first refusal in write_ok even when this immediate release succeeds.
    if (writeReleased(position)) {
      release_write_pending_ = false;
    }
    last_target_ = kNaN;
  } else if ((state_ == State::kReleased || state_ == State::kAborted) && wrote) {
    release_write_pending_ = false;
  }
  if (!wrote) {
    // One failed cycle, whether one or both write attempts were refused.
    ++write_failures_;
  }

  // Record.
  if (recording_) {
    if (rows_written_ < rows_.size()) {
      Row & row = rows_[rows_written_++];
      row.steady_ns = now;
      row.ros_time_s = time.seconds();
      row.period_s = dt;
      row.elapsed_s = elapsed_;
      const bool gains = state_ == State::kRunning || state_ == State::kHolding;
      row.target_position = state_ == State::kRunning ? setpoint.position :
        (state_ == State::kHolding ? final_setpoint_.position : kNaN);
      row.target_velocity = state_ == State::kRunning ? setpoint.velocity : (gains ? 0.0 : kNaN);
      row.kp = gains ? run_.kp : (state_ == State::kArmed ? kNaN : 0.0);
      row.kd = gains ? run_.kd : (state_ == State::kArmed ? kNaN : 0.0);
      row.feedforward = state_ == State::kRunning ? feedforward : (gains ? 0.0 : kNaN);
      row.position = position;
      row.velocity = velocity;
      row.effort = effort;
      row.state = static_cast<uint8_t>(state_);
      row.end_reason = static_cast<uint8_t>(end_reason_);
      row.write_ok = wrote ? 1 : 0;
      row.update_ns = (std::chrono::steady_clock::now() - update_start).count();
    } else {
      ++rows_dropped_;
    }
    record_count_.store(rows_written_, std::memory_order_relaxed);
    record_dropped_.store(rows_dropped_, std::memory_order_relaxed);
    if ((state_ == State::kReleased || state_ == State::kAborted) &&
      !release_write_pending_ &&
      static_cast<double>(now - end_ns_) * 1e-9 >= record_tail_s_)
    {
      recording_ = false;
      record_end_reason_ = end_reason_;
      record_complete_.store(true, std::memory_order_release);
    }
  }

  // Status for the non-real-time side: the atomics every cycle, the snapshot when not busy.
  publishRtState();
  {
    std::unique_lock<std::mutex> lock(snapshot_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      snapshot_.state = state_;
      snapshot_.applied_id = isActive(state_) || state_ == State::kReleased ||
        state_ == State::kAborted ? run_.id : 0;
      snapshot_.applied_stop_count = applied_stop_count_;
      snapshot_.end_reason = end_reason_;
      snapshot_.elapsed = elapsed_;
      snapshot_.target_position = last_target_;
      snapshot_.position = position;
      snapshot_.recording = recording_;
      snapshot_.recorded_cycles = rows_written_;
      snapshot_.dropped_cycles = rows_dropped_;
      snapshot_.write_failures = write_failures_;
      snapshot_.release_write_pending = release_write_pending_;
    }
  }
  return controller_interface::return_type::OK;
}

std::string HilTrajectoryController::validate(const srv::StartHilTrajectory::Request & r) const
{
  const double numbers[] = {r.target, r.speed, r.amplitude, r.frequency, r.cycles, r.ramp_cycles,
    r.friction_kinetic, r.friction_static, r.friction_velocity, r.stribeck_velocity, r.kp, r.kd,
    r.hold, r.lease};
  for (const double v : numbers) {
    if (!std::isfinite(v)) {
      return "有限でない数値がある";
    }
  }
  if (r.kind > srv::StartHilTrajectory::Request::SINE) {
    return "種類が不正";
  }
  if (!(r.kp > 0.0 && r.kp <= max_kp_)) {
    return "Kp は 0 より大きく max_kp 以下";
  }
  if (!(r.kd >= 0.0 && r.kd <= kMaxKd)) {
    return "Kd は 0〜2";
  }
  if (!(r.hold >= 0.0 && r.hold <= kMaxHoldS)) {
    return "hold は 0〜10 秒";
  }
  if (!(r.lease >= kMinLeaseS && r.lease <= kMaxLeaseS)) {
    return "lease は 0.1〜2.0 秒";
  }
  if (r.kind == srv::StartHilTrajectory::Request::RAMP) {
    if (!(r.target >= lower_ && r.target <= upper_)) {
      return "目標が関節の範囲の外";
    }
    if (!(r.speed > 0.0 && r.speed <= max_velocity_)) {
      return "速さは 0 より大きく max_velocity 以下";
    }
  } else if (r.kind == srv::StartHilTrajectory::Request::SINE) {
    if (!(r.amplitude > 0.0 && r.amplitude <= max_amplitude_)) {
      return "振幅は 0 より大きく max_amplitude 以下";
    }
    if (!(r.frequency > 0.0 && r.frequency <= kMaxFrequency)) {
      return "周波数は 0 より大きく 5 Hz 以下";
    }
    if (!(r.cycles >= 0.0 && r.cycles <= kMaxCycles)) {
      return "周期数は 0〜20";
    }
    if (!(r.ramp_cycles >= kMinRampCycles && r.ramp_cycles <= kMaxRampCycles)) {
      return "立ち上げの周期数は 0.5〜5";
    }
    const double fs = r.friction_static < 0.0 ? r.friction_kinetic : r.friction_static;
    if (!(r.friction_kinetic >= 0.0 && r.friction_kinetic <= kMaxFriction &&
      fs >= r.friction_kinetic && fs <= kMaxFriction))
    {
      return "摩擦の打ち消しは 0 ≤ 動摩擦 ≤ 静止摩擦 ≤ 1 N·m";
    }
    if (!(r.stribeck_velocity > 0.0 && r.stribeck_velocity <= kMaxStribeckVelocity &&
      r.friction_velocity >= kMinFrictionVelocity && r.friction_velocity <= kMaxFrictionVelocity))
    {
      return "Stribeck の速さは 0〜2 rad/s、切り替え幅は 0.005〜1 rad/s";
    }
    const hil::SineTrajectory sine(0.0, r.amplitude, r.frequency, r.cycles, r.ramp_cycles);
    if (sine.velocityUpperBound() > max_velocity_) {
      return "目標速度の上界が max_velocity を超える（立ち上げ・立ち下げの間を含む）";
    }
    if (sine.duration() + r.hold > max_run_seconds_) {
      return "実行の時間の合計が max_run_seconds を超える";
    }
  }
  return "";
}

HilTrajectoryController::StartResult HilTrajectoryController::requestStart(
  const srv::StartHilTrajectory::Request & request)
{
  StartResult result;
  result.message = validate(request);
  if (!result.message.empty()) {
    return result;
  }
  std::lock_guard<std::mutex> lock(start_mutex_);
  if (pending_start_.has_value()) {
    result.message = "まだ適用していない開始要求がある";
  } else if (busy_.load(std::memory_order_acquire)) {
    bool failed = false;
    {
      std::lock_guard<std::mutex> csv_lock(csv_mutex_);
      failed = csv_failed_;
    }
    result.message = failed ? "前の実行の記録を書き出せていない（再試行中）" :
      "実行中、または前の実行の記録を書き出す前";
  } else if (stop_count_.load(std::memory_order_acquire) !=
    rt_applied_stop_count_.load(std::memory_order_acquire))
  {
    result.message = "停止を適用している最中";
  }
  if (!result.message.empty()) {
    return result;
  }
  StartRequest start;
  start.id = next_id_++;
  start.kind = static_cast<Kind>(request.kind);
  start.target = request.target;
  start.speed = request.speed;
  start.amplitude = request.amplitude;
  start.frequency = request.frequency;
  start.cycles = request.cycles;
  start.ramp_cycles = request.ramp_cycles;
  start.friction_kinetic = request.friction_kinetic;
  start.friction_static = request.friction_static;
  start.friction_velocity = request.friction_velocity;
  start.stribeck_velocity = request.stribeck_velocity;
  start.kp = request.kp;
  start.kd = request.kd;
  start.hold = request.hold;
  start.release = request.release;
  start.lease_ns = static_cast<int64_t>(request.lease * 1e9);
  start.accepted_ns = steadyNowNs();
  pending_start_ = start;
  result.accepted = true;
  result.id = start.id;
  result.message = "受け付けた（適用は ~/status の applied_request_id と state で確かめる）";
  return result;
}

bool HilTrajectoryController::requestFinish(std::string * message)
{
  const uint64_t state_id = rt_state_id_.load(std::memory_order_acquire);
  if (static_cast<State>(state_id >> 32) != State::kHolding) {
    if (message != nullptr) {
      *message = "保持中ではない";
    }
    return false;
  }
  finish_for_id_.store(static_cast<uint32_t>(state_id & 0xFFFFFFFFU), std::memory_order_release);
  if (message != nullptr) {
    *message = "受け付けた（適用は ~/status の end_reason で確かめる）";
  }
  return true;
}

void HilTrajectoryController::requestStop()
{
  // Under the lock update() takes a pending start with: a start accepted before this stop is either
  // still pending (cleared here) or already applied (ended by update() on the new count).
  std::lock_guard<std::mutex> lock(start_mutex_);
  pending_start_.reset();
  stop_count_.fetch_add(1, std::memory_order_acq_rel);
}

void HilTrajectoryController::onKeepalive(uint32_t id)
{
  std::lock_guard<std::mutex> lock(keepalive_mutex_);
  keepalive_id_ = id;
  keepalive_ns_ = steadyNowNs();
}

HilTrajectoryController::Snapshot HilTrajectoryController::snapshot() const
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return snapshot_;
}

std::string HilTrajectoryController::csvPath() const
{
  std::lock_guard<std::mutex> lock(csv_mutex_);
  return csv_path_;
}

bool HilTrajectoryController::writeCompletedRecord()
{
  if (!record_complete_.load(std::memory_order_acquire)) {
    return false;
  }
  const uint64_t count = record_count_.load(std::memory_order_relaxed);
  const uint64_t dropped = record_dropped_.load(std::memory_order_relaxed);
  bool written = false;
  std::string path;
  if (!output_dir_.empty()) {
    const auto stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    path = (std::filesystem::path(output_dir_) / ("hil_trajectory_" +
      std::to_string(record_run_.id) + "_" + std::to_string(stamp) + ".csv")).string();
    std::ofstream file(path);
    if (file) {
      const auto & r = record_run_;
      file << std::setprecision(17) << "#run_id=" << r.id << ",kind=" << static_cast<int>(r.kind) <<
        ",fixed_sine_center=" << (fixed_sine_center_ ? 1 : 0) <<
        ",sine_center=" << sine_center_ << ",target=" << r.target << ",speed=" << r.speed << ",amplitude=" << r.amplitude <<
        ",frequency=" << r.frequency << ",cycles=" << r.cycles << ",ramp_cycles=" <<
        r.ramp_cycles << ",friction_kinetic=" << r.friction_kinetic << ",friction_static=" <<
        r.friction_static << ",friction_velocity=" << r.friction_velocity <<
        ",stribeck_velocity=" << r.stribeck_velocity << ",kp=" << r.kp << ",kd=" << r.kd <<
        ",hold=" << r.hold << ",release=" << (r.release ? 1 : 0) << ",lease_s=" <<
        static_cast<double>(r.lease_ns) * 1e-9 << ",end_reason=" <<
        endReasonName(record_end_reason_) << ",rows=" << count << ",dropped_cycles=" <<
        dropped << '\n';
      file << "steady_ns,ros_time_s,period_s,elapsed_s,target_position,target_velocity,kp,kd,"
        "feedforward,position,velocity,effort,update_ns,state,end_reason,write_ok\n";
      for (uint64_t i = 0; i < count && i < rows_.size(); ++i) {
        const Row & row = rows_[i];
        file << row.steady_ns << ',' << row.ros_time_s << ',' << row.period_s << ',' <<
          row.elapsed_s << ',' << row.target_position << ',' << row.target_velocity << ',' <<
          row.kp << ',' << row.kd << ',' << row.feedforward << ',' << row.position << ',' <<
          row.velocity << ',' << row.effort << ',' << row.update_ns << ',' <<
          static_cast<int>(row.state) << ',' << static_cast<int>(row.end_reason) << ',' <<
          static_cast<int>(row.write_ok) << '\n';
      }
      written = static_cast<bool>(file);
    }
  }
  if (!output_dir_.empty() && !written) {
    // Keep the record and busy_: the next start is refused until a retry writes it.
    {
      std::lock_guard<std::mutex> lock(csv_mutex_);
      csv_path_.clear();
      csv_failed_ = true;
    }
    RCLCPP_ERROR(get_node()->get_logger(), "記録を書き出せない: %s（記録を残して再試行する）",
      path.c_str());
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(csv_mutex_);
    csv_path_ = written ? path : "";
    csv_failed_ = false;
  }
  if (written) {
    RCLCPP_INFO(get_node()->get_logger(), "記録を書き出した: %s（%llu 周期、欠落 %llu）",
      path.c_str(), static_cast<unsigned long long>(count),
      static_cast<unsigned long long>(dropped));
  }
  // Without output_dir the record is discarded. Either way the next run may start.
  record_complete_.store(false, std::memory_order_release);
  busy_.store(false, std::memory_order_release);
  return written;
}

int64_t HilTrajectoryController::steadyNowNs() const
{
  return realSteadyNs();
}

}  // namespace bxi_hardware

PLUGINLIB_EXPORT_CLASS(
  bxi_hardware::HilTrajectoryController, controller_interface::ControllerInterface)
