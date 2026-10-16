#include "bxi_hardware/n_axis_trajectory_controller.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include "pluginlib/class_list_macros.hpp"

namespace bxi_hardware
{
namespace {
using CB = controller_interface::CallbackReturn;
using Ret = controller_interface::return_type;
constexpr double nan = std::numeric_limits<double>::quiet_NaN();
bool finite(const std::optional<double> & v) {return v && std::isfinite(*v);}
}
int64_t NAxisTrajectoryController::steadyNow() const noexcept
{return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();}
bool NAxisTrajectoryController::mayAdvance(bool done, uint64_t g, const std::optional<double> & a) noexcept
{return done && a && std::isfinite(*a) && *a == static_cast<double>(g);}
bool NAxisTrajectoryController::leaseExpired(int64_t now, int64_t sent) noexcept
{return sent <= 0 || sent > std::numeric_limits<int64_t>::max() - kLeaseNs || now < sent || now - sent >= kLeaseNs;}
CB NAxisTrajectoryController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {"joint1", "joint2"});
  auto_declare<int>("api_max_tries", 1);
  auto_declare<double>("record_seconds", 120.);
  auto_declare<double>("max_period", .005);
  auto_declare<double>("state_position_limit", .60);
  auto_declare<double>("max_torque", 5.);
  auto_declare<std::vector<double>>("lower", {});
  auto_declare<std::vector<double>>("upper", {});
  return CB::SUCCESS;
}
controller_interface::InterfaceConfiguration NAxisTrajectoryController::command_interface_configuration() const
{
  std::vector<std::string> keys(command_keys_.begin(), command_keys_.begin() + (n_ ? 5*n_+1 : 0));
  return {controller_interface::interface_configuration_type::INDIVIDUAL, keys};
}
controller_interface::InterfaceConfiguration NAxisTrajectoryController::state_interface_configuration() const
{
  std::vector<std::string> keys(state_keys_.begin(), state_keys_.begin() + (n_ ? 3*n_+1 : 0));
  return {controller_interface::interface_configuration_type::INDIVIDUAL, keys};
}
CB NAxisTrajectoryController::on_configure(const rclcpp_lifecycle::State &)
{
  timing_.configure_in = steadyNow();
  if (activated_once_) {return CB::ERROR;}
  auto node = get_node();
  const auto joints = node->get_parameter("joints").as_string_array();
  if (joints.size() < 2 || joints.size() > 3) {return CB::ERROR;}
  n_ = joints.size();
  const auto tries = node->get_parameter("api_max_tries").as_int();
  const double seconds = node->get_parameter("record_seconds").as_double();
  max_period_ = node->get_parameter("max_period").as_double();
  state_limit_ = node->get_parameter("state_position_limit").as_double();
  max_torque_ = node->get_parameter("max_torque").as_double();
  if (tries < 1 || tries > 10 || !std::isfinite(seconds) || seconds < 0 || seconds > 600 ||
    !std::isfinite(max_period_) || max_period_ <= 0 || max_period_ > .005 ||
    !std::isfinite(state_limit_) || state_limit_ < .05 || state_limit_ > .60 ||
    !std::isfinite(max_torque_) || max_torque_ <= 0 || max_torque_ > 5.) {return CB::ERROR;}
  tries_ = static_cast<unsigned>(tries);
  const auto low = node->get_parameter("lower").as_double_array();
  const auto high = node->get_parameter("upper").as_double_array();
  if ((!low.empty() && low.size() != n_) || (!high.empty() && high.size() != n_)) {return CB::ERROR;}
  const char * fields[] = {"position", "velocity", "kp", "kd", "effort"};
  for (size_t a = 0; a < n_; ++a) {
    if (joints[a] != "joint" + std::to_string(a+1)) {return CB::ERROR;}
    lower_[a] = low.empty() ? -kPi/6. : low[a];
    upper_[a] = high.empty() ? kPi/6. : high[a];
    if (!std::isfinite(lower_[a]) || !std::isfinite(upper_[a]) || lower_[a] >= upper_[a]) {return CB::ERROR;}
    for (size_t f = 0; f < 5; ++f) {command_keys_[a*5+f] = joints[a] + "/" + fields[f];}
    state_keys_[a*3] = joints[a] + "/position";
    state_keys_[a*3+1] = joints[a] + "/velocity";
    state_keys_[a*3+2] = joints[a] + "/effort";
  }
  command_keys_[n_*5] = "bxi_pair/commit_generation";
  state_keys_[n_*3] = "bxi_pair/accepted_generation";
  if (!requests_.lockFree() || !hearts_.lockFree()) {return CB::ERROR;}
  static_assert(std::atomic<bool>::is_always_lock_free && std::atomic<State>::is_always_lock_free,
    "G controller atomics must be lock-free");
  // Provisional: 1 kHz recording capacity, not a runtime performance claim.
  rows_.assign(static_cast<size_t>(std::ceil(seconds * 1000.)), Row{});
  timing_.configure_out = steadyNow();
  return CB::SUCCESS;
}
bool NAxisTrajectoryController::bindInterfaces()
{
  if (command_interfaces_.size() != 5*n_+1 || state_interfaces_.size() != 3*n_+1) {return false;}
  for (size_t k = 0; k < 5*n_+1; ++k) {
    size_t count = 0;
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
      if (command_interfaces_[i].get_name() == command_keys_[k]) {
        if (command_interfaces_[i].get_data_type().to_string() != "double") {return false;}
        command_index_[k] = i; ++count;
      }
    }
    if (count != 1) {return false;}
  }
  for (size_t k = 0; k < 3*n_+1; ++k) {
    size_t count = 0;
    for (size_t i = 0; i < state_interfaces_.size(); ++i) {
      if (state_interfaces_[i].get_name() == state_keys_[k]) {
        if (state_interfaces_[i].get_data_type().to_string() != "double") {return false;}
        state_index_[k] = i; ++count;
      }
    }
    if (count != 1) {return false;}
  }
  return true;
}
CB NAxisTrajectoryController::on_activate(const rclcpp_lifecycle::State &)
{
  timing_.activate_in = steadyNow();
  if (activated_once_ || n_ < 2 || n_ > 3) {return CB::ERROR;}
  activated_once_ = true;
  if (!bindInterfaces()) {terminal_ = true; return CB::ERROR;}
  // Invalid loans may throw; fail activation, never enter update with that loan.
  try {
    const auto accepted = getState(n_*3);
    if (!finite(accepted) || *accepted != 0.) {terminal_ = true; return CB::ERROR;}
  } catch (...) {terminal_ = true; return CB::ERROR;}
  ++activation_; state_ = State::kBoot; published_state_ = state_;
  active_ = true; timing_.activate_out = steadyNow();
  return CB::SUCCESS;
}
CB NAxisTrajectoryController::on_deactivate(const rclcpp_lifecycle::State &)
{
  // No command writes here: update's abnormal release must not be retried.
  terminal_ = true; active_ = false;
  if (reason_ == Reason::kNone) {reason_ = Reason::kDeactivated;}
  complete_.store(true, std::memory_order_release);
  return CB::SUCCESS;
}
bool NAxisTrajectoryController::setCommand(size_t i, double value)
{return command_interfaces_[command_index_[i]].set_value<double>(value, tries_);}
std::optional<double> NAxisTrajectoryController::getCommand(size_t i)
{return command_interfaces_[command_index_[i]].get_optional<double>(tries_);}
std::optional<double> NAxisTrajectoryController::getState(size_t i)
{return state_interfaces_[state_index_[i]].get_optional<double>(tries_);}

bool NAxisTrajectoryController::validRequest(const Request & r, int64_t now) const
{
  if (r.axes != n_ || !r.nonce || !r.id || leaseExpired(now, r.sent_ns) ||
    r.amplitude != kPi/6. || r.frequency != .2 || r.cycles != 5. || r.ramp_cycles != 1. || r.hold != 1.) {return false;}
  for (size_t a = 0; a < n_; ++a) {
    if (!std::isfinite(r.kp[a]) || r.kp[a] <= 0 || r.kp[a] > 20. ||
      !std::isfinite(r.kd[a]) || r.kd[a] < 0 || r.kd[a] > 1. ||
      -r.amplitude < lower_[a] || r.amplitude > upper_[a]) {return false;}
  }
  const hil::SineTrajectory q(0., r.amplitude, r.frequency, r.cycles, r.ramp_cycles);
  const auto acceleration = q.accelerationUpperBound();
  return q.velocityUpperBound() <= .70 && acceleration && *acceleration <= .90;
}
bool NAxisTrajectoryController::requestStart(const Request & request)
{
  std::lock_guard<std::mutex> lock(input_mutex_);
  if (!active_ || terminal_ || busy_ || save_failed_ || published_state_ != State::kIdle ||
    !validRequest(request, steadyNow())) {return false;}
  input_request_ = request; input_request_.received_ns = steadyNow();
  input_heart_ = {}; busy_ = true;
  requests_.publish(&input_request_);
  return true;
}
bool NAxisTrajectoryController::keepalive(const Heartbeat & value)
{
  std::lock_guard<std::mutex> lock(input_mutex_);
  if (!active_ || terminal_ || !busy_ || value.nonce != input_request_.nonce || value.id != input_request_.id ||
    !value.sequence || value.sequence <= input_heart_.sequence || value.sent_ns < input_request_.sent_ns ||
    value.sent_ns < input_heart_.sent_ns || leaseExpired(steadyNow(), value.sent_ns)) {return false;}
  input_heart_ = value; hearts_.publish(&value);
  return true;
}
bool NAxisTrajectoryController::requestFinish()
{
  if (!active_ || terminal_ || published_state_ != State::kHolding) {return false;}
  finish_ = true; return true;
}
bool NAxisTrajectoryController::attempt(const std::array<double, 16> & values, uint64_t g, Attempt & out)
{
  out.values = values;
  if (!g || g > kMaxGeneration) {return false;}
  bool ok = true;
  for (size_t i = 0; i < n_*5; ++i) {
    const auto bit = uint32_t{1} << i; out.write_attempted |= bit;
    if (!setCommand(i, values[i])) {ok = false; continue;}
    out.write_ok |= bit; out.read_attempted |= bit;
    const auto read = getCommand(i);
    if (!finite(read)) {out.unavailable |= bit; ok = false; continue;}
    out.readback[i] = *read;
    if (*read != values[i]) {out.mismatch |= bit; ok = false;} else {out.read_ok |= bit;}
  }
  if (!ok) {return false;}
  const size_t i = n_*5; const auto bit = uint32_t{1} << i;
  const double value = static_cast<double>(g); out.values[i] = value;
  out.write_attempted |= bit; out.commit_set_ns = steadyNow();
  if (!setCommand(i, value)) {return false;}
  out.write_ok |= bit; out.read_attempted |= bit;
  const auto read = getCommand(i); out.commit_read_ns = steadyNow();
  if (!finite(read)) {out.unavailable |= bit; return false;}
  out.readback[i] = *read;
  if (*read != value) {out.mismatch |= bit; return false;}
  out.read_ok |= bit; return true;
}
void NAxisTrajectoryController::abort(Reason reason, Row & row) noexcept
{
  state_ = State::kAborted; reason_ = reason; terminal_ = true;
  row.reason = reason; row.detected_ns = steadyNow();
}
void NAxisTrajectoryController::append(Row & row)
{
  row.after = state_; row.reason = reason_; row.holds = holds_; row.max_holds = max_holds_;
  row.failures = failures_; row.elapsed = elapsed_; row.end_ns = steadyNow(); final_row_ = row;
  if (row_count_ < rows_.size()) {rows_[row_count_++] = row;} else {++dropped_;}
  published_state_ = state_;
  if (state_ == State::kAborted) {complete_.store(true, std::memory_order_release);}
}
Ret NAxisTrajectoryController::update(const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (state_ == State::kAborted || !active_) {return Ret::ERROR;}
  Row row; row.axes = n_; row.activation = activation_; row.cycle = ++cycle_;
  row.before = state_; row.start_ns = steadyNow();
  if (!timing_.first_update) {timing_.first_update = row.start_ns;}
  std::array<double, 16> values{}, released{};
  for (size_t a = 0; a < n_; ++a) {
    for (size_t f = 0; f < 3; ++f) {
      const size_t i = a*3+f; const auto value = getState(i);
      row.states[i] = finite(value) ? *value : nan;
      if (finite(value)) {row.state_ok |= uint32_t{1} << i;}
    }
    const double position = row.states[a*3];
    if (!std::isfinite(position) || position < lower_[a] || position > upper_[a]) {row.material_missing |= uint32_t{1} << a;}
    released[a*5] = position;
  }
  const auto accepted = getState(n_*3); row.accepted_ns = steadyNow();
  row.accepted = accepted.value_or(nan);
  row.previous_generation = generation_;
  if (generation_) {
    row.accepted_result = !accepted ? Accepted::kUnavailable : !std::isfinite(*accepted) ? Accepted::kNonfinite :
      (*accepted == static_cast<double>(generation_) ? Accepted::kMatch : Accepted::kMismatch);
    row.advanced = mayAdvance(previous_complete_, generation_, accepted);
    row.held = !row.advanced;
    if (row.held) {++holds_; max_holds_ = std::max(max_holds_, holds_);} else {holds_ = 0;}
  }
  uint64_t candidate = !generation_ ? 1 : generation_ + (row.advanced ? 1 : 0);
  row.generation = candidate;
  const auto state_mask = (uint32_t{1} << (n_*3)) - 1;
  const double dt = period.seconds();
  if (row_count_ >= rows_.size()) {abort(Reason::kCapacity, row);}
  else if (row.state_ok != state_mask) {abort(Reason::kState, row);}
  else if (!std::isfinite(dt) || dt <= 0 || dt > max_period_) {abort(Reason::kPeriod, row);}
  // Normal generation exhaustion does not create an ERROR or a special release number.
  // Leave the last publication's watchdog untouched (contract section 3.1).
  else if (candidate > kMaxGeneration) {
    terminal_ = true; reason_ = Reason::kUpperLimit; row.advanced = false;
    append(row); return Ret::OK;
  }
  for (size_t a = 0; a < n_ && state_ != State::kAborted; ++a) {
    if (std::abs(row.states[a*3]) > state_limit_) {abort(Reason::kState, row);}
  }
  const bool boot = state_ == State::kBoot;
  if (state_ != State::kAborted) {
    if (stop_.load()) {state_ = State::kStopped; reason_ = Reason::kStop; terminal_ = true;}
    else if (!boot) {
      Request request;
      if (state_ == State::kIdle && requests_.tryRead(&request)) {
        run_ = request;
        bool valid = request.axes == n_ && request.nonce && request.id &&
          request.sent_ns > 0 && request.sent_ns <= row.start_ns && request.received_ns <= row.start_ns &&
          request.amplitude == kPi/6. && request.frequency == .2 && request.cycles == 5. &&
          request.ramp_cycles == 1. && request.hold == 1.;
        for (size_t a = 0; a < n_; ++a) {
          valid = valid && std::isfinite(request.kp[a]) && request.kp[a] > 0 && request.kp[a] <= 20. &&
            std::isfinite(request.kd[a]) && request.kd[a] >= 0 && request.kd[a] <= 1. &&
            -request.amplitude >= lower_[a] && request.amplitude <= upper_[a];
        }
        if (!valid) {abort(Reason::kTrajectory, row);}
        else {sine_.emplace(0., run_.amplitude, run_.frequency, run_.cycles, run_.ramp_cycles);
        state_ = State::kArmed; armed_ns_ = row.start_ns;}
      }
      Heartbeat heartbeat;
      if (hearts_.tryRead(&heartbeat) && heartbeat.nonce == run_.nonce && heartbeat.id == run_.id &&
        heartbeat.sequence > heart_.sequence && heartbeat.sent_ns >= heart_.sent_ns && heartbeat.sent_ns <= row.start_ns) {heart_ = heartbeat;}
      if (state_ == State::kArmed) {
        if (heart_.sequence && !leaseExpired(row.start_ns, heart_.sent_ns)) {
          for (size_t a = 0; a < n_; ++a) {
            if (std::abs(row.states[a*3]) > .05) {abort(Reason::kStartRange, row);}
          }
          if (state_ != State::kAborted) {state_ = State::kRunning; elapsed_ = 0.;}
        } else if (row.start_ns - armed_ns_ >= kArmNs) {abort(Reason::kArmTimeout, row);}
      } else if (state_ == State::kRunning || state_ == State::kHolding) {elapsed_ += dt;}
      if (state_ == State::kRunning || state_ == State::kHolding) {
        if (leaseExpired(row.start_ns, heart_.sent_ns)) {abort(Reason::kLease, row);}
        else if (state_ == State::kHolding && finish_.load()) {state_ = State::kReleased; reason_ = Reason::kFinish; terminal_ = true;}
        else if (elapsed_ >= sine_->duration() + run_.hold && run_.release) {state_ = State::kReleased; reason_ = Reason::kRelease; terminal_ = true;}
        else if (elapsed_ >= sine_->duration()) {state_ = State::kHolding;}
      }
    }
  }
  values = released;
  if (state_ == State::kRunning || state_ == State::kHolding) {
    const auto point = sine_->at(elapsed_); const double a = sine_->accelerationAt(elapsed_); row.acceleration = a;
    if (!std::isfinite(point.position) || !std::isfinite(point.velocity) || !std::isfinite(a) ||
      std::abs(point.velocity) > .70 || std::abs(a) > .90) {abort(Reason::kTrajectory, row);}
    for (size_t axis = 0; axis < n_ && state_ != State::kAborted; ++axis) {
      if (point.position < lower_[axis] || point.position > upper_[axis]) {abort(Reason::kTrajectory, row); break;}
      if (run_.kp[axis] * std::abs(point.position - row.states[axis*3]) > max_torque_) {abort(Reason::kTorque, row); break;}
      values[axis*5] = point.position; values[axis*5+1] = state_ == State::kHolding ? 0. : point.velocity;
      values[axis*5+2] = run_.kp[axis]; values[axis*5+3] = run_.kd[axis];
    }
  }
  row.nonce = run_.nonce; row.request_id = run_.id;
  row.request_sent_ns = run_.sent_ns; row.request_received_ns = run_.received_ns;
  row.heartbeat_sent_ns = heart_.sent_ns; row.deadline_ns = heart_.sent_ns ? heart_.sent_ns + kLeaseNs : 0;
  if (state_ != State::kAborted) {
    if (row.material_missing && state_ != State::kRunning && state_ != State::kHolding) {abort(Reason::kState, row);}
    else if (!attempt(values, candidate, row.normal)) {abort(Reason::kWrite, row);}
    else {generation_ = candidate; previous_complete_ = true; if (state_ == State::kBoot) {state_ = State::kIdle;}}
  }
  if (state_ == State::kAborted) {
    ++failures_;
    if (!row.material_missing && candidate <= kMaxGeneration) {
      row.released_attempted = true; attempt(released, candidate, row.released);
    }
    row.error_ns = steadyNow(); append(row); return Ret::ERROR;
  }
  append(row); return Ret::OK;
}
bool NAxisTrajectoryController::saveRecord(const std::string & path)
{
  std::lock_guard<std::mutex> lock(save_mutex_);
  if (!complete_.load(std::memory_order_acquire)) {return false;}
  std::ofstream out(path);
  out << "#schema=n_axis_controller_v1\n#provisional=accepted_hold,api_tries,lease_boundary,capacity\n"
    << "#readback_is_not_hwif_ack=1;error_request=mode_stop_after_return\n"
    << "#profile=center0,amplitude_pi_over_6,frequency0.2,cycles5,ramp1,hold1,effort0\n"
    << "#dropped=" << dropped_ << ";capacity=" << rows_.size() << ";row_bytes=" << sizeof(Row)
    << ";api_max_tries=" << tries_ << '\n'
    << "#configure_in=" << timing_.configure_in << ";configure_out=" << timing_.configure_out
    << ";activate_in=" << timing_.activate_in << ";activate_out=" << timing_.activate_out
    << ";first_update=" << timing_.first_update << '\n';
  out << "N,activation,cycle,nonce,request_id,g,previous_g,accepted,accepted_result,accepted_ns,advanced,held,holds,max_holds,failures,start_ns,end_ns,detected_ns,error_return_ns,heartbeat_sent_ns,lease_deadline_ns,request_sent_ns,request_received_ns,elapsed,acceleration,before,after,reason,state_ok,material_missing,release_attempted";
  for (size_t a = 0; a < n_; ++a) {out << ",joint" << a << ",position" << a << ",velocity" << a << ",effort" << a;}
  for (const auto * kind : {"normal", "release"}) {
    for (const auto * field : {"write_attempted", "write_ok", "read_attempted", "read_ok", "unavailable", "mismatch", "commit_set_ns", "commit_read_ns"}) {out << ',' << kind << '_' << field;}
    for (size_t i = 0; i < 5*n_+1; ++i) {out << ',' << kind << "_value" << i << ',' << kind << "_readback" << i;}
  }
  out << '\n' << std::setprecision(17);
  const auto write = [&](const Row & r) {
    out << static_cast<unsigned>(r.axes) << ',' << r.activation << ',' << r.cycle << ',' << r.nonce << ',' << r.request_id << ',' << r.generation << ',' << r.previous_generation << ',' << r.accepted << ',' << static_cast<unsigned>(r.accepted_result) << ',' << r.accepted_ns << ',' << r.advanced << ',' << r.held << ',' << r.holds << ',' << r.max_holds << ',' << r.failures << ',' << r.start_ns << ',' << r.end_ns << ',' << r.detected_ns << ',' << r.error_ns << ',' << r.heartbeat_sent_ns << ',' << r.deadline_ns << ',' << r.request_sent_ns << ',' << r.request_received_ns << ',' << r.elapsed << ',' << r.acceleration << ',' << static_cast<unsigned>(r.before) << ',' << static_cast<unsigned>(r.after) << ',' << static_cast<unsigned>(r.reason) << ',' << r.state_ok << ',' << r.material_missing << ',' << r.released_attempted;
    for (size_t a = 0; a < n_; ++a) {out << ",joint" << a+1 << ',' << r.states[3*a] << ',' << r.states[3*a+1] << ',' << r.states[3*a+2];}
    for (const auto * attempt : {&r.normal, &r.released}) {
      out << ',' << attempt->write_attempted << ',' << attempt->write_ok << ',' << attempt->read_attempted << ',' << attempt->read_ok << ',' << attempt->unavailable << ',' << attempt->mismatch << ',' << attempt->commit_set_ns << ',' << attempt->commit_read_ns;
      for (size_t i = 0; i < 5*n_+1; ++i) {out << ',' << attempt->values[i] << ',' << attempt->readback[i];}
    }
    out << '\n';
  };
  for (size_t i = 0; i < row_count_; ++i) {write(rows_[i]);}
  if (dropped_) {write(final_row_);}
  out.flush();
  const bool ok = out.good() && !dropped_ && !rows_.empty() && row_count_ != 0;
  if (!ok) {save_failed_ = true; terminal_ = true;}
  return ok;
}
}  // namespace bxi_hardware
PLUGINLIB_EXPORT_CLASS(bxi_hardware::NAxisTrajectoryController, controller_interface::ControllerInterface)
