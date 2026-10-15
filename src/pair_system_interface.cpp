#include "bxi_hardware/bxi_system_interface.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include "rclcpp/rclcpp.hpp"

namespace bxi_hardware
{
namespace
{
constexpr const char * fields[] = {"position", "velocity", "kp", "kd", "effort"};
// Mode-switch exposes names, not owners. This tests only the requested key set.
bool complete(const std::vector<std::string> & request,
  const std::array<std::string, 11> & expected)
{
  if (request.empty()) {return true;}
  if (request.size() != expected.size()) {return false;}
  for (const auto & key : expected) {
    if (std::count(request.begin(), request.end(), key) != 1) {return false;}
  }
  return true;
}
bool number(const std::string & text, double & value)
{
  try {
    size_t end = 0;
    value = std::stod(text, &end);
    return end == text.size() && std::isfinite(value);
  } catch (...) {return false;}
}
}

hardware_interface::CallbackReturn BxiSystemInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return configurePair() ? hardware_interface::CallbackReturn::SUCCESS :
    hardware_interface::CallbackReturn::ERROR;
}

bool BxiSystemInterface::configurePair()
{
  const auto gpio = std::find_if(info_.gpios.begin(), info_.gpios.end(),
    [](const auto & item) {return item.name == "bxi_pair";});
  if (!pair_mode_) {return gpio == info_.gpios.end();}
  if (gpio == info_.gpios.end() || info_.gpios.size() != 1 ||
    gpio->command_interfaces.size() != 1 || gpio->state_interfaces.size() != 1 ||
    gpio->command_interfaces[0].name != "commit_generation" ||
    gpio->state_interfaces[0].name != "accepted_generation" ||
    gpio->command_interfaces[0].data_type != "double" ||
    gpio->state_interfaces[0].data_type != "double" || n_joints_ != 2 ||
    info_.joints[0].name != "joint1" || info_.joints[1].name != "joint2" ||
    active_buses_.size() != 1 || active_buses_[0] != "can0" ||
    joints_[0].can_bus != "can0" || joints_[1].can_bus != "can0" ||
    !joints_[0].enabled || !joints_[1].enabled || joints_[0].can_id != 1 ||
    joints_[1].can_id != 2 || bus_formats_.at("can0") != CanFrameFormat::kFdBrs)
  {return false;}
  const auto bus = bus_transport_.find("can0");
  if (bus == bus_transport_.end() || !bus->second.separate || !bus->second.broadcast ||
    !bus->second.write_triggered || !bus->second.direct_feedback || can_hz_ != 1000 ||
    bus->second.stream_command_max_age_ms != 5.0 || bus->second.stream_feedback_max_age_ms != 3.0)
  {return false;}
  PairCommit::Bounds bounds;
  for (size_t axis = 0; axis < 2; ++axis) {
    const auto & joint = info_.joints[axis];
    const auto & runtime = joints_[axis];
    if (joint.command_interfaces.size() != 5 || !loadSpec(runtime.motor_type)) {return false;}
    const auto & spec = specs_.at(runtime.motor_type);
    const auto * driver = dynamic_cast<const BxiActuatorDriver *>(runtime.actuator.get());
    const double offset = driver ? driver->effectiveAngleOffset() : runtime.offset_angle;
    if (!std::isfinite(offset)) {return false;}
    const double low_pos = std::min((spec.p_min - offset) * runtime.motor_dir,
      (spec.p_max - offset) * runtime.motor_dir);
    const double high_pos = std::max((spec.p_min - offset) * runtime.motor_dir,
      (spec.p_max - offset) * runtime.motor_dir);
    const double low_vel = std::min(spec.v_min * runtime.motor_dir, spec.v_max * runtime.motor_dir);
    const double high_vel = std::max(spec.v_min * runtime.motor_dir, spec.v_max * runtime.motor_dir);
    const double low_eff = std::min(spec.t_min * runtime.motor_dir, spec.t_max * runtime.motor_dir);
    const double high_eff = std::max(spec.t_min * runtime.motor_dir, spec.t_max * runtime.motor_dir);
    const double lower[] = {std::max(runtime.lower, low_pos), low_vel, 0, 0, low_eff};
    const double upper[] = {std::min(runtime.upper, high_pos), high_vel, spec.kp_max, spec.kd_max, high_eff};
    for (size_t field = 0; field < 5; ++field) {
      const auto found = std::find_if(joint.command_interfaces.begin(), joint.command_interfaces.end(),
        [&](const auto & item) {return item.name == fields[field];});
      if (found == joint.command_interfaces.end() || found->data_type != "double") {return false;}
      const size_t i = axis * 5 + field;
      pair_keys_[i] = joint.name + "/" + fields[field];
      // Explicit G profile, represented by standard per-axis command-interface min/max.
      if (!number(found->min, bounds.lower[i]) || !number(found->max, bounds.upper[i]) ||
        bounds.lower[i] > bounds.upper[i] || bounds.lower[i] < lower[field] ||
        bounds.upper[i] > upper[field] ||
        (field != 0 && (bounds.lower[i] > 0 || bounds.upper[i] < 0)))
      {return false;}
    }
  }
  pair_keys_[10] = "bxi_pair/commit_generation";
  pair_commit_.bounds(bounds);
  return true;
}

hardware_interface::return_type BxiSystemInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  if (!pair_mode_) {return hardware_interface::return_type::OK;}
  if (!complete(start, pair_keys_) || !complete(stop, pair_keys_) ||
    (!start.empty() && (pair_terminal_ || pair_claimed_ || !stop.empty())) ||
    (!stop.empty() && !pair_claimed_))
  {return hardware_interface::return_type::ERROR;}
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type BxiSystemInterface::perform_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  const auto result = prepare_command_mode_switch(start, stop);
  if (!pair_mode_ || result != hardware_interface::return_type::OK) {return result;}
  if (!stop.empty()) {pair_claimed_ = false; pair_terminal_ = true; pair_commit_.latch();}
  if (!start.empty()) {pair_claimed_ = true;}
  return result;
}

hardware_interface::return_type BxiSystemInterface::writePair(int64_t now)
{
  ++control_sequence_;
  PairRecord row;
  row.cycle = control_sequence_; row.received = pair_generation_;
  row.previous = pair_commit_.last(); row.inspected_ns = now;
  for (size_t axis = 0; axis < 2; ++axis) {
    const size_t i = axis * 5;
    row.values[i] = cmd_pos_[axis]; row.values[i + 1] = cmd_vel_[axis];
    row.values[i + 2] = cmd_kp_[axis]; row.values[i + 3] = cmd_kd_[axis];
    row.values[i + 4] = cmd_eff_[axis];
  }
  // Stage is untrusted even when unclaimed; it never supplies bootstrap values.
  row.reason = pair_claimed_ ? pair_commit_.inspect(pair_generation_, row.values, now) :
    PairCommit::Reason::kNotClaimed;
  if (row.reason == PairCommit::Reason::kAccepted) {
    std::array<PublishedCommand, 2> packets;
    for (size_t axis = 0; axis < 2; ++axis) {
      const size_t i = axis * 5;
      packets[axis].command = {row.values[i], row.values[i + 1], row.values[i + 2],
        row.values[i + 3], row.values[i + 4]};
      packets[axis].sequence = static_cast<uint64_t>(pair_generation_);
    }
    row.published_ns = pairSteadyNow();
    for (auto & packet : packets) {packet.published_ns = row.published_ns;}
    // Recheck the fixed initial deadline at batch completion, not only at write entry.
    row.reason = pair_commit_.inspect(pair_generation_, row.values, row.published_ns);
    if (row.reason == PairCommit::Reason::kAccepted) {
      row.accepted_ns = row.published_ns;
      pair_commit_.accept(pair_generation_);
      pair_accepted_ = static_cast<double>(pair_commit_.last());
      row.result = publishPairBatch(packets.data());
      if (row.result != CommandPublishResult::kSnapshotPublished) {
        pair_commit_.latch(); row.reason = PairCommit::Reason::kPublishFailed;
      }
    } else {row.published_ns = 0;}
  }
  row.accepted = pair_commit_.last();
  if (pair_record_count_ < pair_records_.size()) {pair_records_[pair_record_count_] = row;}
  ++pair_record_count_;
  const auto end = std::chrono::steady_clock::now().time_since_epoch().count();
  for (auto & control : control_streams_) {
    if (control.record_index < control.capacity) {
      for (size_t axis = 0; axis < control.indices.size(); ++axis) {
        auto & record = control.records[control.record_index * control.indices.size() + axis];
        record.setCycle(control_sequence_); record.write_start = now; record.write_end = end;
        record.published = row.result == CommandPublishResult::kNotPublished ? 0 : row.published_ns;
      }
    }
    control.record_index = SIZE_MAX;
  }
  // b-1 latches and ceases normal publication. ERROR/safety ordering belongs to b-2.
  return hardware_interface::return_type::OK;
}

void BxiSystemInterface::savePairRecords()
{
  if (!pair_mode_ || pair_records_saved_) {return;}
  std::ofstream out(worker_timing_dir_ + "/pair_commits.csv");
  out << "#dropped=" << (pair_record_count_ > pair_records_.size() ?
    pair_record_count_ - pair_records_.size() : 0) << '\n';
  out << "cycle,received_g,previous_L,accepted_g,reason,inspected_ns,accepted_ns,published_ns,publish_result";
  for (size_t i = 0; i < 10; ++i) {out << ",value" << i;}
  out << '\n' << std::setprecision(17);
  for (size_t i = 0; i < std::min(pair_record_count_, pair_records_.size()); ++i) {
    const auto & row = pair_records_[i];
    out << row.cycle << ',' << row.received << ',' << row.previous << ',' << row.accepted << ',' <<
      static_cast<unsigned>(row.reason) << ',' << row.inspected_ns << ',' << row.accepted_ns << ',' << row.published_ns << ',' <<
      static_cast<unsigned>(row.result);
    for (const auto value : row.values) {out << ',' << value;}
    out << '\n';
  }
  out.flush();
  if (out) {pair_records_saved_ = true;}
  else {RCLCPP_ERROR(rclcpp::get_logger("BxiSystemInterface"), "pair commit record save failed");}
}
}  // namespace bxi_hardware
