#include "bxi_hardware/bxi_pair_adapter.hpp"
#include "bxi_hardware/pair_commit.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <set>
#include "pluginlib/class_list_macros.hpp"
namespace bxi_hardware
{
namespace
{
constexpr double nan = std::numeric_limits<double>::quiet_NaN();
int64_t steadyNow()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

}
BxiPairAdapter::CB BxiPairAdapter::on_init()
{
  auto_declare<std::vector<std::string>>("hardware_groups", {});
  auto_declare<double>("record_seconds", 120.);
  auto_declare<double>("max_period_sec", .005);
  auto_declare<int>("api_max_tries", 1);
  auto_declare<std::string>("output_dir", "");
  return CB::SUCCESS;
}

BxiPairAdapter::CB BxiPairAdapter::on_configure(const rclcpp_lifecycle::State &)
{
  if (activated_once_) return CB::ERROR;
  auto node = get_node();
  try {
    if (!node->has_parameter("velocity_feedforward"))
      node->declare_parameter("velocity_feedforward", rclcpp::ParameterType::PARAMETER_BOOL);
    const auto velocity_parameter = node->get_parameter("velocity_feedforward");
    if (velocity_parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) return CB::ERROR;
    velocity_feedforward_ = velocity_parameter.as_bool();
  } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
    return CB::ERROR;
  }
  const auto names = node->get_parameter("hardware_groups").as_string_array();
  if (names.empty() || names.size() > 4) return CB::ERROR;
  std::set<std::string> hardware, gpios, joints;
  joints_.clear();
  command_keys_.clear();
  state_keys_.clear();
  group_count_ = names.size();
  const char * fields[] = {"position", "velocity", "kp", "kd", "effort"};
  const char * states[] = {"position", "velocity", "effort", "temperature"};
  for (size_t h = 0; h < names.size(); ++h) {
    const auto prefix = "groups." + names[h] + ".";
    const auto joint_names = auto_declare<std::vector<std::string>>(prefix + "joints", {});
    auto & hardware_group = groups_[h];
    hardware_group.name = auto_declare<std::string>(prefix + "hardware_name", names[h]);
    hardware_group.gpio = auto_declare<std::string>(prefix + "gpio", "");
    if (joint_names.size() < 2 || joint_names.size() > 3 || hardware_group.gpio.empty() ||
      !hardware.insert(hardware_group.name).second || !gpios.insert(hardware_group.gpio).second)
      return CB::ERROR;
    hardware_group.first = joints_.size();
    hardware_group.axes = joint_names.size();
    hardware_group.command = command_keys_.size();
    hardware_group.state = state_keys_.size();
    for (const auto & joint_name: joint_names) {
      if (joint_name.empty() || !joints.insert(joint_name).second) return CB::ERROR;
      joints_.push_back(joint_name);
      for (auto f: fields) command_keys_.push_back(joint_name + "/" + f);
      for (auto f: states) state_keys_.push_back(joint_name + "/" + f);
    }
    command_keys_.push_back(hardware_group.gpio + "/commit_generation");
    state_keys_.push_back(hardware_group.gpio + "/accepted_generation");
  }
  for (size_t a = 0; a < joints_.size(); ++a) {
    const auto parameter_prefix = "joint_settings." + joints_[a] + ".";
    auto read_double = [&](const char * name){
        return auto_declare<double>(parameter_prefix + name, nan);
      };
    auto & joint_limits = limits_[a];
    joint_limits = {read_double("position_min"), read_double("position_max"),
      read_double("velocity_limit"), read_double("kp"), read_double("kd"), read_double("kp_max"),
      read_double("kd_max"), read_double("max_target_delta_per_cycle"), read_double("torque_limit"),
      read_double("deviation_limit_rad"), read_double("temperature_limit_c"), 0};
    const auto deviation_cycles = auto_declare<int>(parameter_prefix + "deviation_cycles", 0);
    const double values[] = {joint_limits.position_min, joint_limits.position_max,
      joint_limits.velocity, joint_limits.kp, joint_limits.kd, joint_limits.kp_max,
      joint_limits.kd_max, joint_limits.delta, joint_limits.torque, joint_limits.deviation,
      joint_limits.temperature};
    if (!std::all_of(std::begin(values), std::end(values), [](double value){
        return std::isfinite(value);
      })
      ||
      joint_limits.position_min >= joint_limits.position_max || joint_limits.velocity <= 0 ||
      joint_limits.kp<0 || joint_limits.kd<0 || joint_limits.kp> joint_limits.kp_max ||
      joint_limits.kd> joint_limits.kd_max || joint_limits.delta <= 0 || joint_limits.torque <= 0 ||
      joint_limits.deviation <= 0 || deviation_cycles <= 0)
      return CB::ERROR;
    joint_limits.cycles = static_cast<unsigned>(deviation_cycles);
  }
  max_period_ = node->get_parameter("max_period_sec").as_double();
  const double seconds = node->get_parameter("record_seconds").as_double();
  directory_ = node->get_parameter("output_dir").as_string();
  if (node->get_parameter("api_max_tries").as_int() != 1 || !std::isfinite(seconds) ||
    seconds < 0 ||
    seconds > 600 || !std::isfinite(max_period_) || max_period_ <= 0 || max_period_ > .005)
    return CB::ERROR;
  reference_interfaces_.assign(joints_.size() * (velocity_feedforward_ ? 2 : 1), nan);
  rows_.assign(static_cast<size_t>(std::ceil(seconds * 1000.)), Row{});
  return CB::SUCCESS;
}

controller_interface::InterfaceConfiguration BxiPairAdapter::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, command_keys_};
}

controller_interface::InterfaceConfiguration BxiPairAdapter::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, state_keys_};
}

std::vector<hardware_interface::CommandInterface> BxiPairAdapter::on_export_reference_interfaces()
{
  std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), nan);
  std::vector<hardware_interface::CommandInterface> refs;
  for (size_t a = 0; a < joints_.size();
    ++a)
    refs.emplace_back(get_node()->get_name(), joints_[a] + "/position", &reference_interfaces_[a]);
  if (velocity_feedforward_)
    for (size_t a = 0; a < joints_.size(); ++a)
      refs.emplace_back(get_node()->get_name(), joints_[a] + "/velocity",
        &reference_interfaces_[joints_.size() + a]);
  return refs;
}

bool BxiPairAdapter::bind()
{
  if (command_interfaces_.size() != command_keys_.size() ||
    state_interfaces_.size() != state_keys_.size())
    return false;
  for (size_t k = 0; k < command_keys_.size(); ++k) {
    size_t count = 0;
    for (size_t i = 0; i < command_interfaces_.size();
      ++i)
      if (command_interfaces_[i].get_name() == command_keys_[k]) {
        if (command_interfaces_[i].get_data_type().to_string() != "double") return false;
        command_index_[k] = i;
        ++count;
      }
    if (count != 1) return false;
  }
  for (size_t k = 0; k < state_keys_.size(); ++k) {
    size_t count = 0;
    for (size_t i = 0; i < state_interfaces_.size();
      ++i)
      if (state_interfaces_[i].get_name() == state_keys_[k]) {
        if (state_interfaces_[i].get_data_type().to_string() != "double") return false;
        state_index_[k] = i;
        ++count;
      }
    if (count != 1) return false;
  }
  return true;
}

BxiPairAdapter::CB BxiPairAdapter::on_activate(const rclcpp_lifecycle::State &)
{
  if (activated_once_ || group_count_ == 0) return CB::ERROR;
  activated_once_ = true;
  if (!bind()) return CB::ERROR;
  try {
    for (size_t h = 0; h < group_count_; ++h) {
      auto & hardware_group = groups_[h];
      const auto accepted = getState(hardware_group.state + 4 * hardware_group.axes);
      if (!accepted || *accepted != 0.) return CB::ERROR;
      hardware_group.generation = 0;
      for (size_t a = 0; a < hardware_group.axes; ++a) {
        for (size_t f = 0; f < 4; ++f) {
          const auto value = getState(hardware_group.state + 4 * a + f);
          if (!value || !std::isfinite(*value)) return CB::ERROR;
          const auto axis = hardware_group.first + a;
          const auto & joint_limits = limits_[axis];
          if (f == 0) {
            if (*value < joint_limits.position_min || *value > joint_limits.position_max) {
              RCLCPP_ERROR(get_node()->get_logger(),
                  "startup rejected joint=%s angle=%.17g min=%.17g max=%.17g",
                  joints_[axis].c_str(), *value, joint_limits.position_min,
                  joint_limits.position_max);
              return CB::ERROR;
            }
            boot_[axis] = previous_[axis] = *value;
          }
          if (f == 3 && *value >= joint_limits.temperature) return CB::ERROR;
        }
      }
    }
  } catch (...) {
    return CB::ERROR;
  }
  std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), nan);
  seen_reference_.fill(false);
  seen_velocity_.fill(false);
  deviation_count_.fill(0);
  count_ = 0;
  cycle_ = 0;
  dropped_ = 0;
  ++activation_;
  active_ = true;
  terminal_ = false;
  saved_ = false;
  return CB::SUCCESS;
}

bool BxiPairAdapter::writeValue(size_t k, double value, Attempt & out)
{
  const uint64_t bit = uint64_t{1} << k;
  out.values[k] = value;
  out.write_attempted |= bit;
  if (!setCommand(k, value)) return false;
  out.write_ok |= bit;
  out.read_attempted |= bit;
  const auto read = getCommand(k);
  if (!read || !std::isfinite(*read)) {
    out.unavailable |= bit;
    return false;
  }
  out.readback[k] = *read;
  if (*read != value) {
    out.mismatch |= bit;
    return false;
  }
  out.read_ok |= bit;
  return true;
}

bool BxiPairAdapter::writeBatch(Row & row, bool release)
{
  auto & out = release ? row.release : row.normal;
  std::array<bool, 4> good{};
  bool all = true;
  for (size_t h = 0; h < group_count_; ++h) {
    const auto & hardware_group = groups_[h];
    good[h] = true;
    if (release && ((row.commit_touched | row.release_missing) & (1U << h))) {
      good[h] = false;
      row.release_missing |= 1U << h;
      continue;
    }
    for (size_t a = 0; a < hardware_group.axes; ++a) {
      const auto axis = hardware_group.first + a;
      const double position = release ? row.states[hardware_group.state + 4 * a] : row.target[axis];
      if (!std::isfinite(position) || position < limits_[axis].position_min ||
        position > limits_[axis].position_max)
      {
        good[h] = false;
      }
    }
    if (row.candidate_g[h] == 0 || row.candidate_g[h] > PairCommit::kMaxGeneration) good[h] = false;
    if (!good[h]) {
      if (release) row.release_missing |= 1U << h;
      all = false;
      continue;
    }
    for (size_t a = 0; a < hardware_group.axes; ++a) {
      const auto axis = hardware_group.first + a;
      const bool zero = release || hardware_group.generation == 0;
      const double values[] = {release ? row.states[hardware_group.state + 4 *
          a] : row.target[axis], zero ? 0. : row.target_velocity[axis], zero ? 0. : limits_[axis].kp, zero ? 0. : limits_[axis].kd,
        0.};
      for (size_t f = 0; f < 5;
        ++f)
        if (!writeValue(hardware_group.command + 5 * a + f, values[f], out)) good[h] = false;



    }
    all = all && good[h];
  }
  if (!release && !all) return false;
  for (size_t h = 0; h < group_count_; ++h) {
    if (!good[h]) continue;
    const auto & hardware_group = groups_[h];
    if (!release) row.commit_touched |= 1U << h;
    out.commit_set[h] = steadyNow();
    if (!writeValue(hardware_group.command + 5 * hardware_group.axes,
        static_cast<double>(row.candidate_g[h]), out))
      all = false;
    out.commit_read[h] = steadyNow();
  }
  return all;
}

void BxiPairAdapter::record(Row & row)
{
  row.end = steadyNow();
  final_ = row;
  if (count_ < rows_.size()) rows_[count_++] = row;
  else ++dropped_;
}

BxiPairAdapter::Ret BxiPairAdapter::update_and_write_commands(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  if (terminal_ || !active_) return final_.reason == 11 ? Ret::OK : Ret::ERROR;
  Row row;
  row.activation = activation_;
  row.cycle = ++cycle_;
  row.start = steadyNow();
  row.dt = period.seconds();
  for (size_t k = 0; k < state_keys_.size(); ++k) {
    const auto value = getState(k);
    row.states[k] = value.value_or(nan);
    if (value && std::isfinite(*value)) row.state_ok |= uint64_t{1}
      << k;
  }
  if (row.state_ok != ((uint64_t{1} << state_keys_.size()) - 1)) row.reason = 1;
  if (!std::isfinite(row.dt) || row.dt <= 0 || row.dt > max_period_) row.reason = 2;
  bool boot = false;
  for (size_t h = 0; h < group_count_; ++h) {
    const auto & hardware_group = groups_[h];
    row.previous_g[h] = hardware_group.generation;
    row.candidate_g[h] = hardware_group.generation + 1;
    boot = boot || hardware_group.generation == 0;
    const auto accepted = row.states[hardware_group.state + 4 * hardware_group.axes];
    if (!std::isfinite(accepted) || accepted != static_cast<double>(hardware_group.generation)) {
      row.reason = 4;
      row.release_missing |= 1U << h;
    }
    if (row.candidate_g[h] > PairCommit::kMaxGeneration) row.reason = 11;
    for (size_t a = 0; a < hardware_group.axes; ++a) {
      const auto axis = hardware_group.first + a;
      const auto & joint_limits = limits_[axis];
      const double position = row.states[hardware_group.state + 4 * a],
        temp = row.states[hardware_group.state + 4 * a + 3];
      row.raw[axis] = reference_interfaces_[axis];
      row.raw_velocity[axis] = velocity_feedforward_ ? reference_interfaces_[joints_.size() + axis] : nan;
      double target_velocity = velocity_feedforward_ ? row.raw_velocity[axis] : 0.;
      if (std::isnan(target_velocity) && !seen_velocity_[axis]) target_velocity = 0.;
      else if (std::isfinite(target_velocity)) seen_velocity_[axis] = true;
      if (!std::isfinite(target_velocity)) row.reason = 5;
      if (!boot && std::isfinite(target_velocity) && std::abs(target_velocity) > joint_limits.velocity)
        row.reason = 6;
      row.target_velocity[axis] = boot ? 0. : target_velocity;
      row.previous[axis] = previous_[axis];
      double target = reference_interfaces_[axis];
      if (std::isnan(target) && !seen_reference_[axis]) target = boot_[axis];
      else if (std::isfinite(target)) seen_reference_[axis] = true;
      if ((!std::isfinite(reference_interfaces_[axis]) &&
        !std::isnan(reference_interfaces_[axis])) ||
        (std::isnan(reference_interfaces_[axis]) && seen_reference_[axis]))
        row.reason = 5;
      if (boot) target = position;
      row.target[axis] = target;
      if (std::isfinite(reference_interfaces_[axis])) row.reference_ok |= uint16_t{1}
        << axis;
      row.delta[axis] = target - previous_[axis];
      row.velocity[axis] = std::abs(row.delta[axis]) / row.dt;
      row.torque[axis] = std::abs(joint_limits.kp * (target - position));
      row.torque_with_kd[axis] = std::abs((boot ? 0. : joint_limits.kp) * (target - position) +
        (boot ? 0. : joint_limits.kd) * (row.target_velocity[axis] - row.states[hardware_group.state + 4 * a + 1]));
      if (!std::isfinite(target) || target < joint_limits.position_min ||
        target > joint_limits.position_max || !std::isfinite(position) ||
        position < joint_limits.position_min || position > joint_limits.position_max)
        row.reason = 5;
      if (!boot &&
        (std::abs(row.delta[axis]) > joint_limits.delta ||
        row.velocity[axis] > joint_limits.velocity))
        row.reason = 6;
      if (!boot && row.torque[axis] > joint_limits.torque) row.reason = 7;
      if (std::abs(target - position) > joint_limits.deviation) {
        if (deviation_count_[axis] < joint_limits.cycles) ++deviation_count_[axis];
      } else deviation_count_[axis] = 0;
      row.deviation_count[axis] = deviation_count_[axis];
      if (deviation_count_[axis] >= joint_limits.cycles) row.reason = 8;
      if (temp >= joint_limits.temperature) row.reason = 9;
    }
  }
  if (row.reason == 11) {
    terminal_ = true;
    record(row);
    return Ret::OK;
  }
  if (!row.reason && !writeBatch(row, false)) row.reason = 10;
  if (row.reason) {
    terminal_ = true;
    row.target_velocity.fill(0.);
    row.torque_with_kd.fill(0.);
    writeBatch(row, true);
    record(row);
    return Ret::ERROR;
  }
  for (size_t h = 0; h < group_count_; ++h) groups_[h].generation = row.candidate_g[h];
  for (size_t a = 0; a < joints_.size(); ++a) previous_[a] = row.target[a];
  record(row);
  return Ret::OK;
}

BxiPairAdapter::CB BxiPairAdapter::on_deactivate(const rclcpp_lifecycle::State &)
{
  terminal_ = true;
  active_ = false;
  return CB::SUCCESS;
}

BxiPairAdapter::CB BxiPairAdapter::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (active_) return CB::ERROR;
  save();
  return CB::SUCCESS;
}

BxiPairAdapter::~BxiPairAdapter()
{
  save();
}

void BxiPairAdapter::save() noexcept
{
  if (saved_ || cycle_ == 0 || directory_.empty()) return;
  try {
    std::filesystem::create_directories(directory_);
    std::ofstream cycles(directory_ + "/cycles.csv"), joints(directory_ + "/joint_cycles.csv"),
    groups(directory_ + "/group_cycles.csv"), commands(directory_ + "/command_cycles.csv"),
    manifest(directory_ + "/manifest.csv");
    for (auto * file: {&cycles, &joints, &groups, &commands, &manifest}) {
      file->exceptions(std::ios::failbit | std::ios::badbit);
      *file << std::setprecision(17);
    }
    cycles <<
      "schema_version,activation,cycle,start_ns,end_ns,period,reason,state_ok,reference_ok,dropped\n";
    joints <<
      "cycle,axis,position,velocity,effort,temperature,reference_raw,target,previous,delta,delta_velocity,torque_proxy,deviation_count,reference_velocity_raw,target_velocity,torque_proxy_with_kd\n";
    groups <<
      "cycle,group,previous_g,candidate_g,accepted,commit_touched,release_missing,normal_set_ns,normal_read_ns,release_set_ns,release_read_ns\n";
    commands <<
      "cycle,attempt,key,value,readback,write_attempted,write_ok,read_attempted,read_ok,unavailable,mismatch\n";
    manifest << "kind,index,name,detail\n";
    manifest << "parameter,0,velocity_feedforward," << (velocity_feedforward_ ? "true" : "false") << '\n';
    for (size_t a = 0; a < joints_.size(); ++a) {
      const auto & joint_limits = limits_[a];
      manifest << "joint," << a << ',' << joints_[a] << ",bounds=" << joint_limits.position_min <<
        ':' << joint_limits.position_max << ";kp=" << joint_limits.kp << ";kd=" <<
        joint_limits.kd << ";velocity=" << joint_limits.velocity << ";delta=" <<
        joint_limits.delta << ";torque=" << joint_limits.torque << ";deviation=" <<
        joint_limits.deviation << ";cycles=" << joint_limits.cycles << ";temperature=" <<
        joint_limits.temperature << '\n';
    }
    for (size_t h = 0; h < group_count_;
      ++h)
      manifest << "hardware," << h << ',' << groups_[h].name << ',' << groups_[h].gpio << '\n';
    manifest << "row_bytes,0,Row," << sizeof(Row) << "\ncapacity,0,rows," << rows_.size() << '\n';
    // Reason 3 is retained for compatibility but is no longer emitted.
    manifest <<
      "reason,0,none,OK\nreason,1,state,unavailable\nreason,2,period,invalid\nreason,3,capacity,full\nreason,4,accepted,mismatch\nreason,5,position_or_reference,invalid\nreason,6,delta_or_velocity,exceeded\nreason,7,torque,exceeded\nreason,8,deviation,exceeded\nreason,9,temperature,exceeded\nreason,10,write_readback,failed\nreason,11,generation,exhausted\n";
    auto emit = [&](const Row & recorded_row){
        cycles << 1 << ',' << recorded_row.activation << ',' << recorded_row.cycle << ',' <<
          recorded_row.start << ',' << recorded_row.end << ',' << recorded_row.dt << ',' <<
          recorded_row.reason << ',' << recorded_row.state_ok << ',' << recorded_row.reference_ok <<
          ',' << dropped_ << '\n';
        for (size_t h = 0; h < group_count_; ++h) {
          const auto & hardware_group = groups_[h];
          groups << recorded_row.cycle << ',' << h << ',' << recorded_row.previous_g[h] << ',' <<
            recorded_row.candidate_g[h] << ',' <<
            recorded_row.states[hardware_group.state + 4 * hardware_group.axes] << ',' <<
            unsigned(recorded_row.commit_touched) << ',' <<
            unsigned(recorded_row.release_missing) << ',' << recorded_row.normal.commit_set[h] <<
            ',' << recorded_row.normal.commit_read[h] << ',' <<
            recorded_row.release.commit_set[h] << ',' << recorded_row.release.commit_read[h] <<
            '\n';
          for (size_t a = 0; a < hardware_group.axes; ++a) {
            const auto axis = hardware_group.first + a;
            joints << recorded_row.cycle << ',' << axis;
            for (size_t f = 0; f < 4;
              ++f)
              joints << ',' << recorded_row.states[hardware_group.state + 4 * a + f];
            joints << ',' << recorded_row.raw[axis] << ',' << recorded_row.target[axis] << ',' <<
              recorded_row.previous[axis] << ',' << recorded_row.delta[axis] << ',' <<
              recorded_row.velocity[axis] << ',' << recorded_row.torque[axis] << ',' <<
              recorded_row.deviation_count[axis] << ',' << recorded_row.raw_velocity[axis] << ',' <<
              recorded_row.target_velocity[axis] << ',' << recorded_row.torque_with_kd[axis] << '\n';
          }
        }
        for (int a = 0; a < 2; ++a) {
          const auto & attempt = a ? recorded_row.release : recorded_row.normal;
          for (size_t k = 0; k < command_keys_.size(); ++k) {
            const auto bit = uint64_t{1} << k;
            commands << recorded_row.cycle << ',' << (a ? "release" : "normal") << ',' <<
              command_keys_[k] << ',' << attempt.values[k] << ',' << attempt.readback[k] << ',' <<
              bool(attempt.write_attempted & bit) << ',' << bool(attempt.write_ok & bit) << ',' <<
              bool(attempt.read_attempted & bit) << ',' << bool(attempt.read_ok & bit) << ',' <<
              bool(attempt.unavailable & bit) << ',' << bool(attempt.mismatch & bit) << '\n';
          }
        }
      };
    for (size_t i = 0; i < count_; ++i) emit(rows_[i]);
    if (!count_ || rows_[count_ - 1].cycle != final_.cycle) emit(final_);
    for (auto * file: {&cycles, &joints, &groups, &commands, &manifest}) file->close();
    saved_ = true;
  } catch (const std::exception & e) {
    std::fprintf(stderr, "BxiPairAdapter CSV save failed: %s\n", e.what());
  }
}

}
PLUGINLIB_EXPORT_CLASS(bxi_hardware::BxiPairAdapter,
  controller_interface::ChainableControllerInterface)
