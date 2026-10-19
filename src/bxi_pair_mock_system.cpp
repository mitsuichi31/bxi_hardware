#include "bxi_hardware/bxi_pair_mock_system.hpp"
#include "bxi_hardware/pair_commit.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
namespace bxi_hardware
{
BxiPairMockSystem::CB BxiPairMockSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CB::SUCCESS) return CB::ERROR;
  axes_ = info_.joints.size();
  if (axes_ < 2 || axes_ > 3 || info_.gpios.size() != 1) return CB::ERROR;
  const auto & gpio = info_.gpios[0];
  gpio_ = gpio.name;
  if (gpio.command_interfaces.size() != 1 || gpio.state_interfaces.size() != 1 ||
    gpio.command_interfaces[0].name != "commit_generation" ||
    gpio.state_interfaces[0].name != "accepted_generation")
    return CB::ERROR;
  const char * fields[] = {"position", "velocity", "kp", "kd", "effort"};
  const char * state_fields[] = {"position", "velocity", "effort", "temperature"};
  std::set<std::string> names;
  commands_.fill(std::numeric_limits<double>::quiet_NaN());
  try {
    for (size_t a = 0; a < axes_; ++a) {
      const auto & joint = info_.joints[a];
      if (!names.insert(joint.name).second || joint.command_interfaces.size() != 5 ||
        joint.state_interfaces.size() != 4)
        return CB::ERROR;
      for (size_t f = 0; f < 5; ++f) {
        const auto interface_definition = std::find_if(joint.command_interfaces.begin(),
            joint.command_interfaces.end(), [&](const auto & i){return i.name == fields[f];});
        if (interface_definition == joint.command_interfaces.end() ||
          interface_definition->data_type != "double")
          return CB::ERROR;
        const auto k = a * 5 + f;
        lower_[k] = std::stod(interface_definition->min);
        upper_[k] = std::stod(interface_definition->max);
        if (!std::isfinite(lower_[k]) || !std::isfinite(upper_[k]) || lower_[k] > upper_[k] ||
          (f && (lower_[k] > 0 || upper_[k] < 0)))
          return CB::ERROR;
        keys_.push_back(joint.name + "/" + fields[f]);
      }
      for (size_t f = 0; f < 4; ++f) {
        const auto interface_definition = std::find_if(joint.state_interfaces.begin(),
            joint.state_interfaces.end(), [&](const auto & i){return i.name == state_fields[f];});
        if (interface_definition == joint.state_interfaces.end() ||
          interface_definition->data_type != "double")
          return CB::ERROR;
        states_[a * 4 + f] = interface_definition->initial_value.empty() ? (f ==
          3 ? 25. : 0.) : std::stod(interface_definition->initial_value);
      }
    }
    keys_.push_back(gpio_ + "/commit_generation");
    auto get = [&](const char * key){
        auto i = info_.hardware_parameters.find(key);
        return i == info_.hardware_parameters.end() ? std::string() : i->second;
      };
    directory_ = get("output_dir");
    for (auto entry: {std::pair<const char *, std::atomic<uint64_t> *>{"read_error_at",
          &read_error_at}, {"write_error_at", &write_error_at}, {"reject_at", &reject_at}})
    {
      const auto parameter_value = get(entry.first);
      if (!parameter_value.empty()) entry.second->store(std::stoull(parameter_value));
    }
    records_.resize(120000);
  } catch (...) {
    return CB::ERROR;
  }
  return CB::SUCCESS;
}

BxiPairMockSystem::CB BxiPairMockSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (accepted_ != 0.) return CB::ERROR;
  for (size_t a = 0; a < axes_; ++a) {
    const auto position = states_[4 * a];
    if (!std::isfinite(position) || position < lower_[5 * a] || position > upper_[5 * a]) {
      RCLCPP_ERROR(rclcpp::get_logger("BxiPairMockSystem"),
          "TEST ONLY startup rejected joint=%s angle=%.17g min=%.17g max=%.17g",
          info_.joints[a].name.c_str(), position, lower_[5 * a], upper_[5 * a]);
      return CB::ERROR;
    }
  }
  return CB::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> BxiPairMockSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  const char * names[] = {"position", "velocity", "kp", "kd", "effort"};
  for (size_t a = 0; a < axes_;
    ++a)
    for (size_t f = 0; f < 5;
      ++f)
      out.emplace_back(info_.joints[a].name, names[f], &commands_[5 * a + f]);



  out.emplace_back(gpio_, "commit_generation", &generation_);
  return out;
}

std::vector<hardware_interface::StateInterface> BxiPairMockSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  const char * names[] = {"position", "velocity", "effort", "temperature"};
  for (size_t a = 0; a < axes_;
    ++a)
    for (size_t f = 0; f < 4;
      ++f)
      out.emplace_back(info_.joints[a].name, names[f], &states_[4 * a + f]);



  out.emplace_back(gpio_, "accepted_generation", &accepted_);
  return out;
}

BxiPairMockSystem::Ret BxiPairMockSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  auto complete = [&](const auto & request){
      return request.empty() || (request.size() == keys_.size() && std::all_of(keys_.begin(),
        keys_.end(), [&](const auto & k){
               return std::count(request.begin(), request.end(), k) == 1;
        }));
    };
  return complete(start) && complete(stop) &&
         (start.empty() || stop.empty()) ? Ret::OK : Ret::ERROR;
}

BxiPairMockSystem::Ret BxiPairMockSystem::perform_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  const auto result = prepare_command_mode_switch(start, stop);
  if (result != Ret::OK) return result;
  if (!start.empty()) claimed_ = true;
  if (!stop.empty()) claimed_ = false;
  return Ret::OK;
}

BxiPairMockSystem::Ret BxiPairMockSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  ++reads_;
  return read_error_at.load() && reads_ >= read_error_at.load() ? Ret::ERROR : Ret::OK;
}

BxiPairMockSystem::Ret BxiPairMockSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  Row row;
  row.cycle = ++writes_;
  row.received = generation_;
  row.accepted = accepted_;
  row.values = commands_;
  auto finish = [&](int result, Ret ret){
      row.result = result;
      row.accepted = accepted_;
      if (count_ < records_.size()) records_[count_++] = row;
      return ret;
    };
  if (write_error_at.load() && writes_ >= write_error_at.load()) return finish(3, Ret::ERROR);
  if (!claimed_ || generation_ == accepted_) return finish(0, Ret::OK);
  if (reject_at.load() && writes_ >= reject_at.load()) return finish(2, Ret::OK);
  if (!std::isfinite(generation_) || generation_ < 1 ||
    generation_ > static_cast<double>(PairCommit::kMaxGeneration) ||
    std::floor(generation_) != generation_ || generation_ != accepted_ + 1)
    return finish(4, Ret::ERROR);
  for (size_t k = 0; k < axes_ * 5; ++k) {
    if (!std::isfinite(commands_[k]) || commands_[k] < lower_[k] || commands_[k] > upper_[k] ||
      (!accepted_ && k % 5 && commands_[k] != 0))
      return finish(5, Ret::ERROR);
  }
  accepted_ = generation_;
  for (size_t a = 0; a < axes_; ++a) {
    states_[4 * a] = commands_[5 * a];
    states_[4 * a + 1] = commands_[5 * a + 1];
    states_[4 * a + 2] = commands_[5 * a + 4];
  }
  return finish(1, Ret::OK);
}

void BxiPairMockSystem::save() noexcept
{
  if (saved_ || directory_.empty()) return;
  try {
    std::filesystem::create_directories(directory_);
    std::ofstream out(directory_ + "/mock_commits.csv");
    out.exceptions(std::ios::failbit | std::ios::badbit);
    out << std::setprecision(17) << "cycle,received,accepted,result";
    for (size_t k = 0; k < axes_ * 5; ++k) out << ",value" << k;
    out << '\n';
    for (size_t i = 0; i < count_; ++i) {
      const auto & recorded_row = records_[i];
      out << recorded_row.cycle << ',' << recorded_row.received << ',' << recorded_row.accepted <<
        ',' << recorded_row.result;
      for (size_t k = 0; k < axes_ * 5; ++k) out << ',' << recorded_row.values[k];
      out << '\n';
    }
    out.close();
    saved_ = true;
  } catch (const std::exception & e) {
    std::fprintf(stderr, "TEST ONLY mock CSV save failed: %s\n", e.what());
  }
}

BxiPairMockSystem::CB BxiPairMockSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  save();
  return CB::SUCCESS;
}

BxiPairMockSystem::~BxiPairMockSystem()
{
  save();
}

}
PLUGINLIB_EXPORT_CLASS(bxi_hardware::BxiPairMockSystem, hardware_interface::SystemInterface)
