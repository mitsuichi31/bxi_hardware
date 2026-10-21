#ifndef BXI_HARDWARE__BXI_PAIR_ADAPTER_HPP_
#define BXI_HARDWARE__BXI_PAIR_ADAPTER_HPP_
#include <array>
#include <vector>
#include <string>
#include <cstdint>
#include "controller_interface/chainable_controller_interface.hpp"
namespace bxi_hardware
{
class BxiPairAdapter : public controller_interface::ChainableControllerInterface {
public:
  using CB = controller_interface::CallbackReturn;
  using Ret = controller_interface::return_type;
  CB on_init() override;
  CB on_configure(const rclcpp_lifecycle::State &) override;
  CB on_activate(const rclcpp_lifecycle::State &) override;
  CB on_deactivate(const rclcpp_lifecycle::State &) override;
  CB on_cleanup(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  Ret update_reference_from_subscribers(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return Ret::OK;
  }

  Ret update_and_write_commands(const rclcpp::Time &, const rclcpp::Duration &) override;
  ~BxiPairAdapter() override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  bool on_set_chained_mode(bool) override
  {
    return true;
  }

  virtual bool setCommand(size_t i, double value)
  {
    return command_interfaces_[command_index_[i]].set_value<double>(value, 1);
  }

  virtual std::optional<double> getCommand(size_t i)
  {
    return command_interfaces_[command_index_[i]].get_optional<double>(1);
  }

  virtual std::optional<double> getState(size_t i)
  {
    return state_interfaces_[state_index_[i]].get_optional<double>(1);
  }

private:
  struct Limits
  {
    double position_min, position_max, velocity, kp, kd, kp_max, kd_max, delta, torque,
      deviation, temperature;
    unsigned cycles;
  };
  struct Group
  {
    size_t first = 0, axes = 0, command = 0, state = 0;
    uint64_t generation = 0;
    std::string gpio, name;
  };
  struct Attempt
  {
    std::array<double, 64> values{}, readback{};
    std::array<int64_t, 4> commit_set{}, commit_read{};
    uint64_t write_attempted = 0, write_ok = 0, read_attempted = 0, read_ok = 0, unavailable = 0,
      mismatch = 0;
  };
  struct Row
  {
    uint64_t activation = 0, cycle = 0;
    int64_t start = 0, end = 0;
    double dt = 0;
    int reason = 0;
    uint64_t state_ok = 0;
    uint16_t reference_ok = 0;
    std::array<double, 52> states{};
    std::array<double, 12> raw{}, target{}, previous{}, delta{}, velocity{}, torque{};
    std::array<double, 12> raw_velocity{}, target_velocity{}, torque_with_kd{};
    std::array<unsigned, 12> deviation_count{};
    std::array<uint64_t, 4> previous_g{}, candidate_g{};
    Attempt normal{}, release{};
    uint8_t commit_touched = 0, release_missing = 0;
  };
  bool bind();
  bool writeValue(size_t, double, Attempt &);
  bool writeBatch(Row &, bool);
  void save() noexcept;
  void record(Row &);
  std::vector<std::string> joints_, command_keys_, state_keys_;
  std::array<Group, 4> groups_{};
  size_t group_count_ = 0;
  std::array<Limits, 12> limits_{};
  std::array<size_t, 64> command_index_{};
  std::array<size_t, 52> state_index_{};
  std::array<double, 12> boot_{}, previous_{};
  std::array<bool, 12> seen_reference_{};
  std::array<bool, 12> seen_velocity_{};
  bool velocity_feedforward_ = false;
  std::array<unsigned, 12> deviation_count_{};
  std::vector<Row> rows_;
  size_t count_ = 0;
  Row final_{};
  uint64_t activation_ = 0, cycle_ = 0, dropped_ = 0;
  bool activated_once_ = false, active_ = false, terminal_ = false, saved_ = false;
  double max_period_ = 0.005;
  std::string directory_;
};
}
#endif
