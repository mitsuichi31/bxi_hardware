#ifndef BXI_HARDWARE__BXI_PAIR_MOCK_SYSTEM_HPP_
#define BXI_HARDWARE__BXI_PAIR_MOCK_SYSTEM_HPP_
#include <array>
#include <atomic>
#include <string>
#include <vector>
#include "hardware_interface/system_interface.hpp"
namespace bxi_hardware
{
// TEST ONLY. No CAN I/O, motor dynamics, or safety-batch emulation.
class BxiPairMockSystem : public hardware_interface::SystemInterface {
public:
  using CB = hardware_interface::CallbackReturn;
  using Ret = hardware_interface::return_type;
  CB on_init(const hardware_interface::HardwareInfo &) override;
  CB on_activate(const rclcpp_lifecycle::State &) override;
  CB on_deactivate(const rclcpp_lifecycle::State &) override
  {
    return CB::SUCCESS;
  }

  CB on_cleanup(const rclcpp_lifecycle::State &) override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  Ret prepare_command_mode_switch(
    const std::vector<std::string> &,
    const std::vector<std::string> &) override;
  Ret perform_command_mode_switch(
    const std::vector<std::string> &,
    const std::vector<std::string> &) override;
  Ret read(const rclcpp::Time &, const rclcpp::Duration &) override;
  Ret write(const rclcpp::Time &, const rclcpp::Duration &) override;
  ~BxiPairMockSystem() override;
  std::atomic<uint64_t> read_error_at{0}, write_error_at{0}, reject_at{0};

private:
  struct Row
  {
    uint64_t cycle = 0;
    double received = 0, accepted = 0;
    int result = 0;
    std::array<double, 15> values{};
  };
  void save() noexcept;
  size_t axes_ = 0;
  std::string gpio_, directory_;
  std::vector<std::string> keys_;
  std::array<double, 15> commands_{}, lower_{}, upper_{};
  std::array<double, 12> states_{};
  double generation_ = 0, accepted_ = 0;
  uint64_t reads_ = 0, writes_ = 0;
  bool claimed_ = false, saved_ = false;
  std::vector<Row> records_;
  size_t count_ = 0;
};
}
#endif
