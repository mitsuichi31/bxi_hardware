// ros2_control HWIF層: state/command interface の export と worker スレッド管理。
// mevius2_hardware/RobstrideSystemInterface を雛形に BXI 用に改修。
#ifndef BXI_HARDWARE__BXI_SYSTEM_INTERFACE_HPP_
#define BXI_HARDWARE__BXI_SYSTEM_INTERFACE_HPP_

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "bxi_hardware/actuator/bxi_actuator.hpp"
#include "bxi_hardware/can/socketcan_bus.hpp"

namespace bxi_hardware
{

struct JointRuntime
{
  std::string name;
  bool enabled = false;
  std::string can_bus;
  int can_id = 0;
  int motor_dir = 1;
  double offset_angle = 0.0;
  std::optional<double> offset_thre;
  double default_kp = 50.0;
  double default_kd = 2.0;
  double lower = 0.0;
  double upper = 0.0;
  std::string motor_type;
  std::unique_ptr<ActuatorDriver> actuator;
  bool overheated = false;
};

class BxiSystemInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool parseConfig();
  bool loadSpec(const std::string & motor_type);
  void workerLoop(std::string bus_name, std::vector<size_t> indices);
  void stopWorkers();

  size_t n_joints_ = 0;

  std::vector<double> st_pos_, st_vel_, st_eff_, st_temp_;
  std::vector<double> cmd_pos_, cmd_vel_, cmd_kp_, cmd_kd_, cmd_eff_;

  std::vector<JointRuntime> joints_;
  std::map<std::string, std::shared_ptr<SocketCanBus>> buses_;
  std::vector<std::string> active_buses_;
  std::map<std::string, BxiSpec> specs_;

  std::string config_file_;
  std::string spec_dir_;
  int can_hz_ = 200;  // 初期 200Hz、YAMLで上書き可
  int main_can_id_ = 254;
  int control_timeout_ms_ = 3;

  std::mutex data_mutex_;
  std::vector<double> tgt_pos_, tgt_vel_, tgt_kp_, tgt_kd_, tgt_eff_;
  std::vector<double> fb_pos_, fb_vel_, fb_eff_, fb_temp_;

  std::vector<std::thread> workers_;
  std::map<std::string, std::vector<size_t>> live_joints_;
  std::atomic<bool> running_{false};
};

}  // namespace bxi_hardware

#endif  // BXI_HARDWARE__BXI_SYSTEM_INTERFACE_HPP_
