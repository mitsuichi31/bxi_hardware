#include "bxi_hardware/hil_update_stall_controller.hpp"

#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>

#include "pluginlib/class_list_macros.hpp"

namespace bxi_hardware
{

controller_interface::CallbackReturn HilUpdateStallController::on_init()
{
  auto_declare<int64_t>("stall_ms", 50);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
HilUpdateStallController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
HilUpdateStallController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::CallbackReturn HilUpdateStallController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  const char * allowed = std::getenv("BXI_ALLOW_TEST_FAULTS");
  if (allowed == nullptr || std::string(allowed) != "1") {
    RCLCPP_ERROR(node->get_logger(),
      "HilUpdateStallController は vcan の故障経路の試験専用で、BXI_ALLOW_TEST_FAULTS=1 のときだけ使える");
    return controller_interface::CallbackReturn::ERROR;
  }
  stall_ms_ = node->get_parameter("stall_ms").as_int();
  if (stall_ms_ <= 0 || stall_ms_ > 10'000) {
    RCLCPP_ERROR(node->get_logger(), "stall_ms は 1〜10000");
    return controller_interface::CallbackReturn::ERROR;
  }
  stall_service_ = node->create_service<std_srvs::srv::Trigger>("~/stall",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        requestStall();
        response->success = true;
        response->message = "次の update() で止まる";
      });
  RCLCPP_WARN(node->get_logger(),
    "試験用：~/stall で controller_manager の更新を %ld ms 止める。実機では使わない",
    static_cast<long>(stall_ms_));
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type HilUpdateStallController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Deliberately blocks the real-time loop: this is the fault being tested.
  if (stall_requested_.exchange(false, std::memory_order_acq_rel)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(stall_ms_));
    stalls_.fetch_add(1, std::memory_order_release);
  }
  return controller_interface::return_type::OK;
}

}  // namespace bxi_hardware

PLUGINLIB_EXPORT_CLASS(
  bxi_hardware::HilUpdateStallController, controller_interface::ControllerInterface)
