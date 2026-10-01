#include "bxi_hardware/hil_update_stall_controller.hpp"

#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>
#include <fstream>
#include <sched.h>
#include <sys/syscall.h>
#include <unistd.h>

#include "pluginlib/class_list_macros.hpp"

namespace bxi_hardware
{
HilUpdateStallController::~HilUpdateStallController()
{
  // Controller manager has finished the update loop before destroying plugins.
  // No file output in update() or lifecycle callbacks on the update thread.
  if (!probe_path_.empty()) {
    std::ofstream out(probe_path_);
    out << "#policy=" << probe_policy_ << ",priority=" << probe_priority_ << ",cpu=" << probe_cpu_
        << ",tid=" << probe_tid_ << ",rows=" << std::min(probe_count_, probe_times_.size())
        << ",dropped=" << (probe_count_ > probe_times_.size() ? probe_count_ - probe_times_.size() : 0)
        << "\nsteady_ns\n";
    for (size_t i = 0; i < std::min(probe_count_, probe_times_.size()); ++i) {out << probe_times_[i] << '\n';}
    out.close();
    if (!out) {RCLCPP_ERROR(get_node()->get_logger(), "update probe write failed");}
  }
}

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
  if (const char * directory = std::getenv("BXI_WORKER_TIMING_DIR")) {
    probe_path_ = std::string(directory) + "/controller_update_probe.csv";
    probe_times_.resize(120000);
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
  if (!probe_times_.empty()) {
    if (probe_count_ == 0) {
      sched_param scheduling{}; ::sched_getparam(0, &scheduling);
      probe_policy_ = ::sched_getscheduler(0); probe_priority_ = scheduling.sched_priority;
      probe_cpu_ = ::sched_getcpu(); probe_tid_ = ::syscall(SYS_gettid);
    }
    if (probe_count_ < probe_times_.size()) {
      probe_times_[probe_count_] = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    ++probe_count_;
  }
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
