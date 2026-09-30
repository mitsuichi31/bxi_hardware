// HilUpdateStallController: test only (vcan fault paths). After ~/stall is called, the next
// update() sleeps for stall_ms, so controller_manager's loop (read, update, write) stops for
// that long while the HWIF's send and receive threads keep running. This reproduces the stalled
// control loop that the HWIF's stream_command_max_age_ms has to catch.
//
// It claims no interfaces. It is built into its own library (bxi_hil_test_controllers) and
// refuses to configure unless BXI_ALLOW_TEST_FAULTS=1, which the HIL runners never set.
// Plan: docs/plans/20260929_bxi_rt_trajectory_controller_plan.md, step 4 (workspace).
#ifndef BXI_HARDWARE__HIL_UPDATE_STALL_CONTROLLER_HPP_
#define BXI_HARDWARE__HIL_UPDATE_STALL_CONTROLLER_HPP_

#include <atomic>
#include <cstdint>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace bxi_hardware
{

class HilUpdateStallController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Non-real-time: the next update() stalls once. Tests call it directly.
  void requestStall() {stall_requested_.store(true, std::memory_order_release);}
  // Number of stalls done by update().
  uint32_t stalls() const {return stalls_.load(std::memory_order_acquire);}

private:
  int64_t stall_ms_ = 50;
  std::atomic<bool> stall_requested_{false};
  std::atomic<uint32_t> stalls_{0};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stall_service_;
};

}  // namespace bxi_hardware

#endif  // BXI_HARDWARE__HIL_UPDATE_STALL_CONTROLLER_HPP_
