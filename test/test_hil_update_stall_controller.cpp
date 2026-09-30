// HilUpdateStallController (test only): refused without BXI_ALLOW_TEST_FAULTS=1, and one
// requested stall blocks exactly one update() for stall_ms.
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "controller_interface/controller_interface_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "bxi_hardware/hil_update_stall_controller.hpp"

namespace bxi_hardware
{
namespace
{

class StallControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase() {rclcpp::init(0, nullptr);}
  static void TearDownTestCase() {rclcpp::shutdown();}
  void TearDown() override {unsetenv("BXI_ALLOW_TEST_FAULTS");}

  std::unique_ptr<HilUpdateStallController> make(int64_t stall_ms)
  {
    auto controller = std::make_unique<HilUpdateStallController>();
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("stall_ms", stall_ms)});
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "hil_update_stall_" + std::to_string(serial_++);
    params.update_rate = 1000;
    params.node_options = options;
    EXPECT_EQ(controller->init(params), controller_interface::return_type::OK);
    return controller;
  }

  static double updateSeconds(HilUpdateStallController & controller)
  {
    const auto start = std::chrono::steady_clock::now();
    EXPECT_EQ(controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001)),
      controller_interface::return_type::OK);
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  }

private:
  int serial_ = 0;
};

TEST_F(StallControllerTest, RefusedWithoutTheEnvironmentSwitch)
{
  unsetenv("BXI_ALLOW_TEST_FAULTS");
  EXPECT_EQ(make(50)->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
  ASSERT_EQ(setenv("BXI_ALLOW_TEST_FAULTS", "true", 1), 0);
  EXPECT_EQ(make(50)->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(StallControllerTest, StallMsOutOfRangeIsRefused)
{
  ASSERT_EQ(setenv("BXI_ALLOW_TEST_FAULTS", "1", 1), 0);
  EXPECT_EQ(make(0)->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
  EXPECT_EQ(make(10'001)->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(StallControllerTest, OneRequestStallsOneUpdate)
{
  ASSERT_EQ(setenv("BXI_ALLOW_TEST_FAULTS", "1", 1), 0);
  auto controller = make(30);
  ASSERT_EQ(controller->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  EXPECT_LT(updateSeconds(*controller), 0.01);  // nothing requested
  controller->requestStall();
  EXPECT_GE(updateSeconds(*controller), 0.03);
  EXPECT_EQ(controller->stalls(), 1U);
  EXPECT_LT(updateSeconds(*controller), 0.01);  // once only
  EXPECT_EQ(controller->stalls(), 1U);
}

}  // namespace
}  // namespace bxi_hardware
