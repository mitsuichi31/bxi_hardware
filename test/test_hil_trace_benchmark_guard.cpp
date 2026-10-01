#include <cstdlib>
#include <gtest/gtest.h>
#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"

// The rejection happens in on_init(), before the base can open any CAN socket or enable.
TEST(HilTraceBenchmarkGuard, RefusesPhysicalConfigurationEvenWithTestOptIn)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  auto hardware = loader.createSharedInstance("bxi_hardware/HilTraceBenchmarkSystem");
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["config_file"] =
    std::string(BXI_TEST_SOURCE_DIR) + "/hil/single_joint/bxi_hardware_stream_1khz.yaml";
  unsetenv("BXI_ALLOW_TEST_FAULTS");
  EXPECT_EQ(hardware->on_init(info), hardware_interface::CallbackReturn::ERROR);
  setenv("BXI_ALLOW_TEST_FAULTS", "1", 1);
  EXPECT_EQ(hardware->on_init(info), hardware_interface::CallbackReturn::ERROR);
  unsetenv("BXI_ALLOW_TEST_FAULTS");
}
