#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include "bxi_hardware/bxi_pair_mock_system.hpp"

namespace
{
using Mock = bxi_hardware::BxiPairMockSystem;
using CB = hardware_interface::CallbackReturn;
using Ret = hardware_interface::return_type;

hardware_interface::HardwareInfo mockInfo(size_t axes)
{
  hardware_interface::HardwareInfo info;
  info.name = "test_pair";
  info.type = "system";
  const std::vector<std::string> commands = {"position", "velocity", "kp", "kd", "effort"};
  const std::vector<std::string> states = {"position", "velocity", "effort", "temperature"};
  for (size_t a = 0; a < axes; ++a) {
    hardware_interface::ComponentInfo joint;
    joint.name = "test_joint_" + std::to_string(a);
    joint.type = "joint";
    for (const auto & name : commands) {
      hardware_interface::InterfaceInfo interface;
      interface.name = name;
      interface.data_type = "double";
      interface.min = name == "kp" || name == "kd" ? "0" : "-5";
      interface.max = "20";
      joint.command_interfaces.push_back(interface);
    }
    for (const auto & name : states) {
      hardware_interface::InterfaceInfo interface;
      interface.name = name;
      interface.data_type = "double";
      interface.initial_value = name == "temperature" ? "25" : "0";
      joint.state_interfaces.push_back(interface);
    }
    info.joints.push_back(joint);
  }
  hardware_interface::ComponentInfo gpio;
  gpio.name = "pair";
  gpio.type = "gpio";
  hardware_interface::InterfaceInfo interface;
  interface.name = "commit_generation";
  interface.data_type = "double";
  gpio.command_interfaces.push_back(interface);
  interface.name = "accepted_generation";
  gpio.state_interfaces.push_back(interface);
  info.gpios.push_back(gpio);
  return info;
}

struct Rig
{
  Mock system;
  std::vector<hardware_interface::CommandInterface> commands;
  std::vector<hardware_interface::StateInterface> states;
  std::vector<std::string> keys;
  void init(size_t axes)
  {
    ASSERT_EQ(system.on_init(mockInfo(axes)), CB::SUCCESS);
    commands = system.export_command_interfaces();
    states = system.export_state_interfaces();
    for (auto & command : commands) {
      keys.push_back(command.get_name());
      ASSERT_TRUE(command.set_value(0.));
    }
    ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()), CB::SUCCESS);
    ASSERT_EQ(system.perform_command_mode_switch(keys, {}), Ret::OK);
  }

  Ret write(double generation)
  {
    EXPECT_TRUE(commands.back().set_value(generation));
    return system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(.001));
  }

  double accepted() {return states.back().get_value();}
};

TEST(PairMock, TwoAndThreeAxesRequireCompleteClaimsAndAdoptWholeGeneration)
{
  for (size_t axes : {2u, 3u}) {
    Rig rig;
    rig.init(axes);
    ASSERT_EQ(rig.commands.size(), 5 * axes + 1);
    ASSERT_EQ(rig.states.size(), 4 * axes + 1);
    auto incomplete = rig.keys;
    incomplete.pop_back();
    EXPECT_EQ(rig.system.prepare_command_mode_switch(incomplete, {}), Ret::ERROR);
    auto reordered = rig.keys;
    std::reverse(reordered.begin(), reordered.end());
    EXPECT_EQ(rig.system.prepare_command_mode_switch(reordered, {}), Ret::OK);
    EXPECT_EQ(rig.write(1), Ret::OK);
    EXPECT_EQ(rig.accepted(), 1);
    for (size_t a = 0; a < axes; ++a) {
      ASSERT_TRUE(rig.commands[5 * a].set_value(.1 * (a + 1)));
      ASSERT_TRUE(rig.commands[5 * a + 2].set_value(10.));
      ASSERT_TRUE(rig.commands[5 * a + 3].set_value(1.));
    }
    EXPECT_EQ(rig.write(2), Ret::OK);
    EXPECT_EQ(rig.accepted(), 2);
    for (size_t a = 0; a < axes; ++a) EXPECT_DOUBLE_EQ(rig.states[4 * a].get_value(), .1 * (a + 1));
    EXPECT_EQ(rig.write(2), Ret::OK);
    EXPECT_EQ(rig.accepted(), 2);
  }
}

TEST(PairMock, RejectsNonfiniteOutOfBoundsAndNonzeroBootGainsWithoutAdoption)
{
  for (double value : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), 21.})
  {
    Rig rig;
    rig.init(3);
    ASSERT_TRUE(rig.commands[0].set_value(value));
    EXPECT_EQ(rig.write(1), Ret::ERROR);
    EXPECT_EQ(rig.accepted(), 0);
  }
  Rig rig;
  rig.init(3);
  ASSERT_TRUE(rig.commands[2].set_value(1.));
  EXPECT_EQ(rig.write(1), Ret::ERROR);
  EXPECT_EQ(rig.accepted(), 0);
}

TEST(PairMock, RejectsSkippedFractionalNegativeAndNonfiniteGenerations)
{
  for (double generation : {2., 1.5, -1., std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN(), 9007199254740992.})
  {
    Rig rig;
    rig.init(2);
    EXPECT_EQ(rig.write(generation), Ret::ERROR);
    EXPECT_EQ(rig.accepted(), 0);
  }
}

TEST(PairMock, FaultInjectionReadWriteAndFrozenAdoption)
{
  Rig rig;
  rig.init(3);
  rig.system.reject_at.store(1);
  EXPECT_EQ(rig.write(1), Ret::OK);
  EXPECT_EQ(rig.accepted(), 0);
  rig.system.write_error_at.store(2);
  EXPECT_EQ(rig.write(1), Ret::ERROR);
  EXPECT_EQ(rig.accepted(), 0);
  rig.system.read_error_at.store(1);
  EXPECT_EQ(rig.system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(.001)), Ret::ERROR);
}

TEST(PairMock, StartupBoundsAreInclusiveAndOutsideOrNonfiniteRejectsActivation)
{
  for (double position : {-5., 20., -5.0001, 20.0001,
      std::numeric_limits<double>::quiet_NaN()})
  {
    auto info = mockInfo(3);
    info.joints[1].state_interfaces[0].initial_value = std::to_string(position);
    Mock system;
    ASSERT_EQ(system.on_init(info), CB::SUCCESS);
    EXPECT_EQ(system.on_activate(rclcpp_lifecycle::State()),
      std::isfinite(position) && position >= -5 && position <= 20 ? CB::SUCCESS : CB::ERROR);
  }
}

}  // namespace
