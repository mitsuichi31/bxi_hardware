#include <deque>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "bxi_hardware/bxi_system_interface.hpp"

namespace bxi_hardware
{
namespace
{

class EmulatedBus : public CanBus
{
public:
  bool open(const std::string & name) override
  {
    name_ = name;
    open_ = !fail_open;
    return open_;
  }
  void close() override {open_ = false; ++close_count;}
  bool isOpen() const override {return open_;}
  const std::string & name() const override {return name_;}
  void flushRx() override {responses_.clear();}

  bool send(const CanFrame & frame) override
  {
    sent.push_back(frame);
    const bool control_frame = frame.data !=
      protocol::encodeSpecialCommand(protocol::SpecialCommand::kEnable) &&
      frame.data != protocol::encodeSpecialCommand(protocol::SpecialCommand::kDisable);
    if (open_ && reply_ids.count(frame.id) != 0 && (!control_frame || reply_to_control)) {
      CanFrame feedback;
      feedback.id = protocol::defaultMasterId(frame.id);
      feedback.extended = false;
      feedback.dlc = 8;
      feedback.data = {{static_cast<uint8_t>(frame.id), 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 0x55, 0x5A}};
      responses_.push_back(feedback);
    }
    return open_;
  }
  bool recv(CanFrame & frame, int) override
  {
    if (responses_.empty()) {
      return false;
    }
    frame = responses_.front();
    responses_.pop_front();
    return true;
  }

  bool fail_open = false;
  bool reply_to_control = true;
  int close_count = 0;
  std::set<uint32_t> reply_ids;
  std::vector<CanFrame> sent;

private:
  bool open_ = false;
  std::string name_;
  std::deque<CanFrame> responses_;
};

class TestSystemInterface : public BxiSystemInterface
{
public:
  std::set<uint32_t> reply_ids{1};
  bool fail_second_bus_open = false;
  bool reply_to_control = true;
  std::vector<std::shared_ptr<EmulatedBus>> created_buses;

protected:
  std::shared_ptr<CanBus> makeBus(CanFrameFormat) override
  {
    auto bus = std::make_shared<EmulatedBus>();
    bus->reply_ids = reply_ids;
    bus->fail_open = created_buses.size() == 1 && fail_second_bus_open;
    bus->reply_to_control = reply_to_control;
    created_buses.push_back(bus);
    return bus;
  }
};

hardware_interface::HardwareInfo hardwareInfo(const std::string & config)
{
  hardware_interface::HardwareInfo info;
  info.name = "test_bxi";
  info.type = "system";
  info.hardware_parameters["config_file"] =
    std::string(BXI_TEST_SOURCE_DIR) + "/test/config/" + config;
  info.hardware_parameters["spec_dir"] = std::string(BXI_TEST_SOURCE_DIR) + "/config/specs";
  for (const auto & name : {"joint1", "joint2"}) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.type = "joint";
    info.joints.push_back(joint);
  }
  return info;
}

bool hasDisable(const EmulatedBus & bus, uint32_t id)
{
  for (const auto & frame : bus.sent) {
    if (frame.id == id &&
      frame.data == protocol::encodeSpecialCommand(protocol::SpecialCommand::kDisable))
    {
      return true;
    }
  }
  return false;
}

TEST(ActivationRollback, FailedRequiredJointDisablesEarlierJointAndClosesBus)
{
  TestSystemInterface system;
  ASSERT_EQ(system.on_init(hardwareInfo("valid.yaml")), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::ERROR);
  ASSERT_EQ(system.created_buses.size(), 1U);
  EXPECT_TRUE(hasDisable(*system.created_buses[0], 1));
  EXPECT_FALSE(system.created_buses[0]->isOpen());
  EXPECT_GT(system.created_buses[0]->close_count, 0);
}

TEST(ActivationRollback, LaterBusOpenFailureDisablesEarlierBusAndClosesIt)
{
  TestSystemInterface system;
  system.fail_second_bus_open = true;
  ASSERT_EQ(system.on_init(hardwareInfo("activation_two_buses.yaml")),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::ERROR);
  ASSERT_EQ(system.created_buses.size(), 2U);
  EXPECT_TRUE(hasDisable(*system.created_buses[0], 1));
  EXPECT_FALSE(system.created_buses[0]->isOpen());
  EXPECT_GT(system.created_buses[0]->close_count, 0);
}

TEST(ActivationRollback, LaterSpecFailureDisablesEarlierJointAndClosesBus)
{
  TestSystemInterface system;
  ASSERT_EQ(system.on_init(hardwareInfo("activation_missing_spec.yaml")),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::ERROR);
  ASSERT_EQ(system.created_buses.size(), 1U);
  EXPECT_TRUE(hasDisable(*system.created_buses[0], 1));
  EXPECT_FALSE(system.created_buses[0]->isOpen());
}

TEST(ActivationRollback, AllRequiredJointsFailDoesNotReportSuccess)
{
  TestSystemInterface system;
  system.reply_ids.clear();
  ASSERT_EQ(system.on_init(hardwareInfo("valid.yaml")), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::ERROR);
  ASSERT_EQ(system.created_buses.size(), 1U);
  EXPECT_FALSE(system.created_buses[0]->isOpen());
}

TEST(ActivationRollback, MissingInitialFeedbackRollsBackEnabledJoints)
{
  TestSystemInterface system;
  system.reply_ids = {1, 2};
  system.reply_to_control = false;
  ASSERT_EQ(system.on_init(hardwareInfo("valid.yaml")), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::ERROR);
  ASSERT_EQ(system.created_buses.size(), 1U);
  EXPECT_TRUE(hasDisable(*system.created_buses[0], 1));
  EXPECT_TRUE(hasDisable(*system.created_buses[0], 2));
  EXPECT_FALSE(system.created_buses[0]->isOpen());
}

}  // namespace
}  // namespace bxi_hardware
