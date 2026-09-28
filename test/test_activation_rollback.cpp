#include <atomic>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
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
    std::lock_guard<std::mutex> lock(mutex_);
    name_ = name;
    open_ = !fail_open;
    return open_;
  }
  void close() override {std::lock_guard<std::mutex> lock(mutex_); open_ = false; ++close_count;}
  bool isOpen() const override {std::lock_guard<std::mutex> lock(mutex_); return open_;}
  const std::string & name() const override {return name_;}
  void flushRx() override {std::lock_guard<std::mutex> lock(mutex_); responses_.clear();}

  bool send(const CanFrame & frame) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sent.push_back(frame);
    const bool control_frame = frame.data8() !=
      protocol::encodeSpecialCommand(protocol::SpecialCommand::kEnable) &&
      frame.data8() != protocol::encodeSpecialCommand(protocol::SpecialCommand::kDisable);
    if (open_ && reply_ids.count(frame.id) != 0 && (!control_frame || reply_to_control)) {
      CanFrame feedback;
      feedback.id = protocol::defaultMasterId(frame.id);
      feedback.extended = false;
      feedback.dlc = 8;
      const bool both_faulted = faulted_ids.count(frame.id) != 0 || double_temperature_fault;
      feedback.data = {{static_cast<uint8_t>(frame.id), 0x7F, 0xFF, 0x7F, 0xF7, 0xFF,
        static_cast<uint8_t>(both_faulted ? 0x00 : 0x55),
        static_cast<uint8_t>(both_faulted ? 0x02 : 0x5A)}};
      responses_.push_back(feedback);
    }
    return open_;
  }
  bool recv(CanFrame & frame, int) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (responses_.empty()) {
      return false;
    }
    frame = responses_.front();
    responses_.pop_front();
    return true;
  }

  bool fail_open = false;
  bool reply_to_control = true;
  std::atomic<bool> double_temperature_fault{false};
  int close_count = 0;
  std::set<uint32_t> reply_ids;
  std::set<uint32_t> faulted_ids;
  std::vector<CanFrame> sent;

  bool hasDisable(uint32_t id) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & frame : sent) {
      if (frame.id == id &&
        frame.data8() == protocol::encodeSpecialCommand(protocol::SpecialCommand::kDisable))
      {
        return true;
      }
    }
    return false;
  }

  size_t sentCount() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return sent.size();
  }

private:
  mutable std::mutex mutex_;
  bool open_ = false;
  std::string name_;
  std::deque<CanFrame> responses_;
};

class TestSystemInterface : public BxiSystemInterface
{
public:
  std::set<uint32_t> reply_ids{1};
  std::set<uint32_t> faulted_ids;
  bool fail_second_bus_open = false;
  bool reply_to_control = true;
  std::vector<std::shared_ptr<EmulatedBus>> created_buses;

protected:
  std::shared_ptr<CanBus> makeBus(CanFrameFormat) override
  {
    auto bus = std::make_shared<EmulatedBus>();
    bus->reply_ids = reply_ids;
    bus->faulted_ids = faulted_ids;
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

TEST(ActivationRollback, FailedRequiredJointDisablesEarlierJointAndClosesBus)
{
  TestSystemInterface system;
  ASSERT_EQ(system.on_init(hardwareInfo("valid.yaml")), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::ERROR);
  ASSERT_EQ(system.created_buses.size(), 1U);
  EXPECT_TRUE(system.created_buses[0]->hasDisable(1));
  EXPECT_FALSE(system.created_buses[0]->isOpen());
  EXPECT_GT(system.created_buses[0]->close_count, 0);
}

TEST(Configuration, RejectsMotorCanId16BeforeOpeningBus)
{
  TestSystemInterface system;
  EXPECT_EQ(system.on_init(hardwareInfo("invalid_can_id16.yaml")),
    hardware_interface::CallbackReturn::ERROR);
  EXPECT_TRUE(system.created_buses.empty());
}

TEST(ActivationRollback, LaterBusOpenFailureDisablesEarlierBusAndClosesIt)
{
  TestSystemInterface system;
  system.fail_second_bus_open = true;
  ASSERT_EQ(system.on_init(hardwareInfo("activation_two_buses.yaml")),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::ERROR);
  ASSERT_EQ(system.created_buses.size(), 2U);
  EXPECT_TRUE(system.created_buses[0]->hasDisable(1));
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
  EXPECT_TRUE(system.created_buses[0]->hasDisable(1));
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
  EXPECT_TRUE(system.created_buses[0]->hasDisable(1));
  EXPECT_TRUE(system.created_buses[0]->hasDisable(2));
  EXPECT_FALSE(system.created_buses[0]->isOpen());
}

TEST(ActivationRollback, DualTemperatureFaultOnEnableRollsBack)
{
  TestSystemInterface system;
  system.reply_ids = {1, 2};
  system.faulted_ids = {2};
  ASSERT_EQ(system.on_init(hardwareInfo("valid.yaml")), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::ERROR);
  ASSERT_EQ(system.created_buses.size(), 1U);
  EXPECT_TRUE(system.created_buses[0]->hasDisable(1));
  EXPECT_TRUE(system.created_buses[0]->hasDisable(2));
  EXPECT_FALSE(system.created_buses[0]->isOpen());
}

TEST(SystemSafety, DualTemperatureFaultDuringOperationStopsBothBuses)
{
  TestSystemInterface system;
  system.reply_ids = {1, 2};
  ASSERT_EQ(system.on_init(hardwareInfo("activation_two_buses.yaml")),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  struct Cleanup
  {
    BxiSystemInterface & system;
    ~Cleanup() {system.on_deactivate(rclcpp_lifecycle::State());}
  } cleanup{system};
  ASSERT_EQ(system.created_buses.size(), 2U);
  system.created_buses[0]->double_temperature_fault = true;

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (std::chrono::steady_clock::now() < deadline &&
    !(system.created_buses[0]->hasDisable(1) && system.created_buses[1]->hasDisable(2)))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(system.created_buses[0]->hasDisable(1));
  ASSERT_TRUE(system.created_buses[1]->hasDisable(2));
  const auto first_bus_count = system.created_buses[0]->sentCount();
  const auto second_bus_count = system.created_buses[1]->sentCount();
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_EQ(system.created_buses[0]->sentCount(), first_bus_count);
  EXPECT_EQ(system.created_buses[1]->sentCount(), second_bus_count);
}

}  // namespace
}  // namespace bxi_hardware
