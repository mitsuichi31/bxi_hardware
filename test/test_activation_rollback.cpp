#include <atomic>
#include <chrono>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
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

std::string writeConfig(const std::string & file_name, const std::string & text)
{
  const std::string path = ::testing::TempDir() + file_name;
  std::ofstream(path) << text;
  return path;
}

hardware_interface::HardwareInfo hardwareInfoAt(const std::string & config_path)
{
  auto info = hardwareInfo("valid.yaml");
  info.hardware_parameters["config_file"] = config_path;
  return info;
}

// joint1 and joint2 on can0 with IDs 1 and 2 unless replaced.
std::string transportConfig(
  const std::string & transport, const std::string & format = "fd", int can_hz = 1000,
  const std::string & joint2 = "{enabled: true, can_bus: can0, can_id: 2, motor_dir: 1, "
  "motor_type: MOTOR_70, lower: -1.0, upper: 1.0}")
{
  return "active_can_buses: [can0]\n"
         "can_frame_format: {can0: " + format + "}\n"
         "defaults: {control_timeout_ms: 3, maximum_consecutive_timeouts: 3, can_hz: " +
         std::to_string(can_hz) + "}\n"
         "can_transport:\n  can0: " + transport + "\n"
         "joints:\n"
         "  joint1: {enabled: true, can_bus: can0, can_id: 1, motor_dir: 1, motor_type: MOTOR_70, "
         "lower: -1.0, upper: 1.0}\n"
         "  joint2: " + joint2 + "\n";
}

hardware_interface::CallbackReturn initWith(const std::string & name, const std::string & text)
{
  TestSystemInterface system;
  const auto result = system.on_init(hardwareInfoAt(writeConfig(name, text)));
  EXPECT_TRUE(system.created_buses.empty());
  return result;
}

TEST(Configuration, AcceptsStreamedTransports)
{
  EXPECT_EQ(initWith("stream_broadcast.yaml", transportConfig(
      "{rx_mode: separate, tx_frame: broadcast, stream_feedback_max_age_ms: 3}")),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(initWith("stream_unicast.yaml", transportConfig(
      "{rx_mode: separate, stream_feedback_max_age_ms: 15}", "classic", 200)),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(initWith("stream_sequential.yaml", transportConfig("{rx_mode: sequential}")),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(initWith("stream_rt.yaml", transportConfig(
      "{rx_mode: separate, stream_feedback_max_age_ms: 3, send_thread_priority: 60, "
      "receive_thread_priority: 61, thread_cpus: [2, 3]}")),
    hardware_interface::CallbackReturn::SUCCESS);
}

TEST(Configuration, RejectsInvalidStreamedTransports)
{
  const std::vector<std::pair<std::string, std::string>> cases = {
    {"no_max_age", transportConfig("{rx_mode: separate}")},
    {"max_age_below_two_periods",
      transportConfig("{rx_mode: separate, stream_feedback_max_age_ms: 1.5}")},
    {"max_age_above_50ms", transportConfig("{rx_mode: separate, stream_feedback_max_age_ms: 60}")},
    {"broadcast_without_separate", transportConfig("{tx_frame: broadcast}")},
    {"max_age_without_separate", transportConfig("{stream_feedback_max_age_ms: 3}")},
    {"unknown_key", transportConfig(
        "{rx_mode: separate, stream_feedback_max_age_ms: 3, rx_poll_ms: 1}")},
    {"bad_rx_mode", transportConfig("{rx_mode: threaded, stream_feedback_max_age_ms: 3}")},
    {"priority_99", transportConfig(
        "{rx_mode: separate, stream_feedback_max_age_ms: 3, send_thread_priority: 99}")},
    {"priority_0", transportConfig(
        "{rx_mode: separate, stream_feedback_max_age_ms: 3, receive_thread_priority: 0}")},
    {"empty_cpus", transportConfig(
        "{rx_mode: separate, stream_feedback_max_age_ms: 3, thread_cpus: []}")},
    {"negative_cpu", transportConfig(
        "{rx_mode: separate, stream_feedback_max_age_ms: 3, thread_cpus: [-1]}")},
    {"priority_without_separate", transportConfig("{send_thread_priority: 60}")},
    {"broadcast_classic", transportConfig(
        "{rx_mode: separate, tx_frame: broadcast, stream_feedback_max_age_ms: 3}", "classic")},
    {"broadcast_ids_1_3", transportConfig(
        "{rx_mode: separate, tx_frame: broadcast, stream_feedback_max_age_ms: 3}", "fd", 1000,
        "{enabled: true, can_bus: can0, can_id: 3, motor_dir: 1, motor_type: MOTOR_70, "
        "lower: -1.0, upper: 1.0}")},
    {"isolated_two_motors", transportConfig(
        "{rx_mode: separate, tx_frame: broadcast, stream_feedback_max_age_ms: 3, "
        "hil_isolated_id1_broadcast_blocks: 3}")},
    {"isolated_fd_brs", transportConfig(
        "{rx_mode: separate, tx_frame: broadcast, stream_feedback_max_age_ms: 3, "
        "hil_isolated_id1_broadcast_blocks: 3}", "fd_brs", 1000, "{enabled: false}")},
    {"isolated_blocks_4", transportConfig(
        "{rx_mode: separate, tx_frame: broadcast, stream_feedback_max_age_ms: 3, "
        "hil_isolated_id1_broadcast_blocks: 4}", "fd", 1000, "{enabled: false}")},
  };
  for (const auto & [name, text] : cases) {
    EXPECT_EQ(initWith(name + ".yaml", text), hardware_interface::CallbackReturn::ERROR) << name;
  }
  // The isolated single-motor form itself is accepted on can0.
  EXPECT_EQ(initWith("isolated_ok.yaml", transportConfig(
      "{rx_mode: separate, tx_frame: broadcast, stream_feedback_max_age_ms: 3, "
      "hil_isolated_id1_broadcast_blocks: 3}", "fd", 1000, "{enabled: false}")),
    hardware_interface::CallbackReturn::SUCCESS);
  // A transport entry for a bus that is not active is rejected.
  auto unknown_bus = transportConfig("{rx_mode: sequential}");
  unknown_bus.replace(unknown_bus.find("can_transport:\n  can0:"),
    std::string("can_transport:\n  can0:").size(), "can_transport:\n  can9:");
  EXPECT_EQ(initWith("unknown_bus.yaml", unknown_bus), hardware_interface::CallbackReturn::ERROR);
}

// The emulated send bus answers unicast frames only, so the receive bus never sees a streamed
// reply: activation must fail and disable both motors with acknowledgements.
TEST(ActivationRollback, StreamedBusWithoutStreamedRepliesRollsBack)
{
  TestSystemInterface system;
  system.reply_ids = {1, 2};
  ASSERT_EQ(system.on_init(hardwareInfoAt(writeConfig("stream_rollback.yaml", transportConfig(
      "{rx_mode: separate, tx_frame: broadcast, stream_feedback_max_age_ms: 3}")))),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::ERROR);
  ASSERT_EQ(system.created_buses.size(), 2U);  // send and receive buses
  const auto & tx = *system.created_buses[0];
  EXPECT_TRUE(tx.hasDisable(1));
  EXPECT_TRUE(tx.hasDisable(2));
  EXPECT_FALSE(tx.isOpen());
  EXPECT_FALSE(system.created_buses[1]->isOpen());
  bool broadcast_seen = false;
  for (const auto & frame : tx.sent) {
    broadcast_seen = broadcast_seen || (frame.id == 0x7FFU && frame.dlc == 16);
  }
  EXPECT_TRUE(broadcast_seen);
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
