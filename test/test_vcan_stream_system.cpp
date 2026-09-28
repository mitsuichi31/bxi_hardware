// BxiSystemInterface with rx_mode: separate on vcan, against emulated motors whose replies can
// be delayed, dropped, or silenced. Skipped unless BXI_VCAN_IFACE names a CAN FD vcan.
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <fstream>
#include <map>
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

using namespace std::chrono_literals;
using clock_type = std::chrono::steady_clock;

CanFrame feedbackFrame(uint8_t motor_id, bool overheated = false)
{
  CanFrame frame;
  frame.id = protocol::defaultMasterId(motor_id);
  frame.extended = false;
  frame.dlc = 8;
  // Raw 0xFF is 150 degC, above the 80 degC stop threshold; 0x55/0x5A are about 30 degC.
  const uint8_t ntc = overheated ? 0xFF : 0x55;
  frame.data = {{motor_id, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, ntc, overheated ? ntc : uint8_t{0x5A}}};
  return frame;
}

// Motors with IDs 1..3 on one vcan. Unicast frames are answered at once; replies to the 0x7FF
// broadcast follow the configured delay pattern on a separate sender thread. Receiving and
// sending use separate sockets: SocketCanBus holds its I/O mutex while recv() waits, which would
// otherwise delay every reply by up to the receive timeout.
class MotorEmulator
{
public:
  MotorEmulator(const char * interface_name, std::set<int> ids)
  : bus_(CanFrameFormat::kFd), tx_bus_(CanFrameFormat::kFd), ids_(std::move(ids))
  {
    opened_ = bus_.open(interface_name) && tx_bus_.open(interface_name) &&
      tx_bus_.setReceiveEnabled(false);
    if (opened_) {
      receiver_ = std::thread([this]() {receiveLoop();});
      sender_ = std::thread([this]() {sendLoop();});
    }
  }
  ~MotorEmulator()
  {
    running_ = false;
    cv_.notify_all();
    if (receiver_.joinable()) {
      receiver_.join();
    }
    if (sender_.joinable()) {
      sender_.join();
    }
    bus_.close();
    tx_bus_.close();
  }

  bool opened() const {return opened_;}
  void setDelays(std::vector<clock_type::duration> delays)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    delays_ = std::move(delays);
  }
  int disables(int id) const {return disables_[static_cast<size_t>(id)].load();}

  std::atomic<bool> answer_broadcast{true};
  std::atomic<bool> overheat{false};  // every reply reports 150 degC
  std::atomic<int> silent_mask{0};  // bit id - 1: ignore the broadcast
  std::atomic<int> drop_next{0};
  std::atomic<int> broadcast_frames{0};
  std::atomic<int> last_broadcast_length{0};

private:
  void schedule(clock_type::time_point due, int id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.emplace(due, id);
    cv_.notify_all();
  }

  void receiveLoop()
  {
    const auto disable = protocol::encodeSpecialCommand(protocol::SpecialCommand::kDisable);
    CanFrame frame;
    while (running_) {
      if (!bus_.recv(frame, 5) || frame.extended) {
        continue;
      }
      const auto now = clock_type::now();
      const int id = static_cast<int>(frame.id);
      if (id >= 1 && id <= 3 && frame.dlc == 8 && ids_.count(id) != 0) {
        if (frame.data8() == disable) {
          ++disables_[static_cast<size_t>(id)];
        }
        schedule(now, id);
      } else if (frame.id == 0x7FFU) {
        ++broadcast_frames;
        last_broadcast_length = frame.dlc;
        if (!answer_broadcast) {
          continue;
        }
        for (int block = 0; block < frame.dlc / 8; ++block) {
          const int motor = block + 1;
          if (ids_.count(motor) == 0 || (silent_mask.load() & (1 << block)) != 0) {
            continue;
          }
          if (drop_next.load() > 0) {
            --drop_next;
            continue;
          }
          clock_type::duration delay;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            delay = delays_[delay_index_++ % delays_.size()];
          }
          schedule(now + delay, motor);
        }
      }
    }
  }

  void sendLoop()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (running_) {
      if (pending_.empty()) {
        cv_.wait_for(lock, 5ms);
        continue;
      }
      const auto due = pending_.begin()->first;
      if (clock_type::now() < due) {
        cv_.wait_until(lock, due);
        continue;
      }
      const int id = pending_.begin()->second;
      pending_.erase(pending_.begin());
      lock.unlock();
      tx_bus_.send(feedbackFrame(static_cast<uint8_t>(id), overheat.load()));
      lock.lock();
    }
  }

  SocketCanBus bus_;
  SocketCanBus tx_bus_;
  std::set<int> ids_;
  bool opened_ = false;
  std::atomic<bool> running_{true};
  std::mutex mutex_;
  std::condition_variable cv_;
  std::multimap<clock_type::time_point, int> pending_;
  std::vector<clock_type::duration> delays_{300us};
  size_t delay_index_ = 0;
  std::array<std::atomic<int>, 4> disables_{};
  std::thread receiver_;
  std::thread sender_;
};

hardware_interface::HardwareInfo hardwareInfo(
  const std::string & config_text, const std::vector<std::string> & joints)
{
  static int serial = 0;
  const std::string path =
    ::testing::TempDir() + "vcan_stream_" + std::to_string(serial++) + ".yaml";
  std::ofstream(path) << config_text;
  hardware_interface::HardwareInfo info;
  info.name = "test_bxi_stream";
  info.type = "system";
  info.hardware_parameters["config_file"] = path;
  info.hardware_parameters["spec_dir"] = std::string(BXI_TEST_SOURCE_DIR) + "/config/specs";
  for (const auto & name : joints) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.type = "joint";
    info.joints.push_back(joint);
  }
  return info;
}

std::string jointYaml(const std::string & name, const std::string & bus, int id)
{
  return "  " + name + ": {enabled: true, can_bus: " + bus + ", can_id: " + std::to_string(id) +
         ", motor_dir: 1, motor_type: MOTOR_70, lower: -1.0, upper: 1.0}\n";
}

std::string streamConfig(
  const std::string & bus, int can_hz, double max_age_ms, bool broadcast, int motors)
{
  std::string text = "active_can_buses: [" + bus + "]\n"
    "can_frame_format: {" + bus + ": fd}\n"
    "defaults: {control_timeout_ms: 5, maximum_consecutive_timeouts: 3, can_hz: " +
    std::to_string(can_hz) + "}\n"
    "can_transport:\n  " + bus + ": {rx_mode: separate, tx_frame: " +
    (broadcast ? "broadcast" : "unicast") + ", stream_feedback_max_age_ms: " +
    std::to_string(max_age_ms) + "}\n"
    "joints:\n";
  for (int id = 1; id <= motors; ++id) {
    text += jointYaml("joint" + std::to_string(id), bus, id);
  }
  return text;
}

std::vector<std::string> jointNames(int motors)
{
  std::vector<std::string> names;
  for (int id = 1; id <= motors; ++id) {
    names.push_back("joint" + std::to_string(id));
  }
  return names;
}

hardware_interface::return_type readOnce(BxiSystemInterface & system)
{
  return system.read(rclcpp::Time(), rclcpp::Duration(0, 0));
}

// Polls read() until it reports an error, as controller_manager would observe a safe-stop.
bool waitForReadError(BxiSystemInterface & system, clock_type::duration timeout)
{
  const auto deadline = clock_type::now() + timeout;
  while (clock_type::now() < deadline) {
    if (readOnce(system) == hardware_interface::return_type::ERROR) {
      return true;
    }
    std::this_thread::sleep_for(200us);
  }
  return false;
}

struct Rate
{
  int can_hz;
  double max_age_ms;
};

class StreamedSystem : public ::testing::TestWithParam<Rate>
{
protected:
  void SetUp() override
  {
    iface_ = std::getenv("BXI_VCAN_IFACE");
    if (iface_ == nullptr) {
      GTEST_SKIP() << "BXI_VCAN_IFACE is not configured";
    }
  }
  const char * iface_ = nullptr;
};

TEST_P(StreamedSystem, RunsAndDeactivatesWithAcknowledgedDisables)
{
  const auto rate = GetParam();
  MotorEmulator motors(iface_, {1, 2, 3});
  ASSERT_TRUE(motors.opened());
  BxiSystemInterface system;
  ASSERT_EQ(system.on_init(hardwareInfo(
      streamConfig(iface_, rate.can_hz, rate.max_age_ms, true, 3), jointNames(3))),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.write(rclcpp::Time(), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::OK);
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(readOnce(system), hardware_interface::return_type::OK);
  const int frames = motors.broadcast_frames.load();
  EXPECT_GE(frames, rate.can_hz * 3 / 10 * 8 / 10);
  EXPECT_EQ(motors.last_broadcast_length.load(), 24);

  EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  for (int id = 1; id <= 3; ++id) {
    EXPECT_GE(motors.disables(id), 1) << id;
  }
  const int frames_after_stop = motors.broadcast_frames.load();
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(motors.broadcast_frames.load(), frames_after_stop);
}

TEST_P(StreamedSystem, OneDroppedReplyDoesNotStop)
{
  const auto rate = GetParam();
  MotorEmulator motors(iface_, {1, 2, 3});
  ASSERT_TRUE(motors.opened());
  BxiSystemInterface system;
  ASSERT_EQ(system.on_init(hardwareInfo(
      streamConfig(iface_, rate.can_hz, rate.max_age_ms, true, 3), jointNames(3))),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  std::this_thread::sleep_for(50ms);
  motors.drop_next = 1;
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(readOnce(system), hardware_interface::return_type::OK);
  EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
}

TEST_P(StreamedSystem, SilentMotorSafeStopsAndIsDisabledWithoutDeactivate)
{
  const auto rate = GetParam();
  MotorEmulator motors(iface_, {1, 2, 3});
  ASSERT_TRUE(motors.opened());
  BxiSystemInterface system;
  ASSERT_EQ(system.on_init(hardwareInfo(
      streamConfig(iface_, rate.can_hz, rate.max_age_ms, true, 3), jointNames(3))),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  std::this_thread::sleep_for(50ms);

  motors.silent_mask = 0b100;  // motor 3 stops answering the broadcast
  const auto silenced_at = clock_type::now();
  ASSERT_TRUE(waitForReadError(system, 500ms));
  const double detect_ms =
    std::chrono::duration<double, std::milli>(clock_type::now() - silenced_at).count();
  EXPECT_LE(detect_ms, rate.max_age_ms + 1000.0 / rate.can_hz + 2.0);

  // Option 1: the monitor disables the bus without on_deactivate. Motors 1 and 2 are
  // acknowledged; motor 3 gets the best-effort disable latched with its safe-stop.
  const auto deadline = clock_type::now() + 500ms;
  while (clock_type::now() < deadline &&
    !(motors.disables(1) > 0 && motors.disables(2) > 0 && motors.disables(3) > 0))
  {
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_GE(motors.disables(1), 1);
  EXPECT_GE(motors.disables(2), 1);
  EXPECT_GE(motors.disables(3), 1);
  const int frames = motors.broadcast_frames.load();
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(motors.broadcast_frames.load(), frames);
  EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
}

TEST_P(StreamedSystem, NoStreamedReplyFailsActivationAndDisablesEveryMotor)
{
  const auto rate = GetParam();
  MotorEmulator motors(iface_, {1, 2, 3});
  ASSERT_TRUE(motors.opened());
  motors.answer_broadcast = false;
  BxiSystemInterface system;
  ASSERT_EQ(system.on_init(hardwareInfo(
      streamConfig(iface_, rate.can_hz, rate.max_age_ms, true, 3), jointNames(3))),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::ERROR);
  for (int id = 1; id <= 3; ++id) {
    EXPECT_GE(motors.disables(id), 1) << id;
  }
}

// A stream that faults after its first replies were confirmed, but before on_activate() returns,
// must fail activation rather than report an already stopped system as active.
class FaultAfterFeedbackConfirmed : public BxiSystemInterface
{
public:
  MotorEmulator * motors = nullptr;

protected:
  void onStreamFeedbackConfirmed() override
  {
    motors->overheat = true;
    // Many cycles: the send thread applies an overheated reply and stops the stream.
    std::this_thread::sleep_for(30ms);
  }
};

TEST_P(StreamedSystem, FaultAfterFirstFeedbackFailsActivation)
{
  const auto rate = GetParam();
  MotorEmulator motors(iface_, {1, 2, 3});
  ASSERT_TRUE(motors.opened());
  FaultAfterFeedbackConfirmed system;
  system.motors = &motors;
  ASSERT_EQ(system.on_init(hardwareInfo(
      streamConfig(iface_, rate.can_hz, rate.max_age_ms, true, 3), jointNames(3))),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::ERROR);
  for (int id = 1; id <= 3; ++id) {
    EXPECT_GE(motors.disables(id), 1) << id;
  }
  const int frames = motors.broadcast_frames.load();
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(motors.broadcast_frames.load(), frames);
}

INSTANTIATE_TEST_SUITE_P(
  Rates, StreamedSystem, ::testing::Values(Rate{200, 15.0}, Rate{1000, 3.0}),
  [](const ::testing::TestParamInfo<Rate> & info) {
    return std::to_string(info.param.can_hz) + "Hz";
  });

TEST(StreamedSystemAt1kHz, RepliesCrossingTheCycleBoundaryDoNotStop)
{
  const char * iface = std::getenv("BXI_VCAN_IFACE");
  if (iface == nullptr) {
    GTEST_SKIP() << "BXI_VCAN_IFACE is not configured";
  }
  MotorEmulator motors(iface, {1, 2, 3});
  ASSERT_TRUE(motors.opened());
  motors.setDelays({900us, 1100us, 1000us, 950us, 1050us});
  BxiSystemInterface system;
  ASSERT_EQ(system.on_init(hardwareInfo(streamConfig(iface, 1000, 3.0, true, 3), jointNames(3))),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(readOnce(system), hardware_interface::return_type::OK);
  EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
}

TEST(StreamedSystemAt1kHz, UnicastStreamRunsAndDeactivates)
{
  const char * iface = std::getenv("BXI_VCAN_IFACE");
  if (iface == nullptr) {
    GTEST_SKIP() << "BXI_VCAN_IFACE is not configured";
  }
  MotorEmulator motors(iface, {1, 2});
  ASSERT_TRUE(motors.opened());
  BxiSystemInterface system;
  ASSERT_EQ(system.on_init(hardwareInfo(streamConfig(iface, 1000, 3.0, false, 2), jointNames(2))),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(readOnce(system), hardware_interface::return_type::OK);
  EXPECT_EQ(motors.broadcast_frames.load(), 0);
  EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_GE(motors.disables(1), 1);
  EXPECT_GE(motors.disables(2), 1);
}

// A fault on one streamed bus stops and disables the other streamed bus as well.
TEST(StreamedSystemTwoBuses, FaultOnOneBusDisablesTheOther)
{
  const char * iface_a = std::getenv("BXI_VCAN_IFACE");
  const char * iface_b = std::getenv("BXI_VCAN_IFACE_B");
  if (iface_a == nullptr || iface_b == nullptr) {
    GTEST_SKIP() << "BXI_VCAN_IFACE and BXI_VCAN_IFACE_B are not configured";
  }
  MotorEmulator motors_a(iface_a, {1});
  MotorEmulator motors_b(iface_b, {1});
  ASSERT_TRUE(motors_a.opened());
  ASSERT_TRUE(motors_b.opened());
  const std::string a = iface_a;
  const std::string b = iface_b;
  const std::string config =
    "active_can_buses: [" + a + ", " + b + "]\n"
    "can_frame_format: {" + a + ": fd, " + b + ": fd}\n"
    "defaults: {control_timeout_ms: 5, maximum_consecutive_timeouts: 3, can_hz: 1000}\n"
    "can_transport:\n"
    "  " + a + ": {rx_mode: separate, tx_frame: broadcast, stream_feedback_max_age_ms: 3}\n"
    "  " + b + ": {rx_mode: separate, tx_frame: broadcast, stream_feedback_max_age_ms: 3}\n"
    "joints:\n" + jointYaml("joint1", a, 1) + jointYaml("joint2", b, 1);
  BxiSystemInterface system;
  ASSERT_EQ(system.on_init(hardwareInfo(config, {"joint1", "joint2"})),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  std::this_thread::sleep_for(50ms);
  motors_a.silent_mask = 0b1;
  ASSERT_TRUE(waitForReadError(system, 500ms));
  const auto deadline = clock_type::now() + 500ms;
  while (clock_type::now() < deadline && !(motors_a.disables(1) > 0 && motors_b.disables(1) > 0)) {
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_GE(motors_a.disables(1), 1);
  EXPECT_GE(motors_b.disables(1), 1);
  const int frames_b = motors_b.broadcast_frames.load();
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(motors_b.broadcast_frames.load(), frames_b);
  EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
}

}  // namespace
}  // namespace bxi_hardware
