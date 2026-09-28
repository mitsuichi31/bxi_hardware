// BxiSystemInterface with rx_mode: separate on vcan, against emulated motors whose replies can
// be delayed, dropped, or silenced. Skipped unless BXI_VCAN_IFACE names a CAN FD vcan.
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <sys/resource.h>

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
  // steady_clock ticks when the first disable frame for this motor arrived, 0 if none yet.
  int64_t firstDisableSteadyNs(int id) const
  {
    return first_disable_steady_ns_[static_cast<size_t>(id)].load();
  }

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
          int64_t none = 0;
          first_disable_steady_ns_[static_cast<size_t>(id)].compare_exchange_strong(none,
            clock_type::now().time_since_epoch().count());
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
  std::array<std::atomic<int64_t>, 4> first_disable_steady_ns_{};
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

// With BXI_WORKER_TIMING_DIR set, stopping a streamed bus writes its cycle and reply CSVs once.
TEST(StreamedSystemAt1kHz, TimingDirectoryReceivesStreamCsvs)
{
  const char * iface = std::getenv("BXI_VCAN_IFACE");
  if (iface == nullptr) {
    GTEST_SKIP() << "BXI_VCAN_IFACE is not configured";
  }
  const auto dir = std::filesystem::path(::testing::TempDir()) / "bxi_stream_timing";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  ASSERT_EQ(setenv("BXI_WORKER_TIMING_DIR", dir.c_str(), 1), 0);
  struct UnsetTimingDir
  {
    ~UnsetTimingDir() {unsetenv("BXI_WORKER_TIMING_DIR");}
  } unset_timing_dir;

  MotorEmulator motors(iface, {1, 2, 3});
  ASSERT_TRUE(motors.opened());
  BxiSystemInterface system;
  ASSERT_EQ(system.on_init(hardwareInfo(streamConfig(iface, 1000, 3.0, true, 3), jointNames(3))),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  std::this_thread::sleep_for(200ms);
  ASSERT_EQ(system.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  std::vector<std::filesystem::path> cycle_files;
  std::vector<std::filesystem::path> rx_files;
  for (const auto & entry : std::filesystem::directory_iterator(dir)) {
    const auto name = entry.path().filename().string();
    if (name.rfind("bxi_stream_", 0) == 0) {
      const bool is_cycles = name.size() > 11 && name.substr(name.size() - 11) == "_cycles.csv";
      (is_cycles ? cycle_files : rx_files).push_back(entry.path());
    }
  }
  ASSERT_EQ(cycle_files.size(), 1U);
  ASSERT_EQ(rx_files.size(), 1U);

  std::ifstream cycles(cycle_files[0]);
  std::string metadata;
  std::string header;
  std::getline(cycles, metadata);
  std::getline(cycles, header);
  EXPECT_EQ(metadata.rfind("#period_ns=1000000,max_feedback_age_ns=3000000,broadcast=1,"
    "can_ids=1;2;3,", 0), 0U) << metadata;
  EXPECT_NE(metadata.find("tx_thread=SCHED_"), std::string::npos) << metadata;
  EXPECT_EQ(header, "cycle,outcome,wake_lateness_ns,hook_ns,send_lateness_ns,frames,"
    "age_ns_1,fresh_1,age_ns_2,fresh_2,age_ns_3,fresh_3");
  int cycle_rows = 0;
  int sent_rows = 0;
  for (std::string line; std::getline(cycles, line); ) {
    ++cycle_rows;
    sent_rows += line.find(",sent,") != std::string::npos ? 1 : 0;
  }
  EXPECT_GE(cycle_rows, 150);
  EXPECT_GE(sent_rows, 150);

  std::ifstream rx(rx_files[0]);
  std::getline(rx, metadata);
  std::getline(rx, header);
  EXPECT_EQ(header, "can_id,received_ns,rx_kernel_realtime_ns,rx_userspace_realtime_ns,"
    "rx_hardware_raw_ns");
  int rx_rows = 0;
  int kernel_stamped = 0;
  for (std::string line; std::getline(rx, line); ) {
    ++rx_rows;
    // Third field: the kernel software timestamp, present on vcan.
    const auto first = line.find(',');
    const auto second = line.find(',', first + 1);
    kernel_stamped += std::stoll(line.substr(second + 1)) > 0 ? 1 : 0;
  }
  EXPECT_GE(rx_rows, 3 * 150);
  EXPECT_EQ(kernel_stamped, rx_rows);
}

// On a safe-stop the monitor must send the disables before it logs and writes the CSVs, so that
// slow disk I/O cannot delay stopping the motors. File modification times are too coarse to
// order the two, so the report start is taken from a test seam on the same steady clock.
class RecordsReportStart : public BxiSystemInterface
{
public:
  std::atomic<int64_t> report_steady_ns{0};

protected:
  void onStreamReport(const std::string &) override
  {
    int64_t none = 0;
    report_steady_ns.compare_exchange_strong(none,
      clock_type::now().time_since_epoch().count());
  }
};

TEST(StreamedSystemAt1kHz, SafeStopDisablesBeforeReportingTheStream)
{
  const char * iface = std::getenv("BXI_VCAN_IFACE");
  if (iface == nullptr) {
    GTEST_SKIP() << "BXI_VCAN_IFACE is not configured";
  }
  const auto dir = std::filesystem::path(::testing::TempDir()) / "bxi_stream_safe_stop_timing";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  ASSERT_EQ(setenv("BXI_WORKER_TIMING_DIR", dir.c_str(), 1), 0);
  struct UnsetTimingDir
  {
    ~UnsetTimingDir() {unsetenv("BXI_WORKER_TIMING_DIR");}
  } unset_timing_dir;

  MotorEmulator motors(iface, {1, 2, 3});
  ASSERT_TRUE(motors.opened());
  RecordsReportStart system;
  ASSERT_EQ(system.on_init(hardwareInfo(streamConfig(iface, 1000, 3.0, true, 3), jointNames(3))),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  std::this_thread::sleep_for(50ms);
  motors.silent_mask = 0b100;

  // The monitor reports without on_deactivate; wait for it.
  const auto deadline = clock_type::now() + 1s;
  while (clock_type::now() < deadline && system.report_steady_ns.load() == 0) {
    std::this_thread::sleep_for(1ms);
  }
  const int64_t reported_ns = system.report_steady_ns.load();
  ASSERT_GT(reported_ns, 0);
  for (int id = 1; id <= 3; ++id) {
    const int64_t disabled_ns = motors.firstDisableSteadyNs(id);
    ASSERT_GT(disabled_ns, 0) << id;
    EXPECT_LT(disabled_ns, reported_ns) << "motor " << id << " was disabled after the report began";
  }
  EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_FALSE(std::filesystem::is_empty(dir));  // the CSVs were still written
}

// Thread settings from can_transport reach both stream threads (checked through the CSV header).
TEST(StreamedSystemAt1kHz, AppliesConfiguredThreadScheduling)
{
  const char * iface = std::getenv("BXI_VCAN_IFACE");
  if (iface == nullptr) {
    GTEST_SKIP() << "BXI_VCAN_IFACE is not configured";
  }
  const auto dir = std::filesystem::path(::testing::TempDir()) / "bxi_stream_rt_timing";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  ASSERT_EQ(setenv("BXI_WORKER_TIMING_DIR", dir.c_str(), 1), 0);
  struct UnsetTimingDir
  {
    ~UnsetTimingDir() {unsetenv("BXI_WORKER_TIMING_DIR");}
  } unset_timing_dir;

  MotorEmulator motors(iface, {1});
  ASSERT_TRUE(motors.opened());
  auto config = streamConfig(iface, 1000, 3.0, true, 1);
  config.replace(config.find("stream_feedback_max_age_ms"), 0,
    "send_thread_priority: 30, receive_thread_priority: 31, thread_cpus: [0, 1], ");
  BxiSystemInterface system;
  ASSERT_EQ(system.on_init(hardwareInfo(config, jointNames(1))),
    hardware_interface::CallbackReturn::SUCCESS);
  const auto activated = system.on_activate(rclcpp_lifecycle::State());
  if (activated != hardware_interface::CallbackReturn::SUCCESS) {
    GTEST_SKIP() << "activation failed; SCHED_FIFO may not be allowed here";
  }
  std::this_thread::sleep_for(20ms);
  ASSERT_EQ(system.on_deactivate(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);
  std::string metadata;
  for (const auto & entry : std::filesystem::directory_iterator(dir)) {
    const auto name = entry.path().filename().string();
    if (name.size() > 11 && name.substr(name.size() - 11) == "_cycles.csv") {
      std::ifstream(entry.path()) >> metadata;
    }
  }
  EXPECT_NE(metadata.find("tx_thread=SCHED_FIFO_priority=30_cpus=0-1"), std::string::npos)
    << metadata;
  EXPECT_NE(metadata.find("rx_thread=SCHED_FIFO_priority=31_cpus=0-1"), std::string::npos)
    << metadata;
}

// A priority the OS refuses fails activation and rolls back: every motor is disabled.
TEST(StreamedSystemAt1kHz, RefusedThreadPriorityFailsActivationAndDisables)
{
  const char * iface = std::getenv("BXI_VCAN_IFACE");
  if (iface == nullptr) {
    GTEST_SKIP() << "BXI_VCAN_IFACE is not configured";
  }
  MotorEmulator motors(iface, {1, 2, 3});
  ASSERT_TRUE(motors.opened());
  auto config = streamConfig(iface, 1000, 3.0, true, 3);
  config.replace(config.find("stream_feedback_max_age_ms"), 0, "send_thread_priority: 30, ");
  BxiSystemInterface system;
  ASSERT_EQ(system.on_init(hardwareInfo(config, jointNames(3))),
    hardware_interface::CallbackReturn::SUCCESS);
  rlimit saved{};
  ASSERT_EQ(::getrlimit(RLIMIT_RTPRIO, &saved), 0);
  rlimit none = saved;
  none.rlim_cur = 0;
  ASSERT_EQ(::setrlimit(RLIMIT_RTPRIO, &none), 0);
  const auto activated = system.on_activate(rclcpp_lifecycle::State());
  ::setrlimit(RLIMIT_RTPRIO, &saved);
  if (activated == hardware_interface::CallbackReturn::SUCCESS) {
    system.on_deactivate(rclcpp_lifecycle::State());
    GTEST_SKIP() << "this process may use SCHED_FIFO regardless of RLIMIT_RTPRIO";
  }
  for (int id = 1; id <= 3; ++id) {
    EXPECT_GE(motors.disables(id), 1) << id;
  }
  EXPECT_EQ(motors.broadcast_frames.load(), 0);
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
