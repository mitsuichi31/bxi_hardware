#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>
#include <thread>

#include <linux/can.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include "bxi_hardware/actuator/bxi_actuator.hpp"
#include "bxi_hardware/bus_streamer.hpp"
#include "bxi_hardware/can/socketcan_bus.hpp"

namespace bxi_hardware
{
namespace
{

// Stops and joins the emulator thread on every exit path. A failed ASSERT returns early, and a
// still-joinable std::thread would otherwise call std::terminate and lose the test result.
class EmulatorThread
{
public:
  template<typename Fn>
  explicit EmulatorThread(Fn fn)
  : thread_([this, fn]() {
        while (running_) {
          fn();
        }
      }) {}
  ~EmulatorThread() {stop();}
  void stop()
  {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  std::atomic<bool> running_{true};
  std::thread thread_;
};

CanFrame makeFeedback(uint8_t motor_id)
{
  CanFrame frame;
  frame.id = protocol::defaultMasterId(motor_id);
  frame.extended = false;
  frame.dlc = 8;
  frame.data = {{motor_id, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 40, 41}};
  return frame;
}

TEST(VcanLifecycle, ActivateCommandFeedbackDeactivateAndTimeoutSafeStop)
{
  const char * interface_name = std::getenv("BXI_VCAN_IFACE");
  if (interface_name == nullptr) {
    GTEST_SKIP() << "BXI_VCAN_IFACE is not configured";
  }

  SocketCanBus driver_bus;
  SocketCanBus emulator_bus;
  ASSERT_TRUE(driver_bus.open(interface_name));
  ASSERT_TRUE(emulator_bus.open(interface_name));
  std::atomic<bool> respond{true};
  EmulatorThread emulator([&]() {
      CanFrame request;
      if (emulator_bus.recv(request, 10) && request.id == 1 && request.dlc == 8 && respond) {
        emulator_bus.send(makeFeedback(1));
      }
    });

  BxiSpec spec;
  spec.t_min = -80.0;
  spec.t_max = 80.0;
  BxiActuatorDriver driver(&driver_bus, 1, 1, 0.0, -1.0, 1.0, 20.0, 1.0, spec, 30, 2);
  EXPECT_TRUE(driver.enable().valid);
  EXPECT_TRUE(driver.sendControl(0.1, 0.0, 20.0, 1.0, 0.0).valid);
  EXPECT_TRUE(driver.disable().valid);

  EXPECT_TRUE(driver.enable().valid);
  respond = false;
  EXPECT_FALSE(driver.sendControl(0.0, 0.0, 0.0, 0.0, 0.0).valid);
  EXPECT_FALSE(driver.sendControl(0.0, 0.0, 0.0, 0.0, 0.0).valid);
  EXPECT_EQ(driver.state(), ActuatorState::kSafeStopped);

  emulator.stop();
  driver_bus.close();
  emulator_bus.close();
}

// The motor answers in the format it received, so the emulator mirrors the request format.
TEST(VcanLifecycle, FdBrsBusSendsAndReceivesFdFrames)
{
  const char * interface_name = std::getenv("BXI_VCAN_IFACE");
  if (interface_name == nullptr) {
    GTEST_SKIP() << "BXI_VCAN_IFACE is not configured";
  }

  SocketCanBus driver_bus(CanFrameFormat::kFdBrs);
  driver_bus.setRxTimestamping(true);
  SocketCanBus emulator_bus(CanFrameFormat::kFdBrs);
  ASSERT_TRUE(driver_bus.open(interface_name)) << "vcan needs 'mtu 72' for CAN FD";
  ASSERT_TRUE(emulator_bus.open(interface_name));
  std::atomic<int> fd_brs_requests{0};
  EmulatorThread emulator([&]() {
      CanFrame request;
      if (emulator_bus.recv(request, 10) && request.id == 1 && request.dlc == 8) {
        if (request.fd && request.brs) {
          ++fd_brs_requests;
        }
        emulator_bus.send(makeFeedback(1));
      }
    });

  BxiSpec spec;
  BxiActuatorDriver driver(&driver_bus, 1, 1, 0.0, -1.0, 1.0, 20.0, 1.0, spec, 30, 2);
  EXPECT_TRUE(driver.enable().valid);
  const auto control_feedback = driver.sendControl(0.1, 0.0, 20.0, 1.0, 0.0);
  ASSERT_TRUE(control_feedback.valid);
  ASSERT_GT(control_feedback.rx_kernel_realtime_ns, 0);
  EXPECT_GE(control_feedback.rx_userspace_realtime_ns, control_feedback.rx_kernel_realtime_ns);
  EXPECT_TRUE(driver.disable().valid);
  EXPECT_EQ(fd_brs_requests.load(), 3);

  // A classic bus still reads an FD reply instead of dropping it. Stop the emulator first so that
  // it does not hold the emulator socket while the reply is sent.
  emulator.stop();
  SocketCanBus classic_bus;
  ASSERT_TRUE(classic_bus.open(interface_name));
  ASSERT_TRUE(emulator_bus.send(makeFeedback(1)));
  CanFrame reply;
  bool received = false;
  for (int i = 0; i < 10 && !received; ++i) {
    received = classic_bus.recv(reply, 100) && reply.id == protocol::defaultMasterId(1);
  }
  ASSERT_TRUE(received);
  EXPECT_TRUE(reply.fd);
  EXPECT_TRUE(reply.brs);
  EXPECT_EQ(reply.dlc, 8U);

  classic_bus.close();
  driver_bus.close();
  emulator_bus.close();
}

TEST(VcanLifecycle, FdBusRefusesAnInterfaceWithoutFd)
{
  const char * interface_name = std::getenv("BXI_VCAN_CLASSIC_IFACE");
  if (interface_name == nullptr) {
    GTEST_SKIP() << "BXI_VCAN_CLASSIC_IFACE is not configured";
  }
  SocketCanBus bus(CanFrameFormat::kFd);
  EXPECT_FALSE(bus.open(interface_name));
}

// Receives until a frame with the given ID arrives. Other frames on the shared vcan are ignored.
bool receiveId(SocketCanBus & bus, uint32_t id, CanFrame & out)
{
  for (int i = 0; i < 20; ++i) {
    if (bus.recv(out, 50) && !out.extended && out.id == id) {
      return true;
    }
  }
  return false;
}

CanFrame makePatternFrame(uint32_t id, uint8_t length)
{
  CanFrame frame;
  frame.id = id;
  frame.extended = false;
  // dlc may exceed the array to test rejection; only the bytes that exist are filled.
  frame.dlc = length;
  for (size_t i = 0; i < std::min<size_t>(length, frame.data.size()); ++i) {
    frame.data[i] = static_cast<uint8_t>(0xA0 + i);
  }
  return frame;
}

TEST(VcanLifecycle, FdBusSendsAndReceivesPayloadsUpTo64Bytes)
{
  const char * interface_name = std::getenv("BXI_VCAN_IFACE");
  if (interface_name == nullptr) {
    GTEST_SKIP() << "BXI_VCAN_IFACE is not configured";
  }
  SocketCanBus sender(CanFrameFormat::kFd);
  SocketCanBus receiver(CanFrameFormat::kFd);
  ASSERT_TRUE(sender.open(interface_name)) << "vcan needs 'mtu 72' for CAN FD";
  ASSERT_TRUE(receiver.open(interface_name));

  for (const uint8_t length : {8, 24, 64}) {
    const auto frame = makePatternFrame(0x7FF, length);
    ASSERT_TRUE(sender.send(frame)) << static_cast<int>(length);
    CanFrame received;
    ASSERT_TRUE(receiveId(receiver, 0x7FF, received)) << static_cast<int>(length);
    EXPECT_TRUE(received.fd);
    EXPECT_FALSE(received.brs);
    ASSERT_EQ(received.dlc, length);
    for (size_t i = 0; i < length; ++i) {
      EXPECT_EQ(received.data[i], frame.data[i]) << "byte " << i;
    }
  }

  // Lengths that SocketCAN would silently pad are refused instead.
  for (const uint8_t length : {9, 13, 25, 65}) {
    EXPECT_FALSE(sender.send(makePatternFrame(0x7FF, length))) << static_cast<int>(length);
  }
}

TEST(VcanLifecycle, ClassicBusRefusesPayloadsLongerThanEightBytes)
{
  const char * interface_name = std::getenv("BXI_VCAN_IFACE");
  if (interface_name == nullptr) {
    GTEST_SKIP() << "BXI_VCAN_IFACE is not configured";
  }
  SocketCanBus bus;
  ASSERT_TRUE(bus.open(interface_name));
  EXPECT_TRUE(bus.send(makePatternFrame(0x123, 8)));
  EXPECT_FALSE(bus.send(makePatternFrame(0x123, 12)));
  EXPECT_FALSE(bus.send(makePatternFrame(0x123, 24)));
}

// CanFrame has no RTR flag, so remote frames are written through a plain raw socket.
bool sendRemoteFrame(const char * interface_name, uint32_t id)
{
  const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    return false;
  }
  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = static_cast<int>(::if_nametoindex(interface_name));
  struct can_frame frame{};
  frame.can_id = id | CAN_RTR_FLAG;
  frame.len = 8;
  const bool ok = addr.can_ifindex != 0 &&
    ::bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) == 0 &&
    ::write(fd, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
  ::close(fd);
  return ok;
}

TEST(VcanLifecycle, StandardIdFilterPassesOnlyListedReplies)
{
  const char * interface_name = std::getenv("BXI_VCAN_IFACE");
  if (interface_name == nullptr) {
    GTEST_SKIP() << "BXI_VCAN_IFACE is not configured";
  }
  SocketCanBus sender(CanFrameFormat::kFd);
  SocketCanBus receiver(CanFrameFormat::kFd);
  ASSERT_TRUE(sender.open(interface_name));
  ASSERT_TRUE(receiver.open(interface_name));
  EXPECT_FALSE(receiver.setReceiveStandardIds({}));
  EXPECT_FALSE(receiver.setReceiveStandardIds({0x800}));
  ASSERT_TRUE(receiver.setReceiveStandardIds({protocol::defaultMasterId(1)}));

  // The broadcast command, another motor's reply, and extended-ID or remote frames with the
  // listed ID must not reach the filtered socket.
  ASSERT_TRUE(sender.send(makePatternFrame(0x7FF, 24)));
  ASSERT_TRUE(sender.send(makeFeedback(2)));
  CanFrame extended = makeFeedback(1);
  extended.extended = true;
  ASSERT_TRUE(sender.send(extended));
  ASSERT_TRUE(sendRemoteFrame(interface_name, protocol::defaultMasterId(1)));
  ASSERT_TRUE(sender.send(makeFeedback(1)));

  CanFrame received;
  ASSERT_TRUE(receiver.recv(received, 100));
  EXPECT_EQ(received.id, protocol::defaultMasterId(1));
  EXPECT_FALSE(received.extended);
  EXPECT_EQ(received.dlc, 8U);
  EXPECT_FALSE(receiver.recv(received, 50));

  // Restoring the accept-all filter lets other IDs through again. recv() skips a remote frame
  // instead of returning it as an all-zero data frame.
  ASSERT_TRUE(receiver.setReceiveEnabled(true));
  ASSERT_TRUE(sendRemoteFrame(interface_name, protocol::defaultMasterId(2)));
  ASSERT_TRUE(sender.send(makeFeedback(2)));
  ASSERT_TRUE(receiveId(receiver, protocol::defaultMasterId(2), received));
  EXPECT_EQ(received.dlc, 8U);
  EXPECT_EQ(received.data[0], 2U);
}

// The streamed path on real SocketCAN sockets: a send socket with receive disabled, a receive
// socket filtered to the reply IDs, and three emulated motors answering the 24-byte broadcast.
TEST(VcanLifecycle, StreamerRunsThreeBroadcastMotorsAt1kHzAndStopsOnASilentMotor)
{
  const char * interface_name = std::getenv("BXI_VCAN_IFACE");
  if (interface_name == nullptr) {
    GTEST_SKIP() << "BXI_VCAN_IFACE is not configured";
  }
  SocketCanBus tx_bus(CanFrameFormat::kFd);
  SocketCanBus rx_bus(CanFrameFormat::kFd);
  SocketCanBus emulator_bus(CanFrameFormat::kFd);
  ASSERT_TRUE(tx_bus.open(interface_name)) << "vcan needs 'mtu 72' for CAN FD";
  ASSERT_TRUE(rx_bus.open(interface_name));
  ASSERT_TRUE(emulator_bus.open(interface_name));

  std::atomic<int> responding_mask{0b111};  // bit k: motor k + 1 answers
  EmulatorThread emulator([&]() {
      CanFrame request;
      if (!emulator_bus.recv(request, 10) || request.extended) {
        return;
      }
      if (request.id >= 1 && request.id <= 3 && request.dlc == 8) {
        emulator_bus.send(makeFeedback(static_cast<uint8_t>(request.id)));
      } else if (request.id == 0x7FF && request.dlc == 24) {
        for (uint8_t motor = 1; motor <= 3; ++motor) {
          if ((responding_mask.load() & (1 << (motor - 1))) != 0) {
            emulator_bus.send(makeFeedback(motor));
          }
        }
      }
    });

  BxiSpec spec;
  std::vector<std::unique_ptr<BxiActuatorDriver>> drivers;
  std::vector<BxiActuatorDriver *> raw;
  for (int id = 1; id <= 3; ++id) {
    drivers.push_back(std::make_unique<BxiActuatorDriver>(
      &tx_bus, id, 1, 0.0, -1.0, 1.0, 20.0, 1.0, spec, 30, 3));
    ASSERT_TRUE(drivers.back()->enable().valid) << id;
    raw.push_back(drivers.back().get());
  }

  // Switch to the streamed setup, as the hardware interface will after its sequential phase.
  ASSERT_TRUE(rx_bus.setReceiveStandardIds({protocol::defaultMasterId(1),
      protocol::defaultMasterId(2), protocol::defaultMasterId(3)}));
  rx_bus.flushRx();
  ASSERT_TRUE(tx_bus.setReceiveEnabled(false));
  tx_bus.flushRx();

  StreamConfig config;
  config.period = std::chrono::milliseconds(1);
  config.max_feedback_age = std::chrono::milliseconds(3);
  config.broadcast = true;
  std::atomic<bool> stop_all{false};
  BxiBusStreamer streamer(&tx_bus, &rx_bus, raw, config,
    [](const std::vector<StreamFeedback> &, std::vector<StreamCommand> &) {return true;},
    &stop_all);
  ASSERT_TRUE(streamer.start()) << streamer.error();
  EXPECT_TRUE(streamer.waitForFirstFeedback(std::chrono::milliseconds(4)));
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  EXPECT_FALSE(stop_all.load());

  responding_mask = 0b011;  // motor 3 goes silent
  for (int i = 0; i < 100 && !stop_all.load(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_TRUE(stop_all.load());
  streamer.stop();
  const auto stats = streamer.stats();
  EXPECT_EQ(stats.fault, StreamFault::kFeedbackStale);
  EXPECT_EQ(stats.fault_axis, 2);
  EXPECT_GE(stats.cycles, 400U);
  EXPECT_EQ(stats.frames_sent, stats.cycles);
  EXPECT_GE(stats.replies[0] + 1, stats.cycles);
  EXPECT_GE(stats.replies[1] + 1, stats.cycles);
  EXPECT_EQ(stats.rx_ignored, 0U);  // the filter kept the 0x7FF commands out

  // Back to the sequential path: acknowledged disable for motors 1 and 2, best effort for 3.
  ASSERT_TRUE(tx_bus.setReceiveEnabled(true));
  tx_bus.flushRx();
  EXPECT_TRUE(drivers[0]->disable().valid);
  EXPECT_TRUE(drivers[1]->disable().valid);
  EXPECT_EQ(drivers[2]->state(), ActuatorState::kSafeStopped);
  EXPECT_FALSE(drivers[2]->disable().valid);
  emulator.stop();
}

TEST(VcanLifecycle, OptInKernelReceiveTimestampPrecedesUserspaceRead)
{
  const char * interface_name = std::getenv("BXI_VCAN_IFACE");
  if (interface_name == nullptr) {
    GTEST_SKIP() << "BXI_VCAN_IFACE is not configured";
  }

  SocketCanBus receiver;
  receiver.setRxTimestamping(true);
  SocketCanBus sender;
  ASSERT_TRUE(receiver.open(interface_name));
  ASSERT_TRUE(sender.open(interface_name));
  ASSERT_TRUE(sender.send(makeFeedback(1)));

  CanFrame reply;
  ASSERT_TRUE(receiver.recv(reply, 100));
  ASSERT_GT(reply.rx_kernel_realtime_ns, 0);
  EXPECT_GE(reply.rx_userspace_realtime_ns, reply.rx_kernel_realtime_ns);
  EXPECT_LT(reply.rx_userspace_realtime_ns - reply.rx_kernel_realtime_ns, 1000000000LL);
}

}  // namespace
}  // namespace bxi_hardware
