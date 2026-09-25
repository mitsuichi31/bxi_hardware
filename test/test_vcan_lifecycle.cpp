#include <atomic>
#include <chrono>
#include <cstdlib>
#include <thread>

#include <gtest/gtest.h>

#include "bxi_hardware/actuator/bxi_actuator.hpp"
#include "bxi_hardware/can/socketcan_bus.hpp"

namespace bxi_hardware
{
namespace
{

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
  std::atomic<bool> running{true};
  std::thread emulator([&]() {
    while (running) {
      CanFrame request;
      if (emulator_bus.recv(request, 10) && request.id == 1 && request.dlc == 8 && respond) {
        emulator_bus.send(makeFeedback(1));
      }
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

  running = false;
  emulator.join();
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
  SocketCanBus emulator_bus(CanFrameFormat::kFdBrs);
  ASSERT_TRUE(driver_bus.open(interface_name)) << "vcan needs 'mtu 72' for CAN FD";
  ASSERT_TRUE(emulator_bus.open(interface_name));
  std::atomic<bool> running{true};
  std::atomic<int> fd_brs_requests{0};
  std::thread emulator([&]() {
    while (running) {
      CanFrame request;
      if (emulator_bus.recv(request, 10) && request.id == 1 && request.dlc == 8) {
        if (request.fd && request.brs) {
          ++fd_brs_requests;
        }
        emulator_bus.send(makeFeedback(1));
      }
    }
  });

  BxiSpec spec;
  BxiActuatorDriver driver(&driver_bus, 1, 1, 0.0, -1.0, 1.0, 20.0, 1.0, spec, 30, 2);
  EXPECT_TRUE(driver.enable().valid);
  EXPECT_TRUE(driver.sendControl(0.1, 0.0, 20.0, 1.0, 0.0).valid);
  EXPECT_TRUE(driver.disable().valid);
  EXPECT_EQ(fd_brs_requests.load(), 3);

  // A classic bus still reads an FD reply instead of dropping it.
  SocketCanBus classic_bus;
  ASSERT_TRUE(classic_bus.open(interface_name));
  ASSERT_TRUE(emulator_bus.send(makeFeedback(1)));
  CanFrame reply;
  ASSERT_TRUE(classic_bus.recv(reply, 100));
  EXPECT_TRUE(reply.fd);
  EXPECT_TRUE(reply.brs);
  EXPECT_EQ(reply.dlc, 8U);

  running = false;
  emulator.join();
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

}  // namespace
}  // namespace bxi_hardware
