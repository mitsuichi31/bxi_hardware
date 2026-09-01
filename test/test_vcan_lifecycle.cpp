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

}  // namespace
}  // namespace bxi_hardware
