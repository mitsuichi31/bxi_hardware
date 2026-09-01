#include <algorithm>
#include <deque>
#include <limits>

#include <gtest/gtest.h>

#include "bxi_hardware/actuator/bxi_actuator.hpp"

namespace bxi_hardware
{
namespace
{

class EmulatedBus : public CanBus
{
public:
  bool open(const std::string & ifname) override {name_ = ifname; open_ = true; return true;}
  void close() override {open_ = false;}
  bool isOpen() const override {return open_;}
  const std::string & name() const override {return name_;}
  void flushRx() override {responses_.clear();}

  bool send(const CanFrame & frame) override
  {
    sent_.push_back(frame);
    if (respond_) {
      responses_.push_back(feedbackFrame(static_cast<uint8_t>(frame.id)));
    }
    return open_;
  }

  bool recv(CanFrame & out, int) override
  {
    if (responses_.empty()) {
      return false;
    }
    out = responses_.front();
    responses_.pop_front();
    return true;
  }

  static CanFrame feedbackFrame(uint8_t motor_id)
  {
    CanFrame frame;
    frame.id = protocol::defaultMasterId(motor_id);
    frame.extended = false;
    frame.dlc = 8;
    frame.data = {{motor_id, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 35, 36}};
    return frame;
  }

  bool respond_{true};
  std::vector<CanFrame> sent_;

private:
  bool open_{false};
  std::string name_;
  std::deque<CanFrame> responses_;
};

BxiActuatorDriver makeDriver(EmulatedBus & bus, uint32_t maximum_timeouts = 3)
{
  BxiSpec spec;
  spec.t_min = -80.0;
  spec.t_max = 80.0;
  return BxiActuatorDriver(&bus, 1, 1, 0.0, -1.0, 1.0, 20.0, 1.0, spec, 1,
    maximum_timeouts);
}

TEST(BxiActuator, ReproducesActivateCommandFeedbackDeactivateLifecycle)
{
  EmulatedBus bus;
  ASSERT_TRUE(bus.open("emulated0"));
  auto driver = makeDriver(bus);

  EXPECT_TRUE(driver.enable().valid);
  EXPECT_EQ(driver.state(), ActuatorState::kEnabled);
  EXPECT_TRUE(driver.setRunMode());
  const auto feedback = driver.sendControl(0.2, 0.1, 20.0, 1.0, 0.5);
  EXPECT_TRUE(feedback.valid);
  EXPECT_NEAR(feedback.position, 0.0, 0.001);
  EXPECT_DOUBLE_EQ(feedback.temperature, 36.0);
  EXPECT_TRUE(driver.disable().valid);
  EXPECT_EQ(driver.state(), ActuatorState::kDisabled);

  ASSERT_EQ(bus.sent_.size(), 3U);
  EXPECT_EQ(bus.sent_[0].data[7], 0xFC);
  EXPECT_EQ(bus.sent_[2].data[7], 0xFD);
  EXPECT_FALSE(bus.sent_[0].extended);
}

TEST(BxiActuator, SaturatesFiniteRuntimeCommandsBeforePacking)
{
  EmulatedBus bus;
  ASSERT_TRUE(bus.open("emulated0"));
  auto driver = makeDriver(bus);
  ASSERT_TRUE(driver.enable().valid);
  EXPECT_TRUE(driver.sendControl(100.0, 100.0, 1000.0, 100.0, 1000.0).valid);
  ASSERT_EQ(bus.sent_.size(), 2U);
  EXPECT_EQ(bus.sent_[1].data[0], 0x8A);  // joint upper=1.0 rad, not protocol p_max
}

TEST(BxiActuator, RejectsNanWithoutTransmitting)
{
  EmulatedBus bus;
  ASSERT_TRUE(bus.open("emulated0"));
  auto driver = makeDriver(bus);
  ASSERT_TRUE(driver.enable().valid);
  const size_t sends_before = bus.sent_.size();
  EXPECT_FALSE(driver.sendControl(
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0, 0.0).valid);
  EXPECT_EQ(bus.sent_.size(), sends_before);
}

TEST(BxiActuator, LatchesSafeStopAndStopsNewCommandsAfterTimeouts)
{
  EmulatedBus bus;
  ASSERT_TRUE(bus.open("emulated0"));
  auto driver = makeDriver(bus, 2);
  ASSERT_TRUE(driver.enable().valid);
  bus.respond_ = false;
  EXPECT_FALSE(driver.sendControl(0.0, 0.0, 0.0, 0.0, 0.0).valid);
  EXPECT_EQ(driver.state(), ActuatorState::kEnabled);
  EXPECT_FALSE(driver.sendControl(0.0, 0.0, 0.0, 0.0, 0.0).valid);
  EXPECT_EQ(driver.state(), ActuatorState::kSafeStopped);
  const size_t sends_at_stop = bus.sent_.size();
  EXPECT_FALSE(driver.sendControl(0.0, 0.0, 0.0, 0.0, 0.0).valid);
  EXPECT_EQ(bus.sent_.size(), sends_at_stop);
}

}  // namespace
}  // namespace bxi_hardware
