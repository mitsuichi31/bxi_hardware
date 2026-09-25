#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "bxi_hardware/protocol/mit_protocol.hpp"

namespace protocol = bxi_hardware::protocol;

TEST(MitProtocol, KnownSpecialCommandFixtures)
{
  EXPECT_EQ(
    protocol::encodeSpecialCommand(protocol::SpecialCommand::kEnable),
    (std::array<uint8_t, 8>{{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}}));
  EXPECT_EQ(
    protocol::encodeSpecialCommand(protocol::SpecialCommand::kDisable),
    (std::array<uint8_t, 8>{{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}}));
  EXPECT_EQ(
    protocol::encodeSpecialCommand(protocol::SpecialCommand::kSaveZero),
    (std::array<uint8_t, 8>{{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}}));
}

TEST(MitProtocol, KnownZeroCommandFixture)
{
  protocol::MitCommand command;
  const auto encoded = protocol::encodeCommand(command, protocol::MitLimits{});
  ASSERT_TRUE(encoded.has_value());
  EXPECT_EQ(
    encoded.value(),
    (std::array<uint8_t, 8>{{0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF}}));
}

TEST(MitProtocol, RejectsNonFiniteAndOutOfRangeCommands)
{
  protocol::MitCommand command;
  command.position = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(protocol::encodeCommand(command, protocol::MitLimits{}).has_value());
  command.position = 12.6;
  EXPECT_FALSE(protocol::encodeCommand(command, protocol::MitLimits{}).has_value());
}

TEST(MitProtocol, DecodesKnownFeedbackFixture)
{
  const protocol::MitLimits limits;
  // Reply captured from a BXI7010-19 on 2026-09-25 (NTC1 30.5 degC, NTC2 FAIL in the motor's
  // own readout).
  std::array<uint8_t, 8> data{{1, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 0x55, 0x02}};
  const auto feedback = protocol::decodeFeedback(data, limits, 1);
  ASSERT_TRUE(feedback.has_value());
  EXPECT_NEAR(feedback->position, 0.0, 0.001);
  EXPECT_NEAR(feedback->velocity, 0.0, 0.03);
  EXPECT_NEAR(feedback->torque, 0.0, 0.05);
  EXPECT_NEAR(feedback->mos_temperature, 30.0, 1e-9);
  EXPECT_NEAR(feedback->motor_temperature, -28.588, 0.001);
  EXPECT_FALSE(protocol::decodeFeedback(data, limits, 2).has_value());
}

TEST(MitProtocol, DecodesTemperatureOverTheFullByteRange)
{
  EXPECT_DOUBLE_EQ(protocol::decodeTemperature(0), -30.0);
  EXPECT_DOUBLE_EQ(protocol::decodeTemperature(255), 150.0);
  const protocol::MitLimits limits;
  const std::array<uint8_t, 8> hot{{1, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 0xFF, 0xFF}};
  const auto feedback = protocol::decodeFeedback(hot, limits, 1);
  ASSERT_TRUE(feedback.has_value());
  EXPECT_DOUBLE_EQ(feedback->mos_temperature, 150.0);
}

TEST(MitProtocol, PhysicalIntegerConversionIncludesEndpoints)
{
  EXPECT_EQ(protocol::floatToUint(-12.5, -12.5, 12.5, 16), 0U);
  EXPECT_EQ(protocol::floatToUint(12.5, -12.5, 12.5, 16), 65535U);
  EXPECT_DOUBLE_EQ(protocol::uintToFloat(0, -12.5, 12.5, 16), -12.5);
  EXPECT_DOUBLE_EQ(protocol::uintToFloat(65535, -12.5, 12.5, 16), 12.5);
}

TEST(MitProtocol, UsesStandardMasterIdRelationship)
{
  EXPECT_EQ(protocol::defaultMasterId(1), 0x11U);
  EXPECT_EQ(protocol::defaultMasterId(7), 0x17U);
}
