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
  std::array<uint8_t, 8> data{{1, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 42, 43}};
  const auto feedback = protocol::decodeFeedback(data, limits, 1);
  ASSERT_TRUE(feedback.has_value());
  EXPECT_NEAR(feedback->position, 0.0, 0.001);
  EXPECT_NEAR(feedback->velocity, 0.0, 0.03);
  EXPECT_NEAR(feedback->torque, 0.0, 0.05);
  EXPECT_DOUBLE_EQ(feedback->mos_temperature, 42.0);
  EXPECT_DOUBLE_EQ(feedback->motor_temperature, 43.0);
  EXPECT_FALSE(protocol::decodeFeedback(data, limits, 2).has_value());
  data[7] = 151;
  EXPECT_FALSE(protocol::decodeFeedback(data, limits, 1).has_value());
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
