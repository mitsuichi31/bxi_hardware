#include <array>
#include <cstdint>

#include <gtest/gtest.h>

#include "bxi_hardware/can/can_bus.hpp"

namespace bxi_hardware
{
namespace
{

TEST(CanFrame, CanFdDataLengthsFollowTheDlcTable)
{
  for (size_t length = 0; length <= 8; ++length) {
    EXPECT_TRUE(isValidCanFdDataLength(length)) << length;
  }
  for (const size_t length : {12, 16, 20, 24, 32, 48, 64}) {
    EXPECT_TRUE(isValidCanFdDataLength(length)) << length;
  }
  for (const size_t length : {9, 11, 13, 23, 25, 40, 63, 65}) {
    EXPECT_FALSE(isValidCanFdDataLength(length)) << length;
  }
}

TEST(CanFrame, SetDataCopiesThePayloadAndClearsTheRest)
{
  CanFrame frame;
  frame.data.fill(0xAA);
  std::array<uint8_t, 24> payload{};
  for (size_t i = 0; i < payload.size(); ++i) {
    payload[i] = static_cast<uint8_t>(i + 1);
  }
  frame.setData(payload);
  EXPECT_EQ(frame.dlc, 24U);
  for (size_t i = 0; i < payload.size(); ++i) {
    EXPECT_EQ(frame.data[i], payload[i]);
  }
  for (size_t i = payload.size(); i < frame.data.size(); ++i) {
    EXPECT_EQ(frame.data[i], 0U);
  }

  const std::array<uint8_t, 8> block{{1, 2, 3, 4, 5, 6, 7, 8}};
  frame.setData(block);
  EXPECT_EQ(frame.dlc, 8U);
  EXPECT_EQ(frame.data8(), block);
  EXPECT_EQ(frame.data[8], 0U);
}

}  // namespace
}  // namespace bxi_hardware
