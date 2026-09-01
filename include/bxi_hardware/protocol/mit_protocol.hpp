#ifndef BXI_HARDWARE__PROTOCOL__MIT_PROTOCOL_HPP_
#define BXI_HARDWARE__PROTOCOL__MIT_PROTOCOL_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace bxi_hardware::protocol
{

struct MitLimits
{
  double p_min{-12.5};
  double p_max{12.5};
  double v_min{-45.0};
  double v_max{45.0};
  double kp_min{0.0};
  double kp_max{500.0};
  double kd_min{0.0};
  double kd_max{5.0};
  double torque_min{-80.0};
  double torque_max{80.0};
};

struct MitCommand
{
  double position{0.0};
  double velocity{0.0};
  double kp{0.0};
  double kd{0.0};
  double torque{0.0};
};

struct MitFeedback
{
  uint8_t motor_id{0};
  double position{0.0};
  double velocity{0.0};
  double torque{0.0};
  double mos_temperature{0.0};
  double motor_temperature{0.0};
};

enum class SpecialCommand : uint8_t
{
  kDisableTerminalOutput = 0xFA,
  kEnableTerminalOutput = 0xFB,
  kEnable = 0xFC,
  kDisable = 0xFD,
  kSaveZero = 0xFE,
};

bool validateLimits(const MitLimits & limits, std::string * reason = nullptr);

std::optional<uint32_t> floatToUint(
  double value, double minimum, double maximum, unsigned int bits);
double uintToFloat(uint32_t value, double minimum, double maximum, unsigned int bits);

std::optional<std::array<uint8_t, 8>> encodeCommand(
  const MitCommand & command, const MitLimits & limits, std::string * reason = nullptr);
std::optional<MitFeedback> decodeFeedback(
  const std::array<uint8_t, 8> & data, const MitLimits & limits,
  std::optional<uint8_t> expected_motor_id = std::nullopt,
  std::string * reason = nullptr);

std::array<uint8_t, 8> encodeSpecialCommand(SpecialCommand command);
uint32_t defaultMasterId(uint32_t motor_can_id);

}  // namespace bxi_hardware::protocol

#endif  // BXI_HARDWARE__PROTOCOL__MIT_PROTOCOL_HPP_
