#include "bxi_hardware/protocol/mit_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace bxi_hardware::protocol
{
namespace
{

bool setReason(std::string * reason, const std::string & message)
{
  if (reason != nullptr) {
    *reason = message;
  }
  return false;
}

bool isFiniteAndInRange(double value, double minimum, double maximum)
{
  return std::isfinite(value) && value >= minimum && value <= maximum;
}

}  // namespace

bool validateLimits(const MitLimits & limits, std::string * reason)
{
  const std::array<std::pair<double, double>, 5> ranges{{
    {limits.p_min, limits.p_max},
    {limits.v_min, limits.v_max},
    {limits.kp_min, limits.kp_max},
    {limits.kd_min, limits.kd_max},
    {limits.torque_min, limits.torque_max},
  }};
  for (const auto & [minimum, maximum] : ranges) {
    if (!std::isfinite(minimum) || !std::isfinite(maximum) || minimum >= maximum) {
      return setReason(reason, "MIT range must be finite and minimum < maximum");
    }
  }
  return true;
}

std::optional<uint32_t> floatToUint(
  double value, double minimum, double maximum, unsigned int bits)
{
  if (!std::isfinite(value) || !std::isfinite(minimum) || !std::isfinite(maximum) ||
    minimum >= maximum || bits == 0U || bits > 31U || value < minimum || value > maximum)
  {
    return std::nullopt;
  }
  const auto scale = static_cast<double>((uint32_t{1} << bits) - 1U);
  return static_cast<uint32_t>((value - minimum) * scale / (maximum - minimum));
}

double uintToFloat(uint32_t value, double minimum, double maximum, unsigned int bits)
{
  if (!std::isfinite(minimum) || !std::isfinite(maximum) || minimum >= maximum ||
    bits == 0U || bits > 31U)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const uint32_t maximum_integer = (uint32_t{1} << bits) - 1U;
  const uint32_t saturated = std::min(value, maximum_integer);
  return static_cast<double>(saturated) * (maximum - minimum) /
         static_cast<double>(maximum_integer) + minimum;
}

std::optional<std::array<uint8_t, 8>> encodeCommand(
  const MitCommand & command, const MitLimits & limits, std::string * reason)
{
  if (!validateLimits(limits, reason)) {
    return std::nullopt;
  }
  if (!isFiniteAndInRange(command.position, limits.p_min, limits.p_max) ||
    !isFiniteAndInRange(command.velocity, limits.v_min, limits.v_max) ||
    !isFiniteAndInRange(command.kp, limits.kp_min, limits.kp_max) ||
    !isFiniteAndInRange(command.kd, limits.kd_min, limits.kd_max) ||
    !isFiniteAndInRange(command.torque, limits.torque_min, limits.torque_max))
  {
    setReason(reason, "MIT command is non-finite or outside the configured range");
    return std::nullopt;
  }

  const auto p = floatToUint(command.position, limits.p_min, limits.p_max, 16).value();
  const auto v = floatToUint(command.velocity, limits.v_min, limits.v_max, 12).value();
  const auto kp = floatToUint(command.kp, limits.kp_min, limits.kp_max, 12).value();
  const auto kd = floatToUint(command.kd, limits.kd_min, limits.kd_max, 12).value();
  const auto torque = floatToUint(
    command.torque, limits.torque_min, limits.torque_max, 12).value();

  return std::array<uint8_t, 8>{{
    static_cast<uint8_t>(p >> 8), static_cast<uint8_t>(p),
    static_cast<uint8_t>(v >> 4),
    static_cast<uint8_t>((v & 0xFU) << 4 | (kp >> 8)), static_cast<uint8_t>(kp),
    static_cast<uint8_t>(kd >> 4),
    static_cast<uint8_t>((kd & 0xFU) << 4 | (torque >> 8)), static_cast<uint8_t>(torque),
  }};
}

std::optional<MitFeedback> decodeFeedback(
  const std::array<uint8_t, 8> & data, const MitLimits & limits,
  std::optional<uint8_t> expected_motor_id, std::string * reason)
{
  if (!validateLimits(limits, reason)) {
    return std::nullopt;
  }
  if (expected_motor_id.has_value() && data[0] != expected_motor_id.value()) {
    setReason(reason, "feedback motor ID does not match the requested actuator");
    return std::nullopt;
  }
  if (data[6] > 150U || data[7] > 150U) {
    setReason(reason, "feedback temperature is outside the documented communication range");
    return std::nullopt;
  }

  const uint32_t p = static_cast<uint32_t>(data[1]) << 8 | data[2];
  const uint32_t v = static_cast<uint32_t>(data[3]) << 4 | (data[4] >> 4);
  const uint32_t torque = static_cast<uint32_t>(data[4] & 0xFU) << 8 | data[5];
  MitFeedback feedback;
  feedback.motor_id = data[0];
  feedback.position = uintToFloat(p, limits.p_min, limits.p_max, 16);
  feedback.velocity = uintToFloat(v, limits.v_min, limits.v_max, 12);
  feedback.torque = uintToFloat(torque, limits.torque_min, limits.torque_max, 12);
  feedback.mos_temperature = data[6];
  feedback.motor_temperature = data[7];
  return feedback;
}

std::array<uint8_t, 8> encodeSpecialCommand(SpecialCommand command)
{
  return {{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, static_cast<uint8_t>(command)}};
}

uint32_t defaultMasterId(uint32_t motor_can_id)
{
  return motor_can_id | 0x010U;
}

}  // namespace bxi_hardware::protocol
