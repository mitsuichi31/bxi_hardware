#include "bxi_hardware/actuator/bxi_actuator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

namespace bxi_hardware
{
namespace
{

rclcpp::Logger logger() {return rclcpp::get_logger("BxiActuator");}

}  // namespace

BxiActuatorDriver::BxiActuatorDriver(
  CanBus * bus, int can_id, int motor_dir, double offset_angle, double lower, double upper,
  double default_kp, double default_kd, const BxiSpec & spec, int response_timeout_ms,
  uint32_t maximum_consecutive_timeouts)
: bus_(bus), can_id_(can_id), motor_dir_(motor_dir), offset_angle_(offset_angle), lower_(lower),
  upper_(upper), default_kp_(default_kp), default_kd_(default_kd), spec_(spec),
  response_timeout_ms_(response_timeout_ms),
  maximum_consecutive_timeouts_(maximum_consecutive_timeouts)
{
  std::string reason;
  if (bus_ == nullptr || can_id_ <= 0 || can_id_ > 0x7EF ||
    (motor_dir_ != -1 && motor_dir_ != 1) || response_timeout_ms_ <= 0 ||
    maximum_consecutive_timeouts_ == 0 || lower_ >= upper_ ||
    !protocol::validateLimits(limits(), &reason))
  {
    throw std::invalid_argument("invalid BXI actuator configuration: " + reason);
  }
}

protocol::MitLimits BxiActuatorDriver::limits() const
{
  protocol::MitLimits value;
  value.p_min = spec_.p_min;
  value.p_max = spec_.p_max;
  value.v_min = spec_.v_min;
  value.v_max = spec_.v_max;
  value.kp_max = spec_.kp_max;
  value.kd_max = spec_.kd_max;
  value.torque_min = spec_.t_min;
  value.torque_max = spec_.t_max;
  return value;
}

ActuatorFeedback BxiActuatorDriver::transact(const CanFrame & request)
{
  ActuatorFeedback result;
  if (state_ == ActuatorState::kSafeStopped || !bus_->send(request)) {
    state_ = ActuatorState::kSafeStopped;
    return result;
  }

  CanFrame response;
  while (bus_->recv(response, response_timeout_ms_)) {
    if (response.extended || response.dlc != 8 ||
      response.id != protocol::defaultMasterId(static_cast<uint32_t>(can_id_)))
    {
      continue;
    }
    const auto decoded = protocol::decodeFeedback(
      response.data, limits(), static_cast<uint8_t>(can_id_));
    if (!decoded.has_value()) {
      continue;
    }
    consecutive_timeouts_ = 0;
    result.position = (decoded->position - offset_angle_) * motor_dir_;
    result.velocity = decoded->velocity * motor_dir_;
    result.effort = decoded->torque * motor_dir_;
    result.temperature = std::max(decoded->mos_temperature, decoded->motor_temperature);
    result.motor_temperature = decoded->motor_temperature;
    result.valid = true;
    return result;
  }

  ++consecutive_timeouts_;
  result.consecutive_timeouts = consecutive_timeouts_;
  if (consecutive_timeouts_ >= maximum_consecutive_timeouts_) {
    state_ = ActuatorState::kSafeStopped;
    RCLCPP_ERROR(
      logger(), "motor id=%d entered safe-stop after %u consecutive timeouts", can_id_,
      consecutive_timeouts_);
  }
  return result;
}

ActuatorFeedback BxiActuatorDriver::sendSpecial(
  protocol::SpecialCommand command, bool enables_motor)
{
  if (state_ == ActuatorState::kSafeStopped && enables_motor) {
    return {};
  }
  CanFrame frame;
  frame.id = static_cast<uint32_t>(can_id_);
  frame.extended = false;
  frame.dlc = 8;
  frame.data = protocol::encodeSpecialCommand(command);
  auto feedback = transact(frame);
  if (feedback.valid) {
    state_ = enables_motor ? ActuatorState::kEnabled : ActuatorState::kDisabled;
  }
  return feedback;
}

ActuatorFeedback BxiActuatorDriver::enable()
{
  return sendSpecial(protocol::SpecialCommand::kEnable, true);
}

ActuatorFeedback BxiActuatorDriver::disable()
{
  // A disable frame is still allowed while safe-stopped. A missing disable acknowledgement does
  // not clear the latched safe state.
  const bool was_safe_stopped = state_ == ActuatorState::kSafeStopped;
  if (was_safe_stopped) {
    state_ = ActuatorState::kDisabled;
  }
  auto feedback = sendSpecial(protocol::SpecialCommand::kDisable, false);
  if (was_safe_stopped) {
    state_ = ActuatorState::kSafeStopped;
  }
  return feedback;
}

bool BxiActuatorDriver::setRunMode()
{
  // The acquired BXI MIT guide defines no separate run-mode frame. Successful enable is the
  // only documented transition into MIT command acceptance.
  return state_ == ActuatorState::kEnabled;
}

ActuatorFeedback BxiActuatorDriver::setZero()
{
  return sendSpecial(protocol::SpecialCommand::kSaveZero, false);
}

void BxiActuatorDriver::setAngleOffset(double offset)
{
  if (!std::isfinite(offset)) {
    throw std::invalid_argument("angle offset must be finite");
  }
  offset_angle_ = offset;
}

void BxiActuatorDriver::setAngleRange(double lower, double upper)
{
  if (!std::isfinite(lower) || !std::isfinite(upper) || lower >= upper) {
    throw std::invalid_argument("angle range must be finite and lower < upper");
  }
  lower_ = lower;
  upper_ = upper;
}

ActuatorFeedback BxiActuatorDriver::sendControl(
  double p_ref, double v_ref, double kp, double kd, double tau_ff)
{
  if (state_ != ActuatorState::kEnabled || !std::isfinite(p_ref) ||
    !std::isfinite(v_ref) || !std::isfinite(kp) || !std::isfinite(kd) ||
    !std::isfinite(tau_ff))
  {
    return {};
  }

  // Runtime commands are saturated at the configured physical and joint limits. The pure codec
  // remains strict and rejects all out-of-range input.
  protocol::MitCommand command;
  const double motor_position =
    (std::clamp(p_ref, lower_, upper_) * motor_dir_) + offset_angle_;
  command.position = std::clamp(motor_position, spec_.p_min, spec_.p_max);
  command.velocity = std::clamp(v_ref * motor_dir_, spec_.v_min, spec_.v_max);
  command.kp = std::clamp(kp, 0.0, spec_.kp_max);
  command.kd = std::clamp(kd, 0.0, spec_.kd_max);
  command.torque = std::clamp(tau_ff * motor_dir_, spec_.t_min, spec_.t_max);
  const auto encoded = protocol::encodeCommand(command, limits());
  if (!encoded.has_value()) {
    return {};
  }

  CanFrame frame;
  frame.id = static_cast<uint32_t>(can_id_);
  frame.extended = false;
  frame.dlc = 8;
  frame.data = encoded.value();
  return transact(frame);
}

}  // namespace bxi_hardware
