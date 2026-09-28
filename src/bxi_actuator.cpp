#include "bxi_hardware/actuator/bxi_actuator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
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
  uint32_t maximum_consecutive_timeouts, double temperature_sensor_fault_below_c)
: bus_(bus), can_id_(can_id), motor_dir_(motor_dir), offset_angle_(offset_angle), lower_(lower),
  upper_(upper), default_kp_(default_kp), default_kd_(default_kd), spec_(spec),
  response_timeout_ms_(response_timeout_ms),
  maximum_consecutive_timeouts_(maximum_consecutive_timeouts),
  temperature_sensor_fault_below_c_(temperature_sensor_fault_below_c)
{
  std::string reason;
  if (bus_ == nullptr || can_id_ <= 0 ||
    can_id_ > static_cast<int>(protocol::kMaxSupportedMotorCanId) ||
    (motor_dir_ != -1 && motor_dir_ != 1) || response_timeout_ms_ <= 0 ||
    maximum_consecutive_timeouts_ == 0 || lower_ >= upper_ ||
    !std::isfinite(temperature_sensor_fault_below_c_) ||
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
  if (state_ == ActuatorState::kSafeStopped) {
    return result;
  }
  if (!bus_->send(request)) {
    latchSafeStop();
    return result;
  }

  using clock = std::chrono::steady_clock;
  const auto deadline = clock::now() + std::chrono::milliseconds(response_timeout_ms_);
  CanFrame response;
  while (true) {
    const auto now = clock::now();
    if (now >= deadline) {
      break;
    }
    const auto remaining = std::chrono::ceil<std::chrono::milliseconds>(deadline - now);
    if (!bus_->recv(response, static_cast<int>(remaining.count())) ||
      clock::now() >= deadline)
    {
      break;
    }
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
    updateTemperature(*decoded, result);
    result.valid = true;
    result.rx_kernel_realtime_ns = response.rx_kernel_realtime_ns;
    result.rx_userspace_realtime_ns = response.rx_userspace_realtime_ns;
    return result;
  }

  ++consecutive_timeouts_;
  result.consecutive_timeouts = consecutive_timeouts_;
  if (consecutive_timeouts_ >= maximum_consecutive_timeouts_) {
    latchSafeStop();
    RCLCPP_ERROR(
      logger(), "motor id=%d entered safe-stop after %u consecutive timeouts", can_id_,
      consecutive_timeouts_);
  }
  return result;
}

void BxiActuatorDriver::latchSafeStop()
{
  if (state_ == ActuatorState::kSafeStopped) {
    return;
  }
  state_ = ActuatorState::kSafeStopped;
  // A lost reply does not prove the request was lost. Try to clear the motor's latched command
  // without waiting for an acknowledgement, which may be the part of the link that has failed.
  trySendDisable();
}

void BxiActuatorDriver::trySendDisable()
{
  CanFrame disable_frame;
  disable_frame.id = static_cast<uint32_t>(can_id_);
  disable_frame.extended = false;
  disable_frame.dlc = 8;
  disable_frame.data = protocol::encodeSpecialCommand(protocol::SpecialCommand::kDisable);
  if (!bus_->send(disable_frame)) {
    RCLCPP_ERROR(logger(), "motor id=%d disable frame could not be sent", can_id_);
  }
}

void BxiActuatorDriver::updateTemperature(
  const protocol::MitFeedback & decoded, ActuatorFeedback & result)
{
  // Faulted sensors are excluded rather than stopping the actuator; the remaining sensor keeps
  // driving the over-temperature check. The warning is logged once per fault transition.
  const auto check = [&](double value, bool & fault, const char * sensor) {
      const bool now_fault = value < temperature_sensor_fault_below_c_;
      if (now_fault && !fault) {
        RCLCPP_WARN(
          logger(), "motor id=%d %s reads %.1f degC; treating it as a sensor fault and "
          "excluding it from the temperature check", can_id_, sensor, value);
      } else if (!now_fault && fault) {
        RCLCPP_INFO(logger(), "motor id=%d %s recovered (%.1f degC)", can_id_, sensor, value);
      }
      fault = now_fault;
      return now_fault ? std::numeric_limits<double>::quiet_NaN() : value;
    };
  const double mos = check(decoded.mos_temperature, mos_sensor_fault_, "NTC1 (byte 6)");
  const double motor = check(decoded.motor_temperature, motor_sensor_fault_, "NTC2 (byte 7)");
  if (std::isnan(mos)) {
    result.temperature = motor;
  } else if (std::isnan(motor)) {
    result.temperature = mos;
  } else {
    result.temperature = std::max(mos, motor);
  }
  result.motor_temperature = motor;
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
  // Keep the safe-stop latch intact while retrying disable during lifecycle cleanup.
  if (state_ == ActuatorState::kSafeStopped) {
    trySendDisable();
    return {};
  }
  return sendSpecial(protocol::SpecialCommand::kDisable, false);
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
