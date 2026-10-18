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

std::optional<protocol::MitFeedback> decodeMotorReply(
  const CanFrame & frame, int can_id, const protocol::MitLimits & limits)
{
  if (frame.extended || frame.dlc != 8 ||
    frame.id != protocol::defaultMasterId(static_cast<uint32_t>(can_id)))
  {
    return std::nullopt;
  }
  return protocol::decodeFeedback(frame.data8(), limits, static_cast<uint8_t>(can_id));
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

ActuatorFeedback BxiActuatorDriver::transact(
  const CanFrame & request, TransactionObservation * observation)
{
  if (observation) {*observation = {};}
  ActuatorFeedback result;
  if (state_ == ActuatorState::kSafeStopped) {
    return result;
  }
  if (observation) {
    observation->tx_call_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    observation->tx_call_realtime_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    observation->result = "send_failed";
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
    const bool received = bus_->recv(response, static_cast<int>(remaining.count()));
    // This is also the original post-recv deadline check. Allocate observations only after
    // sampling it, so allocation cannot change whether this particular frame met the deadline.
    const auto dequeued_at = received ? clock::now() : clock::time_point{};
    if (observation && received) {
      observation->replies.push_back({std::chrono::duration_cast<std::chrono::nanoseconds>(
        dequeued_at.time_since_epoch()).count(), response.rx_kernel_realtime_ns, false,
        response.id, response.dlc});
    }
    if (!received || dequeued_at >= deadline) {
      break;
    }
    const auto decoded = decodeMotorReply(response, can_id_, limits());
    if (!decoded.has_value()) {
      continue;
    }
    consecutive_timeouts_ = 0;
    result = applyFeedback(*decoded);
    if (observation) {
      observation->replies.back().accepted = true;
      observation->result = std::isfinite(result.temperature) ? "ok" : "temperature_fault";
    }
    result.rx_kernel_realtime_ns = response.rx_kernel_realtime_ns;
    result.rx_userspace_realtime_ns = response.rx_userspace_realtime_ns;
    return result;
  }

  ++consecutive_timeouts_;
  if (observation) {
    observation->result = "timeout";
  }
  result.consecutive_timeouts = consecutive_timeouts_;
  if (consecutive_timeouts_ >= maximum_consecutive_timeouts_) {
    latchSafeStop();
    if (observation) {observation->result = "safe_stop";}
    RCLCPP_ERROR(
      logger(), "motor id=%d entered safe-stop after %u consecutive timeouts", can_id_,
      consecutive_timeouts_);
  }
  return result;
}

ActuatorFeedback convertFeedback(const protocol::MitFeedback & decoded,
  const FeedbackTransform & transform)
{
  ActuatorFeedback result;
  result.position = (decoded.position - transform.offset) * transform.direction;
  result.velocity = decoded.velocity * transform.direction;
  result.effort = decoded.torque * transform.direction;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double mos = decoded.mos_temperature < transform.sensor_fault_below ?
    nan : decoded.mos_temperature;
  const double motor = decoded.motor_temperature < transform.sensor_fault_below ?
    nan : decoded.motor_temperature;
  result.temperature = std::isnan(mos) ? motor :
    (std::isnan(motor) ? mos : std::max(mos, motor));
  result.motor_temperature = motor;
  result.valid = true;
  return result;
}

ActuatorFeedback BxiActuatorDriver::applyFeedback(const protocol::MitFeedback & decoded)
{
  auto result = convertFeedback(decoded, feedbackTransform());
  updateTemperature(decoded, result);  // Preserve sequential sensor-transition diagnostics.
  return result;
}

void BxiActuatorDriver::latchSafeStop(bool report_disable_failure)
{
  if (state_ == ActuatorState::kSafeStopped) {
    return;
  }
  state_ = ActuatorState::kSafeStopped;
  // A lost reply does not prove the request was lost. Try to clear the motor's latched command
  // without waiting for an acknowledgement, which may be the part of the link that has failed.
  trySendDisable(report_disable_failure);
}

void BxiActuatorDriver::trySendDisable(bool report_failure)
{
  CanFrame disable_frame;
  disable_frame.id = static_cast<uint32_t>(can_id_);
  disable_frame.extended = false;
  disable_frame.setData(protocol::encodeSpecialCommand(protocol::SpecialCommand::kDisable));
  if (!bus_->send(disable_frame) && report_failure) {
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
  protocol::SpecialCommand command, bool enables_motor, TransactionObservation * observation)
{
  if (state_ == ActuatorState::kSafeStopped && enables_motor) {
    if (observation) {*observation = {};}
    return {};
  }
  CanFrame frame;
  frame.id = static_cast<uint32_t>(can_id_);
  frame.extended = false;
  frame.setData(protocol::encodeSpecialCommand(command));
  auto feedback = transact(frame, observation);
  if (feedback.valid) {
    state_ = enables_motor ? ActuatorState::kEnabled : ActuatorState::kDisabled;
  }
  return feedback;
}

ActuatorFeedback BxiActuatorDriver::enable()
{
  return sendSpecial(protocol::SpecialCommand::kEnable, true);
}

ActuatorFeedback BxiActuatorDriver::enableObserved(TransactionObservation & observation)
{
  return sendSpecial(protocol::SpecialCommand::kEnable, true, &observation);
}

ActuatorFeedback BxiActuatorDriver::initialControlObserved(TransactionObservation & observation)
{
  observation = {};
  const auto encoded = encodeControlBlock(0, 0, 0, 0, 0);
  if (!encoded) {return {};}
  CanFrame frame;
  frame.id = static_cast<uint32_t>(can_id_);
  frame.extended = false;
  frame.setData(*encoded);
  return transact(frame, &observation);
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

std::optional<std::array<uint8_t, 8>> BxiActuatorDriver::encodeControlBlock(
  double p_ref, double v_ref, double kp, double kd, double tau_ff) const
{
  if (state_ != ActuatorState::kEnabled || !std::isfinite(p_ref) ||
    !std::isfinite(v_ref) || !std::isfinite(kp) || !std::isfinite(kd) ||
    !std::isfinite(tau_ff))
  {
    return std::nullopt;
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
  return protocol::encodeCommand(command, limits());
}

ActuatorFeedback BxiActuatorDriver::sendControl(
  double p_ref, double v_ref, double kp, double kd, double tau_ff)
{
  const auto encoded = encodeControlBlock(p_ref, v_ref, kp, kd, tau_ff);
  if (!encoded.has_value()) {
    return {};
  }

  CanFrame frame;
  frame.id = static_cast<uint32_t>(can_id_);
  frame.extended = false;
  frame.setData(encoded.value());
  return transact(frame);
}

}  // namespace bxi_hardware
