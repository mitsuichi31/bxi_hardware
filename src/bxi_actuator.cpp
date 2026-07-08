// Phase 0: NOOP スタブ実装。
// Phase 2 で BXI 詳細プロトコル仕様書確認後に MIT フレーム送受信を実装する。
#include "bxi_hardware/actuator/bxi_actuator.hpp"

#include <rclcpp/rclcpp.hpp>

namespace bxi_hardware
{

BxiActuatorDriver::BxiActuatorDriver(
  CanBus * bus,
  int can_id,
  int motor_dir,
  double offset_angle,
  double lower,
  double upper,
  double default_kp,
  double default_kd,
  const BxiSpec & spec)
: bus_(bus),
  can_id_(can_id),
  motor_dir_(motor_dir),
  offset_angle_(offset_angle),
  lower_(lower),
  upper_(upper),
  default_kp_(default_kp),
  default_kd_(default_kd),
  spec_(spec)
{
  (void)bus_;
  (void)default_kp_;
  (void)default_kd_;
  (void)spec_;
}

ActuatorFeedback BxiActuatorDriver::enable()
{
  RCLCPP_DEBUG(rclcpp::get_logger("BxiActuator"), "enable() NOOP id=%d", can_id_);
  ActuatorFeedback fb;
  fb.valid = true;
  return fb;
}

ActuatorFeedback BxiActuatorDriver::disable()
{
  RCLCPP_DEBUG(rclcpp::get_logger("BxiActuator"), "disable() NOOP id=%d", can_id_);
  ActuatorFeedback fb;
  fb.valid = true;
  return fb;
}

bool BxiActuatorDriver::setRunMode()
{
  return true;
}

ActuatorFeedback BxiActuatorDriver::setZero()
{
  ActuatorFeedback fb;
  fb.valid = true;
  return fb;
}

void BxiActuatorDriver::setAngleOffset(double offset)
{
  offset_angle_ = offset;
}

void BxiActuatorDriver::setAngleRange(double lower, double upper)
{
  lower_ = lower;
  upper_ = upper;
}

ActuatorFeedback BxiActuatorDriver::sendControl(
  double p_ref, double v_ref, double kp, double kd, double tau_ff)
{
  // Phase 0 NOOP: 指令を受け取るが何もしない。フィードバックはゼロで返す。
  (void)p_ref; (void)v_ref; (void)kp; (void)kd; (void)tau_ff;
  (void)motor_dir_; (void)offset_angle_; (void)lower_; (void)upper_;
  ActuatorFeedback fb;
  fb.valid = true;
  return fb;
}

}  // namespace bxi_hardware
