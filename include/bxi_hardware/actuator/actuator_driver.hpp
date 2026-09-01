// 抽象化層: モータ機種・CAN実装に依存しないアクチュエータIF。
// mevius2_hardware から複製。シグネチャは変更しない（KHI統合IFルール）。
#ifndef BXI_HARDWARE__ACTUATOR__ACTUATOR_DRIVER_HPP_
#define BXI_HARDWARE__ACTUATOR__ACTUATOR_DRIVER_HPP_

#include <cstdint>

namespace bxi_hardware
{

struct ActuatorFeedback
{
  double position = 0.0;     // [rad]
  double velocity = 0.0;     // [rad/s]
  double effort = 0.0;       // [Nm]
  double temperature = 0.0;  // [degC]
  double motor_temperature = 0.0;  // [degC]
  uint32_t consecutive_timeouts = 0;
  bool valid = false;
};

class ActuatorDriver
{
public:
  virtual ~ActuatorDriver() = default;

  virtual ActuatorFeedback enable() = 0;
  virtual ActuatorFeedback disable() = 0;
  virtual bool setRunMode() = 0;
  virtual ActuatorFeedback setZero() = 0;

  virtual void setAngleOffset(double offset) = 0;
  virtual void setAngleRange(double lower, double upper) = 0;

  virtual ActuatorFeedback sendControl(
    double p_ref, double v_ref, double kp, double kd, double tau_ff) = 0;
};

}  // namespace bxi_hardware

#endif  // BXI_HARDWARE__ACTUATOR__ACTUATOR_DRIVER_HPP_
