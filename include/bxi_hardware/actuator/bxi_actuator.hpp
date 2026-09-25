// BXI MITアクチュエータドライバ。フレームcodecはprotocol層へ分離する。
#ifndef BXI_HARDWARE__ACTUATOR__BXI_ACTUATOR_HPP_
#define BXI_HARDWARE__ACTUATOR__BXI_ACTUATOR_HPP_

#include <string>

#include "bxi_hardware/actuator/actuator_driver.hpp"
#include "bxi_hardware/can/can_bus.hpp"
#include "bxi_hardware/protocol/mit_protocol.hpp"

namespace bxi_hardware
{

// BXIモーターのパラメータ仕様（実プロトコル実装時に.yamlから読み込む）。
struct BxiSpec
{
  double p_min = -12.5;
  double p_max =  12.5;
  double v_min = -45.0;
  double v_max =  45.0;
  double kp_max = 500.0;
  double kd_max =   5.0;
  double t_min = -50.0;  // BXI7010-19 ピークトルク
  double t_max =  50.0;
};

enum class ActuatorState
{
  kDisabled,
  kEnabled,
  kSafeStopped,
};

class BxiActuatorDriver : public ActuatorDriver
{
public:
  BxiActuatorDriver(
    CanBus * bus,
    int can_id,
    int motor_dir,
    double offset_angle,
    double lower,
    double upper,
    double default_kp,
    double default_kd,
    const BxiSpec & spec,
    int response_timeout_ms = 3,
    uint32_t maximum_consecutive_timeouts = 3,
    double temperature_sensor_fault_below_c = kDefaultTemperatureSensorFaultBelowC);

  // A reading below this is treated as a disconnected or failed NTC (the raw value sits at the
  // bottom of the -30 degC range). Such a sensor is excluded from the reported temperature.
  static constexpr double kDefaultTemperatureSensorFaultBelowC = -20.0;

  ActuatorFeedback enable() override;
  ActuatorFeedback disable() override;
  bool setRunMode() override;
  ActuatorFeedback setZero() override;

  void setAngleOffset(double offset) override;
  void setAngleRange(double lower, double upper) override;

  ActuatorFeedback sendControl(
    double p_ref, double v_ref, double kp, double kd, double tau_ff) override;

  ActuatorState state() const {return state_;}
  uint32_t consecutiveTimeouts() const {return consecutive_timeouts_;}
  bool mosTemperatureSensorFault() const {return mos_sensor_fault_;}
  bool motorTemperatureSensorFault() const {return motor_sensor_fault_;}

private:
  ActuatorFeedback sendSpecial(protocol::SpecialCommand command, bool enables_motor);
  ActuatorFeedback transact(const CanFrame & request);
  protocol::MitLimits limits() const;
  void updateTemperature(const protocol::MitFeedback & decoded, ActuatorFeedback & result);

  CanBus * bus_;
  int can_id_;
  int motor_dir_;
  double offset_angle_;
  double lower_;
  double upper_;
  double default_kp_;
  double default_kd_;
  BxiSpec spec_;
  int response_timeout_ms_;
  uint32_t maximum_consecutive_timeouts_;
  double temperature_sensor_fault_below_c_;
  uint32_t consecutive_timeouts_{0};
  bool mos_sensor_fault_{false};
  bool motor_sensor_fault_{false};
  ActuatorState state_{ActuatorState::kDisabled};
};

}  // namespace bxi_hardware

#endif  // BXI_HARDWARE__ACTUATOR__BXI_ACTUATOR_HPP_
