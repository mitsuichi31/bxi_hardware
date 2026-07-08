// BXI アクチュエータドライバ。
// Phase 0: NOOP スタブ実装（ビルド確認用）。
// Phase 2: BXI 詳細プロトコル仕様書確認後に MIT フレーム送受信を実装する。
#ifndef BXI_HARDWARE__ACTUATOR__BXI_ACTUATOR_HPP_
#define BXI_HARDWARE__ACTUATOR__BXI_ACTUATOR_HPP_

#include <string>

#include "bxi_hardware/actuator/actuator_driver.hpp"
#include "bxi_hardware/can/can_bus.hpp"

namespace bxi_hardware
{

// BXI モーターのパラメータ仕様（Phase 2 で .yaml から読み込む）。
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
    const BxiSpec & spec);

  ActuatorFeedback enable() override;
  ActuatorFeedback disable() override;
  bool setRunMode() override;
  ActuatorFeedback setZero() override;

  void setAngleOffset(double offset) override;
  void setAngleRange(double lower, double upper) override;

  ActuatorFeedback sendControl(
    double p_ref, double v_ref, double kp, double kd, double tau_ff) override;

private:
  CanBus * bus_;
  int can_id_;
  int motor_dir_;
  double offset_angle_;
  double lower_;
  double upper_;
  double default_kp_;
  double default_kd_;
  BxiSpec spec_;
};

}  // namespace bxi_hardware

#endif  // BXI_HARDWARE__ACTUATOR__BXI_ACTUATOR_HPP_
