// BXI MITアクチュエータドライバ。フレームcodecはprotocol層へ分離する。
#ifndef BXI_HARDWARE__ACTUATOR__BXI_ACTUATOR_HPP_
#define BXI_HARDWARE__ACTUATOR__BXI_ACTUATOR_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

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

// Stateless check of one received frame, safe to call from a receive thread: an 8-byte
// standard frame from the motor's reply ID whose payload names the same motor.
std::optional<protocol::MitFeedback> decodeMotorReply(
  const CanFrame & frame, int can_id, const protocol::MitLimits & limits);

struct FeedbackTransform
{
  int direction = 1;
  double offset = 0.0;
  double sensor_fault_below = -20.0;
};
// Pure conversion, shared by sequential and RX-owned streamed paths.
ActuatorFeedback convertFeedback(const protocol::MitFeedback & decoded,
  const FeedbackTransform & transform);

enum class ActuatorState
{
  kDisabled,
  kEnabled,
  kSafeStopped,
};

// Caller-owned, activation-only observations. No clocks are converted between domains.
struct TransactionObservation
{
  struct Reply
  {
    int64_t rx_dequeue_steady_ns = 0;
    int64_t rx_kernel_realtime_ns = 0;
    bool accepted = false;
    uint32_t can_id = 0;
    uint8_t dlc = 0;
  };
  int64_t tx_call_steady_ns = 0;
  int64_t tx_call_realtime_ns = 0;
  const char * result = "safe_stop";
  std::vector<Reply> replies;
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
  ActuatorFeedback enableObserved(TransactionObservation & observation);
  ActuatorFeedback initialControlObserved(TransactionObservation & observation);
  ActuatorFeedback disable() override;
  bool setRunMode() override;
  ActuatorFeedback setZero() override;

  void setAngleOffset(double offset) override;
  void setAngleRange(double lower, double upper) override;

  ActuatorFeedback sendControl(
    double p_ref, double v_ref, double kp, double kd, double tau_ff) override;

  // Helpers for the streamed transport, where sending and receiving run on separate threads.
  // None is thread-safe: only the thread that owns this driver (the send thread while streaming,
  // otherwise the lifecycle thread) may call them. The receive thread uses decodeMotorReply().
  // Clamps and encodes a command; nullopt unless enabled and every input is finite.
  std::optional<std::array<uint8_t, 8>> encodeControlBlock(
    double p_ref, double v_ref, double kp, double kd, double tau_ff) const;
  // Converts a decoded reply to joint units and updates the temperature-sensor fault flags.
  ActuatorFeedback applyFeedback(const protocol::MitFeedback & decoded);
  // Latches safe-stop after a streamed communication fault and sends a best-effort disable.
  void latchCommunicationFault(bool report_disable_failure = true)
  {latchSafeStop(report_disable_failure);}
  int canId() const {return can_id_;}
  // Snapshot after setAngleOffset/setAngleRange, before streaming starts.
  double effectiveAngleOffset() const {return offset_angle_;}
  std::pair<double, double> effectiveAngleRange() const {return {lower_, upper_};}
  int motorDirection() const {return motor_dir_;}
  FeedbackTransform feedbackTransform() const
  {return {motor_dir_, offset_angle_, temperature_sensor_fault_below_c_};}
  protocol::MitLimits limits() const;

  ActuatorState state() const {return state_;}
  uint32_t consecutiveTimeouts() const {return consecutive_timeouts_;}
  bool mosTemperatureSensorFault() const {return mos_sensor_fault_;}
  bool motorTemperatureSensorFault() const {return motor_sensor_fault_;}

private:
  ActuatorFeedback sendSpecial(protocol::SpecialCommand command, bool enables_motor,
    TransactionObservation * observation = nullptr);
  ActuatorFeedback transact(const CanFrame & request, TransactionObservation * observation = nullptr);
  void latchSafeStop(bool report_disable_failure = true);
  void trySendDisable(bool report_failure = true);
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
