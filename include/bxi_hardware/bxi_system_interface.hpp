// ros2_control HWIF層: state/command interface の export と worker スレッド管理。
// mevius2_hardware/RobstrideSystemInterface を雛形に BXI 用に改修。
#ifndef BXI_HARDWARE__BXI_SYSTEM_INTERFACE_HPP_
#define BXI_HARDWARE__BXI_SYSTEM_INTERFACE_HPP_

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "bxi_hardware/actuator/bxi_actuator.hpp"
#include "bxi_hardware/bus_streamer.hpp"
#include "bxi_hardware/can/socketcan_bus.hpp"

namespace bxi_hardware
{

struct JointRuntime
{
  std::string name;
  bool enabled = false;
  std::string can_bus;
  int can_id = 0;
  int motor_dir = 1;
  double offset_angle = 0.0;
  std::optional<double> offset_thre;
  double default_kp = 50.0;
  double default_kd = 2.0;
  double lower = 0.0;
  double upper = 0.0;
  std::string motor_type;
  std::unique_ptr<ActuatorDriver> actuator;
  bool overheated = false;
  bool temperature_sensor_fault = false;
  bool communication_fault = false;
};

class BxiSystemInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  virtual std::shared_ptr<CanBus> makeBus(CanFrameFormat format);
  // Test seam: called during on_activate() after every streamed bus has answered and the workers
  // run, just before the final safe-stop check.
  virtual void onStreamFeedbackConfirmed() {}
  // Test seam: called when a stopped stream starts its log and CSV output (reportStream()).
  virtual void onStreamReport(const std::string & /*bus_name*/) {}
  // Test seam: called at the end of stopStream(), after the bus's threads have joined.
  virtual void onStreamStopped(const std::string & /*bus_name*/) {}
  // Test seam: called in releaseResources() just before the workers and monitors are joined.
  virtual void onBeforeJoinWorkers() {}
  // Test seam: the statistics of a stopped stream, as reportStream() logs them.
  virtual void onStreamStats(const std::string & /*bus_name*/, const StreamStats & /*stats*/) {}

private:
  struct WorkerTimingCycle
  {
    uint64_t cycle = 0;
    int64_t start_lateness_ns = 0;
    int64_t interval_ns = 0;
    int64_t allocation_ns = 0;
    int64_t target_lock_wait_ns = 0;
    int64_t target_lock_hold_ns = 0;
    int64_t can_ns = 0;
    int64_t feedback_lock_wait_ns = 0;
    int64_t feedback_lock_hold_ns = 0;
    int64_t work_ns = 0;
    uint32_t transactions = 0;
    uint32_t invalid_responses = 0;
  };

  struct WorkerTimingTransaction
  {
    uint64_t cycle = 0;
    size_t joint_index = 0;
    int64_t duration_ns = 0;
    bool valid = false;
    int64_t tx_call_realtime_ns = 0;
    int64_t rx_kernel_realtime_ns = 0;
    int64_t rx_userspace_realtime_ns = 0;
  };

  struct WorkerTimingTrace
  {
    std::vector<WorkerTimingCycle> cycles;
    std::vector<WorkerTimingTransaction> transactions;
    size_t cycle_count = 0;
    size_t transaction_count = 0;
    uint64_t dropped_cycles = 0;
    uint64_t dropped_transactions = 0;
    int64_t period_ns = 0;
  };

  // Per-bus transport from can_transport. The defaults keep the sequential worker.
  struct BusTransport
  {
    bool separate = false;   // rx_mode: separate
    bool broadcast = false;  // tx_frame: broadcast
    double stream_feedback_max_age_ms = 0.0;
    // Safe-stop when write() has not run for this long (stalled control loop). Default: the
    // larger of 5 ms and two stream periods.
    double stream_command_max_age_ms = 0.0;
    size_t isolated_id1_broadcast_blocks = 0;  // hil_isolated_id1_broadcast_blocks
    int send_thread_priority = 0;     // SCHED_FIFO 1-98; 0 = SCHED_OTHER
    int receive_thread_priority = 0;  // SCHED_FIFO 1-98; 0 = SCHED_OTHER
    std::vector<int> thread_cpus;     // CPU affinity of both threads; empty keeps the default
  };

  bool parseConfig();
  bool startStream(const std::string & bus_name, const std::vector<size_t> & indices);
  // Runs on the send thread of a streamed bus once per cycle.
  bool streamCycle(
    const std::string & bus_name, const std::vector<size_t> & indices, const std::vector<StreamFeedback> & feedback,
    std::vector<StreamCommand> & commands);
  // Option 1 of the stop design: waits for the stream to end and, after a safe-stop, disables
  // the bus without waiting for controller_manager.
  void streamMonitorLoop(std::string bus_name, std::vector<size_t> indices);
  void stopStream(const std::string & bus_name);
  void reportStream(const std::string & bus_name);
  void writeStreamTiming(const std::string & bus_name, const BxiBusStreamer & streamer);
  void disableAxes(const std::string & bus_name, const std::vector<size_t> & indices,
    const char * reason);
  bool loadSpec(const std::string & motor_type);
  void workerLoop(std::string bus_name, std::vector<size_t> indices, WorkerTimingTrace * timing);
  void stopWorkers();
  void releaseResources();
  void writeWorkerTiming();

  size_t n_joints_ = 0;

  std::vector<double> st_pos_, st_vel_, st_eff_, st_temp_;
  std::vector<double> cmd_pos_, cmd_vel_, cmd_kp_, cmd_kd_, cmd_eff_;

  std::vector<JointRuntime> joints_;
  std::map<std::string, std::shared_ptr<CanBus>> buses_;
  std::vector<std::string> active_buses_;
  std::map<std::string, CanFrameFormat> bus_formats_;
  std::map<std::string, BusTransport> bus_transport_;
  std::map<std::string, std::shared_ptr<CanBus>> rx_buses_;
  std::map<std::string, std::unique_ptr<BxiBusStreamer>> streamers_;
  // steady_clock ns of the latest write(); read by every send thread (max_command_age).
  std::atomic<int64_t> last_write_ns_{0};
  std::mutex stream_stop_mutex_;
  std::set<std::string> reported_streams_;  // streams already logged and written
  std::map<std::string, BxiSpec> specs_;

  std::string config_file_;
  std::string spec_dir_;
  int can_hz_ = 200;  // 初期 200Hz、YAMLで上書き可
  int main_can_id_ = 254;
  int control_timeout_ms_ = 3;
  uint32_t maximum_consecutive_timeouts_ = 3;
  double temperature_sensor_fault_below_c_ =
    BxiActuatorDriver::kDefaultTemperatureSensorFaultBelowC;

  std::mutex data_mutex_;
  std::vector<double> tgt_pos_, tgt_vel_, tgt_kp_, tgt_kd_, tgt_eff_;
  std::vector<double> fb_pos_, fb_vel_, fb_eff_, fb_temp_;

  std::vector<std::thread> workers_;
  std::map<std::string, std::vector<size_t>> live_joints_;
  std::map<std::string, std::unique_ptr<WorkerTimingTrace>> worker_timing_;
  std::string worker_timing_dir_;
  std::atomic<bool> running_{false};
  std::atomic<bool> safe_stopped_{false};
};

}  // namespace bxi_hardware

#endif  // BXI_HARDWARE__BXI_SYSTEM_INTERFACE_HPP_
