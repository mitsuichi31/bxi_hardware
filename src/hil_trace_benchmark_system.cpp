// vcan-only HWIF subclass: fill synthetic vectors before enable, then use the real serializer
// from the actual monitor/lifecycle reportStream() caller. Not a CAN timing measurement.
#include "bxi_hardware/bxi_system_interface.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sched.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include "pluginlib/class_list_macros.hpp"

namespace bxi_hardware
{
namespace
{
using Clock = std::chrono::steady_clock;
int64_t nowNs()
{return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();}

class BenchmarkOutput : public StreamTraceOutput
{
public:
  BenchmarkOutput(const std::string & directory, const std::string & bus)
  : StreamTraceOutput(directory, bus), directory_(directory), trace_(prepareStreamTrace(620000, 1))
  {
    trace_->synthetic = true;
    trace_->cycle_count = 620000;
    trace_->rx_count = 1240000;
    for (size_t i = 0; i < trace_->cycles.size(); ++i) {
      auto & r = trace_->cycles[i];
      r.cycle = i; r.outcome = StreamCycleOutcome::kSent; r.frames = 1;
      r.command = {std::sin(i * .001) * .1, std::cos(i * .001) * .3,
        20., 1., std::sin(i * .002) * .01};
      r.command_valid = r.encoded_valid = r.send_attempted = r.send_success = true;
      const auto bytes = protocol::encodeCommand({r.command.position, r.command.velocity,
        r.command.kp, r.command.kd, r.command.effort}, protocol::MitLimits{});
      r.command_block = bytes.value();
    }
    for (size_t i = 0; i < trace_->rx.size(); ++i) {
      auto & r = trace_->rx[i];
      r.received_ns = i * 500000 + 5000;
      r.motor_position = std::sin(i * .0005) * .1;
      r.motor_velocity = std::cos(i * .0005) * .3;
      r.motor_torque = std::sin(i * .001) * .01;
    }
  }

  StreamTraceWriteResult write(const BxiBusStreamer & streamer, Clock::time_point start) override
  {
    const auto begin = nowNs();
    sched_param scheduling{};
    ::sched_getparam(0, &scheduling);
    cpu_set_t cpus;
    CPU_ZERO(&cpus); ::sched_getaffinity(0, sizeof(cpus), &cpus);
    std::string allowed;
    for (int i = 0; i < CPU_SETSIZE; ++i) {
      if (CPU_ISSET(i, &cpus)) {allowed += (allowed.empty() ? "" : ";") + std::to_string(i);}
    }
    const auto & real = streamer.trace();
    trace_->axis_metadata = real.axis_metadata;
    trace_->allocation_ns = real.allocation_ns;
    trace_->stream_stop_steady_ns = real.stream_stop_steady_ns;
    trace_->send_loop_start_steady_ns = real.stream_stop_steady_ns - 620000000000LL;
    trace_->origin = Clock::time_point(std::chrono::nanoseconds(trace_->send_loop_start_steady_ns));
    trace_->tx_thread = "synthetic_not_a_CAN_measurement";
    trace_->rx_thread = trace_->tx_thread;
    for (auto & row : trace_->cycles) {
      row.send_before_steady_ns = trace_->send_loop_start_steady_ns + row.cycle * 1000000 + 1000;
      row.send_after_steady_ns = row.send_before_steady_ns + 5000;
    }
    const auto stats = streamer.stats();
    if (stats.fault != StreamFault::kNone) {
      trace_->cycles.back().outcome = StreamCycleOutcome::kStopped;
      if (stats.fault == StreamFault::kCommandStale) {
        auto & row = trace_->cycles.back();
        row.command_stale = true;
        row.command.velocity = row.command.kp = row.command.kd = row.command.effort = 0;
        row.command_block = protocol::encodeCommand({row.command.position, 0, 0, 0, 0}, protocol::MitLimits{}).value();
      }
    }
    if (const char * delay = std::getenv("BXI_HIL_TRACE_DELAY_MS")) {
      const int value = std::stoi(delay);
      if (value != 0 && value != 2000) {throw std::runtime_error("test delay must be 0 or 2000");}
      std::this_thread::sleep_for(std::chrono::milliseconds(value));
    }
    auto config = streamer.config(); config.trace_extended = true; config.trace_cycles = 620000;
    const auto result = StreamTraceOutput::write(*trace_, config, stats,
      streamer.broadcastBlockCount(), streamer.paddingBlock(), start);
    std::ofstream marker(directory_ + "/benchmark_writer.json");
    marker << "{\"synthetic\":true,\"begin_steady_ns\":" << begin << ",\"end_steady_ns\":" << nowNs()
           << ",\"tid\":" << ::syscall(SYS_gettid) << ",\"policy\":" << ::sched_getscheduler(0)
           << ",\"priority\":" << scheduling.sched_priority << ",\"cpus\":\"" << allowed
           << "\",\"report_to_complete_ns\":" << result.report_to_complete_ns
           << ",\"cycle_rows\":" << result.cycle_rows << ",\"rx_rows\":" << result.rx_rows << "}\n";
    marker.close();
    if (!marker) {throw std::runtime_error("benchmark marker close failed");}
    return result;
  }
private:
  std::string directory_;
  std::unique_ptr<StreamTrace> trace_;
};
}  // namespace

class HilTraceBenchmarkSystem : public BxiSystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    const char * enabled = std::getenv("BXI_ALLOW_TEST_FAULTS");
    if (enabled == nullptr || std::string(enabled) != "1") {
      return hardware_interface::CallbackReturn::ERROR;
    }
    try {
      const auto yaml = YAML::LoadFile(info.hardware_parameters.at("config_file"));
      const auto buses = yaml["active_can_buses"];
      if (buses.size() != 1) {return hardware_interface::CallbackReturn::ERROR;}
      const auto bus = buses[0].as<std::string>();
      if (!std::regex_match(bus, std::regex("vcan[0-9]+")) ||
        !std::filesystem::exists("/sys/devices/virtual/net/" + bus)) {
        return hardware_interface::CallbackReturn::ERROR;
      }
      for (const auto & joint : yaml["joints"]) {
        if (joint.second["enabled"].as<bool>() && joint.second["can_bus"].as<std::string>() != bus) {
          return hardware_interface::CallbackReturn::ERROR;
        }
      }
    } catch (const std::exception &) {return hardware_interface::CallbackReturn::ERROR;}
    return BxiSystemInterface::on_init(info);
  }
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    const auto begin = nowNs();
    const auto result = BxiSystemInterface::on_deactivate(state);
    lifecycle("deactivate", begin);
    return result;
  }
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override
  {
    const auto begin = nowNs();
    const auto result = BxiSystemInterface::on_shutdown(state);
    lifecycle("shutdown", begin);
    return result;
  }
protected:
  std::unique_ptr<StreamTraceOutput> prepareTimingOutput(
    const std::string & directory, const std::string & bus) override
  {return std::make_unique<BenchmarkOutput>(directory, bus);}
private:
  void lifecycle(const char * kind, int64_t begin)
  {
    if (const char * dir = std::getenv("BXI_WORKER_TIMING_DIR")) {
      std::ofstream output(std::string(dir) + "/benchmark_lifecycle.jsonl", std::ios::app);
      output << "{\"callback\":\"" << kind << "\",\"begin_steady_ns\":" << begin
             << ",\"end_steady_ns\":" << nowNs() << "}\n";
    }
  }
};
}  // namespace bxi_hardware
PLUGINLIB_EXPORT_CLASS(bxi_hardware::HilTraceBenchmarkSystem, hardware_interface::SystemInterface)
