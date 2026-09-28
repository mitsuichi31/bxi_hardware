// mevius2_hardware/RobstrideSystemInterface を雛形に BXI MIT protocol用へ改修。
#include "bxi_hardware/bxi_system_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <set>

#include <pthread.h>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bxi_hardware
{

static rclcpp::Logger logger() { return rclcpp::get_logger("BxiSystemInterface"); }
static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

static constexpr char HW_IF_KP[] = "kp";
static constexpr char HW_IF_KD[] = "kd";
static constexpr char HW_IF_TEMPERATURE[] = "temperature";

// ---------------------------------------------------------------------------
// on_init
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn BxiSystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto get_param = [&](const std::string & key, const std::string & def) -> std::string {
    auto it = info_.hardware_parameters.find(key);
    return (it != info_.hardware_parameters.end()) ? it->second : def;
  };
  config_file_ = get_param("config_file", "");
  spec_dir_ = get_param("spec_dir", "");
  if (config_file_.empty()) {
    RCLCPP_ERROR(logger(), "hardware param 'config_file' が指定されていません");
    return hardware_interface::CallbackReturn::ERROR;
  }

  n_joints_ = info_.joints.size();
  joints_.resize(n_joints_);

  st_pos_.assign(n_joints_, 0.0);
  st_vel_.assign(n_joints_, 0.0);
  st_eff_.assign(n_joints_, 0.0);
  st_temp_.assign(n_joints_, 0.0);
  cmd_pos_.assign(n_joints_, kNaN);
  cmd_vel_.assign(n_joints_, kNaN);
  cmd_kp_.assign(n_joints_, kNaN);
  cmd_kd_.assign(n_joints_, kNaN);
  cmd_eff_.assign(n_joints_, kNaN);
  tgt_pos_.assign(n_joints_, 0.0);
  tgt_vel_.assign(n_joints_, 0.0);
  tgt_kp_.assign(n_joints_, 0.0);
  tgt_kd_.assign(n_joints_, 0.0);
  tgt_eff_.assign(n_joints_, 0.0);
  fb_pos_.assign(n_joints_, 0.0);
  fb_vel_.assign(n_joints_, 0.0);
  fb_eff_.assign(n_joints_, 0.0);
  fb_temp_.assign(n_joints_, 0.0);

  if (!parseConfig()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger(), "on_init 完了: %zu joints, active_can_buses=%zu",
              n_joints_, active_buses_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// parseConfig
// ---------------------------------------------------------------------------
bool BxiSystemInterface::parseConfig()
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(config_file_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "config_file 読込失敗 '%s': %s", config_file_.c_str(), e.what());
    return false;
  }

  if (root["active_can_buses"]) {
    active_buses_ = root["active_can_buses"].as<std::vector<std::string>>();
  }
  if (root["defaults"]) {
    auto d = root["defaults"];
    if (d["main_can_id"]) main_can_id_ = d["main_can_id"].as<int>();
    if (d["control_timeout_ms"]) control_timeout_ms_ = d["control_timeout_ms"].as<int>();
    if (d["maximum_consecutive_timeouts"]) {
      maximum_consecutive_timeouts_ = d["maximum_consecutive_timeouts"].as<uint32_t>();
    }
    if (d["can_hz"]) can_hz_ = d["can_hz"].as<int>();
    if (d["temperature_sensor_fault_below_c"]) {
      temperature_sensor_fault_below_c_ = d["temperature_sensor_fault_below_c"].as<double>();
    }
  }
  if (active_buses_.empty() || control_timeout_ms_ <= 0 ||
    maximum_consecutive_timeouts_ == 0 || can_hz_ <= 0 ||
    !std::isfinite(temperature_sensor_fault_below_c_))
  {
    RCLCPP_ERROR(logger(), "defaults/active_can_buses の値が不正です");
    return false;
  }
  const std::set<std::string> active_bus_set(active_buses_.begin(), active_buses_.end());
  if (active_bus_set.size() != active_buses_.size()) {
    RCLCPP_ERROR(logger(), "active_can_buses に重複があります");
    return false;
  }

  // Optional per-bus frame format. Buses not listed use classic CAN.
  for (const auto & bus : active_buses_) {
    bus_formats_[bus] = CanFrameFormat::kClassic;
  }
  if (root["can_frame_format"]) {
    for (const auto & entry : root["can_frame_format"]) {
      const auto bus = entry.first.as<std::string>();
      const auto value = entry.second.as<std::string>();
      if (active_bus_set.count(bus) == 0) {
        RCLCPP_ERROR(logger(), "can_frame_format: 未知またはinactiveなCAN bus '%s'", bus.c_str());
        return false;
      }
      if (value == "classic") {
        bus_formats_[bus] = CanFrameFormat::kClassic;
      } else if (value == "fd") {
        bus_formats_[bus] = CanFrameFormat::kFd;
      } else if (value == "fd_brs") {
        bus_formats_[bus] = CanFrameFormat::kFdBrs;
      } else {
        RCLCPP_ERROR(logger(), "can_frame_format.%s: classic / fd / fd_brs のいずれかが必要 ('%s')",
          bus.c_str(), value.c_str());
        return false;
      }
    }
  }

  YAML::Node jnodes = root["joints"];
  if (!jnodes) {
    RCLCPP_ERROR(logger(), "config_file に 'joints' がありません");
    return false;
  }

  std::set<std::pair<std::string, int>> bus_can_ids;
  for (size_t i = 0; i < n_joints_; ++i) {
    const std::string & name = info_.joints[i].name;
    JointRuntime & jr = joints_[i];
    jr.name = name;

    YAML::Node jn = jnodes[name];
    if (!jn) {
      RCLCPP_ERROR(logger(), "joint '%s' の設定が config_file にありません", name.c_str());
      return false;
    }
    jr.enabled = jn["enabled"] ? jn["enabled"].as<bool>() : false;
    jr.can_bus = jn["can_bus"] ? jn["can_bus"].as<std::string>() : "";
    jr.can_id = jn["can_id"] ? jn["can_id"].as<int>() : 0;
    jr.motor_dir = jn["motor_dir"] ? jn["motor_dir"].as<int>() : 1;
    jr.offset_angle = jn["offset_angle"] ? jn["offset_angle"].as<double>() : 0.0;
    if (jn["offset_thre"] && !jn["offset_thre"].IsNull()) {
      jr.offset_thre = jn["offset_thre"].as<double>();
    }
    jr.default_kp = jn["default_kp"] ? jn["default_kp"].as<double>() : 50.0;
    jr.default_kd = jn["default_kd"] ? jn["default_kd"].as<double>() : 2.0;
    jr.lower = jn["lower"] ? jn["lower"].as<double>() : -1.0e9;
    jr.upper = jn["upper"] ? jn["upper"].as<double>() : 1.0e9;
    jr.motor_type = jn["motor_type"] ? jn["motor_type"].as<std::string>() : "TBD";
    if (!jr.enabled) {
      continue;
    }
    if (active_bus_set.count(jr.can_bus) == 0) {
      RCLCPP_ERROR(logger(), "joint '%s': 未知またはinactiveなCAN bus '%s'", name.c_str(),
        jr.can_bus.c_str());
      return false;
    }
    if (jr.can_id <= 0 || jr.can_id > static_cast<int>(protocol::kMaxSupportedMotorCanId) ||
      !bus_can_ids.emplace(jr.can_bus, jr.can_id).second)
    {
      RCLCPP_ERROR(logger(), "joint '%s': CAN IDが範囲外またはbus内で重複", name.c_str());
      return false;
    }
    if (jr.motor_dir != -1 && jr.motor_dir != 1) {
      RCLCPP_ERROR(logger(), "joint '%s': motor_dirは-1または1が必要", name.c_str());
      return false;
    }
    if (!std::isfinite(jr.lower) || !std::isfinite(jr.upper) || jr.lower >= jr.upper) {
      RCLCPP_ERROR(logger(), "joint '%s': lower/upper limitが不正", name.c_str());
      return false;
    }
    if (jr.motor_type.empty() || jr.motor_type == "TBD") {
      RCLCPP_ERROR(logger(), "joint '%s': motor_type specが未確定", name.c_str());
      return false;
    }
  }
  return true;
}

// 実プロトコル実装時にBXI機種spec yamlから物理範囲を読み込む。
bool BxiSystemInterface::loadSpec(const std::string & motor_type)
{
  if (specs_.count(motor_type)) {
    return true;
  }
  const std::string path = spec_dir_ + "/" + motor_type + ".yaml";
  YAML::Node n;
  try {
    n = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "機種spec読込失敗 '%s': %s", path.c_str(), e.what());
    return false;
  }
  BxiSpec s;
  try {
    s.p_min = n["p_min"].as<double>();
    s.p_max = n["p_max"].as<double>();
    s.v_min = n["v_min"].as<double>();
    s.v_max = n["v_max"].as<double>();
    s.kp_max = n["kp_max"].as<double>();
    s.kd_max = n["kd_max"].as<double>();
    s.t_min = n["t_min"].as<double>();
    s.t_max = n["t_max"].as<double>();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "機種spec '%s' のキー不足: %s", path.c_str(), e.what());
    return false;
  }
  std::string reason;
  protocol::MitLimits limits;
  limits.p_min = s.p_min; limits.p_max = s.p_max;
  limits.v_min = s.v_min; limits.v_max = s.v_max;
  limits.kp_max = s.kp_max; limits.kd_max = s.kd_max;
  limits.torque_min = s.t_min; limits.torque_max = s.t_max;
  if (!protocol::validateLimits(limits, &reason)) {
    RCLCPP_ERROR(logger(), "機種spec '%s' のrange不正: %s", path.c_str(), reason.c_str());
    return false;
  }
  specs_[motor_type] = s;
  RCLCPP_INFO(logger(), "機種spec読込: %s", motor_type.c_str());
  return true;
}

// ---------------------------------------------------------------------------
// interface export
// ---------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
BxiSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> v;
  for (size_t i = 0; i < n_joints_; ++i) {
    const std::string & name = info_.joints[i].name;
    v.emplace_back(name, hardware_interface::HW_IF_POSITION, &st_pos_[i]);
    v.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &st_vel_[i]);
    v.emplace_back(name, hardware_interface::HW_IF_EFFORT, &st_eff_[i]);
    v.emplace_back(name, HW_IF_TEMPERATURE, &st_temp_[i]);
  }
  return v;
}

std::vector<hardware_interface::CommandInterface>
BxiSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> v;
  for (size_t i = 0; i < n_joints_; ++i) {
    const std::string & name = info_.joints[i].name;
    v.emplace_back(name, hardware_interface::HW_IF_POSITION, &cmd_pos_[i]);
    v.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &cmd_vel_[i]);
    v.emplace_back(name, HW_IF_KP, &cmd_kp_[i]);
    v.emplace_back(name, HW_IF_KD, &cmd_kd_[i]);
    v.emplace_back(name, hardware_interface::HW_IF_EFFORT, &cmd_eff_[i]);
  }
  return v;
}

// ---------------------------------------------------------------------------
// on_activate
// ---------------------------------------------------------------------------
std::shared_ptr<CanBus> BxiSystemInterface::makeBus(CanFrameFormat format)
{
  return std::make_shared<SocketCanBus>(format);
}

hardware_interface::CallbackReturn BxiSystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto is_active = [&](const std::string & b) {
    return std::find(active_buses_.begin(), active_buses_.end(), b) != active_buses_.end();
  };

  std::map<std::string, std::vector<size_t>> bus_joints;
  for (size_t i = 0; i < n_joints_; ++i) {
    joints_[i].overheated = false;
    joints_[i].temperature_sensor_fault = false;
    joints_[i].communication_fault = false;
    if (joints_[i].enabled && is_active(joints_[i].can_bus)) {
      bus_joints[joints_[i].can_bus].push_back(i);
    }
  }

  safe_stopped_ = false;
  if (bus_joints.empty()) {
    // 全関節disabledのprofileは、実機I/Oを行わない明示的な構成として許可する。
    RCLCPP_WARN(logger(),
                "有効な関節がありません（bxi_hardware.yaml の enabled を確認）。"
                "HW無しのインターフェースのみで起動します。");
    running_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  auto fail_activation = [&]() {
    releaseResources();
    return hardware_interface::CallbackReturn::ERROR;
  };
  try {
    const char * timing_dir = std::getenv("BXI_WORKER_TIMING_DIR");
    worker_timing_dir_ = timing_dir == nullptr ? "" : timing_dir;
    if (!worker_timing_dir_.empty() &&
      (!std::filesystem::is_directory(worker_timing_dir_) ||
      ::access(worker_timing_dir_.c_str(), W_OK) != 0))
    {
      RCLCPP_ERROR(logger(), "worker timing directory is missing or not writable: %s",
        worker_timing_dir_.c_str());
      return fail_activation();
    }

    for (auto & [bus_name, indices] : bus_joints) {
      auto bus = makeBus(bus_formats_[bus_name]);
      if (auto * socketcan = dynamic_cast<SocketCanBus *>(bus.get())) {
        socketcan->setRxTimestamping(!worker_timing_dir_.empty());
      }
      if (!bus->open(bus_name)) {
        RCLCPP_ERROR(logger(), "CANオープン失敗: %s", bus_name.c_str());
        return fail_activation();
      }
      buses_[bus_name] = bus;

      constexpr int kEnableRetries = 3;
      std::map<size_t, double> enable_pos;
      std::vector<size_t> live;

      for (size_t i : indices) {
        if (!loadSpec(joints_[i].motor_type)) {
          return fail_activation();
        }
        joints_[i].actuator = std::make_unique<BxiActuatorDriver>(
          bus.get(), joints_[i].can_id, joints_[i].motor_dir,
          joints_[i].offset_angle, joints_[i].lower, joints_[i].upper,
          joints_[i].default_kp, joints_[i].default_kd,
          specs_[joints_[i].motor_type], control_timeout_ms_, maximum_consecutive_timeouts_,
          temperature_sensor_fault_below_c_);

        ActuatorFeedback fb;
        for (int attempt = 0; attempt < kEnableRetries; ++attempt) {
          fb = joints_[i].actuator->enable();
          if (fb.valid) {
            break;
          }
          RCLCPP_WARN(logger(), "[%s] %s enable応答なし（試行 %d/%d）",
                      bus_name.c_str(), joints_[i].name.c_str(), attempt + 1, kEnableRetries);
        }
        if (!fb.valid) {
          RCLCPP_ERROR(logger(), "[%s] %s enable失敗。起動を中止します。",
                       bus_name.c_str(), joints_[i].name.c_str());
          return fail_activation();
        }
        if (!std::isfinite(fb.temperature)) {
          RCLCPP_ERROR(logger(), "[%s] %s 両温度センサー故障。起動を中止します。",
            bus_name.c_str(), joints_[i].name.c_str());
          joints_[i].temperature_sensor_fault = true;
          return fail_activation();
        }
        RCLCPP_INFO(logger(), "[%s] Enable %s pos=%.3f temp=%.1f",
                    bus_name.c_str(), joints_[i].name.c_str(),
                    fb.position, fb.temperature);
        enable_pos[i] = fb.position;
        if (!joints_[i].actuator->setRunMode()) {
          RCLCPP_ERROR(logger(), "[%s] %s run modeへの遷移失敗", bus_name.c_str(),
            joints_[i].name.c_str());
          return fail_activation();
        }
        live.push_back(i);
      }

      for (size_t i : live) {
        const double pos_orig = enable_pos[i] * joints_[i].motor_dir;
        double offset = 0.0;
        if (joints_[i].offset_thre.has_value() && pos_orig > joints_[i].offset_thre.value()) {
          offset = -2.0 * M_PI * joints_[i].motor_dir;
        }
        offset += joints_[i].offset_angle;
        joints_[i].actuator->setAngleOffset(offset);
        joints_[i].actuator->setAngleRange(joints_[i].lower, joints_[i].upper);
      }

      for (size_t i : live) {
        auto fb = joints_[i].actuator->sendControl(0, 0, 0, 0, 0);
        if (!fb.valid) {
          RCLCPP_ERROR(logger(), "[%s] %s 初期フィードバック取得失敗", bus_name.c_str(),
            joints_[i].name.c_str());
          return fail_activation();
        }
        if (!std::isfinite(fb.temperature)) {
          RCLCPP_ERROR(logger(), "[%s] %s 両温度センサー故障。起動を中止します。",
            bus_name.c_str(), joints_[i].name.c_str());
          joints_[i].temperature_sensor_fault = true;
          return fail_activation();
        }
        st_pos_[i] = fb_pos_[i] = tgt_pos_[i] = fb.position;
        st_vel_[i] = fb_vel_[i] = fb.velocity;
        st_eff_[i] = fb_eff_[i] = fb.effort;
        st_temp_[i] = fb_temp_[i] = fb.temperature;
        tgt_kp_[i] = tgt_kd_[i] = 0.0;
      }
      if (!live.empty()) {
        live_joints_[bus_name] = live;
      }
    }

    running_ = true;
    for (auto & [bus_name, live] : live_joints_) {
      WorkerTimingTrace * timing = nullptr;
      if (!worker_timing_dir_.empty()) {
        constexpr size_t kTimingCycleCapacity = 20000;
        auto trace = std::make_unique<WorkerTimingTrace>();
        trace->cycles.resize(kTimingCycleCapacity);
        trace->transactions.resize(kTimingCycleCapacity * live.size());
        trace->period_ns = static_cast<int64_t>(1e9 / can_hz_);
        timing = trace.get();
        worker_timing_.emplace(bus_name, std::move(trace));
      }
      workers_.emplace_back(&BxiSystemInterface::workerLoop, this, bus_name, live, timing);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "起動中に例外: %s", e.what());
    return fail_activation();
  }
  RCLCPP_INFO(logger(), "on_activate 完了: %zu バスで worker 起動", workers_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_deactivate
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn BxiSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  releaseResources();
  RCLCPP_INFO(logger(), "on_deactivate 完了");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void BxiSystemInterface::releaseResources()
{
  stopWorkers();
  for (auto & jr : joints_) {
    if (jr.actuator) {
      jr.actuator->disable();
    }
  }
  for (auto & [name, bus] : buses_) {
    bus->close();
  }
  writeWorkerTiming();
  buses_.clear();
  live_joints_.clear();
  for (auto & jr : joints_) {
    jr.actuator.reset();
  }
}

void BxiSystemInterface::stopWorkers()
{
  running_ = false;
  for (auto & t : workers_) {
    if (t.joinable()) {
      t.join();
    }
  }
  workers_.clear();
}

void BxiSystemInterface::writeWorkerTiming()
{
  // Called only after the workers have joined and every actuator has been disabled.
  const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  for (const auto & [bus_name, trace_ptr] : worker_timing_) {
    const auto & trace = *trace_ptr;
    std::string safe_bus_name = bus_name;
    for (char & ch : safe_bus_name) {
      if (!((ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'Z') ||
        (ch >= 'a' && ch <= 'z') || ch == '_' || ch == '-'))
      {
        ch = '_';
      }
    }
    const auto stem = std::filesystem::path(worker_timing_dir_) /
      ("bxi_worker_" + safe_bus_name + "_" + std::to_string(timestamp));
    const auto cycle_path = stem.string() + "_cycles.csv";
    const auto transaction_path = stem.string() + "_transactions.csv";

    std::ofstream cycle_file(cycle_path);
    if (!cycle_file) {
      RCLCPP_ERROR(logger(), "worker timing CSV を開けません: %s", cycle_path.c_str());
    } else {
      cycle_file << "#period_ns=" << trace.period_ns << ",dropped_cycles=" <<
        trace.dropped_cycles << ",dropped_transactions=" << trace.dropped_transactions << '\n';
      cycle_file << "cycle,start_lateness_ns,interval_ns,allocation_ns,"
        "target_lock_wait_ns,target_lock_hold_ns,can_ns,feedback_lock_wait_ns,"
        "feedback_lock_hold_ns,work_ns,transactions,invalid_responses\n";
      const auto count = std::min(trace.cycle_count, trace.cycles.size());
      for (size_t i = 0; i < count; ++i) {
        const auto & row = trace.cycles[i];
        cycle_file << row.cycle << ',' << row.start_lateness_ns << ',' << row.interval_ns << ',' <<
          row.allocation_ns << ',' << row.target_lock_wait_ns << ',' <<
          row.target_lock_hold_ns << ',' << row.can_ns << ',' << row.feedback_lock_wait_ns << ',' <<
          row.feedback_lock_hold_ns << ',' << row.work_ns << ',' << row.transactions << ',' <<
          row.invalid_responses << '\n';
      }
      cycle_file.flush();
      if (!cycle_file) {
        RCLCPP_ERROR(logger(), "worker timing CSV 書込失敗: %s", cycle_path.c_str());
      }
    }

    std::ofstream transaction_file(transaction_path);
    if (!transaction_file) {
      RCLCPP_ERROR(logger(), "worker timing CSV を開けません: %s", transaction_path.c_str());
    } else {
      transaction_file << "cycle,joint_index,duration_ns,valid,tx_call_realtime_ns,"
        "rx_kernel_realtime_ns,rx_userspace_realtime_ns\n";
      const auto count = std::min(trace.transaction_count, trace.transactions.size());
      for (size_t i = 0; i < count; ++i) {
        const auto & row = trace.transactions[i];
        transaction_file << row.cycle << ',' << row.joint_index << ',' << row.duration_ns << ',' <<
          (row.valid ? 1 : 0) << ',' << row.tx_call_realtime_ns << ',' <<
          row.rx_kernel_realtime_ns << ',' << row.rx_userspace_realtime_ns << '\n';
      }
      transaction_file.flush();
      if (!transaction_file) {
        RCLCPP_ERROR(logger(), "worker timing CSV 書込失敗: %s", transaction_path.c_str());
      }
    }
    RCLCPP_INFO(logger(), "[%s] worker timing: %zu cycles, %zu transactions, %llu/%llu dropped; %s",
      bus_name.c_str(), trace.cycle_count, trace.transaction_count,
      static_cast<unsigned long long>(trace.dropped_cycles),
      static_cast<unsigned long long>(trace.dropped_transactions), stem.string().c_str());
  }
  worker_timing_.clear();
  worker_timing_dir_.clear();
}

// ---------------------------------------------------------------------------
// workerLoop
// ---------------------------------------------------------------------------
void BxiSystemInterface::workerLoop(
  std::string bus_name, std::vector<size_t> indices, WorkerTimingTrace * timing)
{
  using clock = std::chrono::steady_clock;
  const auto elapsed_ns = [](clock::time_point start, clock::time_point end) {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    };
  const auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / can_hz_));
  clock::time_point previous_start{};
  const auto thread_name = ("bxi_" + bus_name).substr(0, 15);
  if (pthread_setname_np(pthread_self(), thread_name.c_str()) != 0) {
    RCLCPP_WARN(logger(), "[%s] worker thread name could not be set", bus_name.c_str());
  }
  RCLCPP_INFO(logger(), "[%s] worker開始 (%zu motors, %dHz)",
              bus_name.c_str(), indices.size(), can_hz_);

  while (running_) {
    if (safe_stopped_) {
      break;
    }
    const auto t0 = clock::now();
    WorkerTimingCycle sample;
    if (timing != nullptr) {
      sample.cycle = timing->cycle_count;
      if (previous_start != clock::time_point{}) {
        sample.interval_ns = elapsed_ns(previous_start, t0);
        sample.start_lateness_ns = std::max<int64_t>(0, sample.interval_ns - period.count());
      }
      previous_start = t0;
    }

    std::vector<double> lp(indices.size()), lv(indices.size()), lkp(indices.size()),
      lkd(indices.size()), leff(indices.size());
    std::vector<double> ofp(indices.size()), ofv(indices.size()), ofe(indices.size()),
      oft(indices.size());
    if (timing != nullptr) {
      sample.allocation_ns = elapsed_ns(t0, clock::now());
    }
    const auto target_lock_start = timing != nullptr ? clock::now() : t0;
    {
      std::lock_guard<std::mutex> lk(data_mutex_);
      const auto target_lock_acquired = timing != nullptr ? clock::now() : t0;
      if (timing != nullptr) {
        sample.target_lock_wait_ns = elapsed_ns(target_lock_start, target_lock_acquired);
      }
      for (size_t k = 0; k < indices.size(); ++k) {
        size_t i = indices[k];
        lp[k] = tgt_pos_[i]; lv[k] = tgt_vel_[i]; lkp[k] = tgt_kp_[i];
        lkd[k] = tgt_kd_[i]; leff[k] = tgt_eff_[i];
        ofp[k] = fb_pos_[i]; ofv[k] = fb_vel_[i]; ofe[k] = fb_eff_[i]; oft[k] = fb_temp_[i];
      }
      if (timing != nullptr) {
        sample.target_lock_hold_ns = elapsed_ns(target_lock_acquired, clock::now());
      }
    }

    const auto can_start = timing != nullptr ? clock::now() : t0;
    for (size_t k = 0; k < indices.size(); ++k) {
      if (safe_stopped_) {
        break;
      }
      size_t i = indices[k];
      if (joints_[i].overheated) {
        continue;
      }
      const auto transaction_start = timing != nullptr ? clock::now() : t0;
      const auto tx_call_realtime_ns = timing != nullptr ?
        std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count() : 0;
      auto fb = joints_[i].actuator->sendControl(lp[k], lv[k], lkp[k], lkd[k], leff[k]);
      if (timing != nullptr) {
        WorkerTimingTransaction transaction;
        transaction.cycle = sample.cycle;
        transaction.joint_index = i;
        transaction.duration_ns = elapsed_ns(transaction_start, clock::now());
        transaction.valid = fb.valid;
        transaction.tx_call_realtime_ns = tx_call_realtime_ns;
        transaction.rx_kernel_realtime_ns = fb.rx_kernel_realtime_ns;
        transaction.rx_userspace_realtime_ns = fb.rx_userspace_realtime_ns;
        if (timing->transaction_count < timing->transactions.size()) {
          timing->transactions[timing->transaction_count] = transaction;
        } else {
          ++timing->dropped_transactions;
        }
        ++timing->transaction_count;
        ++sample.transactions;
        if (!fb.valid) {
          ++sample.invalid_responses;
        }
      }
      if (fb.valid) {
        ofp[k] = fb.position; ofv[k] = fb.velocity; ofe[k] = fb.effort; oft[k] = fb.temperature;
        if (!std::isfinite(fb.temperature)) {
          RCLCPP_ERROR(logger(), "[%s] %s 両温度センサー故障。全軸を停止します。",
            bus_name.c_str(), joints_[i].name.c_str());
          joints_[i].temperature_sensor_fault = true;
          safe_stopped_ = true;
        } else if (fb.temperature > 80.0) {
          RCLCPP_ERROR(logger(), "[%s] %s 過熱(%.1f)。全軸を停止します。",
                       bus_name.c_str(), joints_[i].name.c_str(), fb.temperature);
          joints_[i].overheated = true;
          safe_stopped_ = true;
        }
      } else if (auto * actuator = dynamic_cast<BxiActuatorDriver *>(joints_[i].actuator.get());
        actuator != nullptr && actuator->state() == ActuatorState::kSafeStopped)
      {
        joints_[i].communication_fault = true;
        safe_stopped_ = true;
        RCLCPP_ERROR(logger(), "[%s] %s communication fault; stopping all motors",
          bus_name.c_str(), joints_[i].name.c_str());
      }
    }
    if (timing != nullptr) {
      sample.can_ns = elapsed_ns(can_start, clock::now());
    }

    const auto feedback_lock_start = timing != nullptr ? clock::now() : t0;
    {
      std::lock_guard<std::mutex> lk(data_mutex_);
      const auto feedback_lock_acquired = timing != nullptr ? clock::now() : t0;
      if (timing != nullptr) {
        sample.feedback_lock_wait_ns = elapsed_ns(feedback_lock_start, feedback_lock_acquired);
      }
      for (size_t k = 0; k < indices.size(); ++k) {
        size_t i = indices[k];
        fb_pos_[i] = ofp[k]; fb_vel_[i] = ofv[k]; fb_eff_[i] = ofe[k]; fb_temp_[i] = oft[k];
      }
      if (timing != nullptr) {
        sample.feedback_lock_hold_ns = elapsed_ns(feedback_lock_acquired, clock::now());
      }
    }
    if (timing != nullptr) {
      sample.work_ns = elapsed_ns(t0, clock::now());
      if (timing->cycle_count < timing->cycles.size()) {
        timing->cycles[timing->cycle_count] = sample;
      } else {
        ++timing->dropped_cycles;
      }
      ++timing->cycle_count;
    }

    if (safe_stopped_) {
      break;
    }

    std::this_thread::sleep_until(t0 + period);
  }
  if (safe_stopped_) {
    // Each worker owns its bus. Disable every actuator on that bus before exiting, including
    // actuators whose most recent target was supplied before another bus reported the fault.
    for (size_t i : indices) {
      auto * actuator = dynamic_cast<BxiActuatorDriver *>(joints_[i].actuator.get());
      if (actuator != nullptr && actuator->state() == ActuatorState::kSafeStopped) {
        continue;  // The driver already sent a best-effort disable on this transition.
      }
      if (!joints_[i].actuator->disable().valid) {
        RCLCPP_ERROR(logger(), "[%s] %s disable acknowledgement missing",
          bus_name.c_str(), joints_[i].name.c_str());
      }
    }
  }
  RCLCPP_INFO(logger(), "[%s] worker終了", bus_name.c_str());
}

// ---------------------------------------------------------------------------
// read / write
// ---------------------------------------------------------------------------
hardware_interface::return_type BxiSystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  for (size_t i = 0; i < n_joints_; ++i) {
    st_pos_[i] = fb_pos_[i];
    st_vel_[i] = fb_vel_[i];
    st_eff_[i] = fb_eff_[i];
    st_temp_[i] = fb_temp_[i];
  }
  return safe_stopped_ ? hardware_interface::return_type::ERROR :
         hardware_interface::return_type::OK;
}

hardware_interface::return_type BxiSystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (safe_stopped_) {
    return hardware_interface::return_type::ERROR;
  }
  std::lock_guard<std::mutex> lk(data_mutex_);
  for (size_t i = 0; i < n_joints_; ++i) {
    if (!joints_[i].actuator) {
      continue;
    }
    const bool active = std::isfinite(cmd_pos_[i]);
    const double kp = active ? (std::isfinite(cmd_kp_[i]) ? cmd_kp_[i] : joints_[i].default_kp)
                             : 0.0;
    const double kd = active ? (std::isfinite(cmd_kd_[i]) ? cmd_kd_[i] : joints_[i].default_kd)
                             : 0.0;
    const double pos = active ? cmd_pos_[i] : st_pos_[i];
    const double vel = (active && std::isfinite(cmd_vel_[i])) ? cmd_vel_[i] : 0.0;
    const double eff = (active && std::isfinite(cmd_eff_[i])) ? cmd_eff_[i] : 0.0;
    tgt_pos_[i] = pos;
    tgt_vel_[i] = vel;
    tgt_kp_[i] = kp;
    tgt_kd_[i] = kd;
    tgt_eff_[i] = eff;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace bxi_hardware

PLUGINLIB_EXPORT_CLASS(
  bxi_hardware::BxiSystemInterface, hardware_interface::SystemInterface)
