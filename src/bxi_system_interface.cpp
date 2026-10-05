// mevius2_hardware/RobstrideSystemInterface を雛形に BXI MIT protocol用へ改修。
#include "bxi_hardware/bxi_system_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <set>

#include <pthread.h>
#include <sched.h>
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
  fb_sequence_.assign(n_joints_, 0);
  fb_received_ns_.assign(n_joints_, 0);
  fb_published_ns_.assign(n_joints_, 0);

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

  // Optional per-bus transport. Buses not listed keep the sequential worker.
  bus_transport_.clear();
  if (!root["can_transport"]) {
    return true;
  }
  std::map<std::string, std::vector<int>> bus_motor_ids;
  for (const auto & jr : joints_) {
    if (jr.enabled) {
      bus_motor_ids[jr.can_bus].push_back(jr.can_id);
    }
  }
  try {
    const YAML::Node transports = root["can_transport"];
    if (!transports.IsMap()) {
      RCLCPP_ERROR(logger(), "can_transport はbusごとのmapが必要です");
      return false;
    }
    const double period_ms = 1000.0 / can_hz_;
    for (const auto & entry : transports) {
      const auto bus = entry.first.as<std::string>();
      if (active_bus_set.count(bus) == 0 || !entry.second.IsMap()) {
        RCLCPP_ERROR(logger(), "can_transport: 未知またはinactiveなCAN bus '%s'", bus.c_str());
        return false;
      }
      BusTransport transport;
      bool has_max_age = false;
      bool has_command_max_age = false;
      bool has_thread_setting = false;
      bool has_grace = false;
      bool has_first_write_timeout = false;
      for (const auto & item : entry.second) {
        const auto key = item.first.as<std::string>();
        if (key == "rx_mode") {
          const auto value = item.second.as<std::string>();
          if (value != "sequential" && value != "separate") {
            RCLCPP_ERROR(logger(), "can_transport.%s.rx_mode: sequential / separate が必要 ('%s')",
              bus.c_str(), value.c_str());
            return false;
          }
          transport.separate = value == "separate";
        } else if (key == "tx_trigger") {
          const auto value = item.second.as<std::string>();
          if (value != "write" && value != "periodic") {return false;}
          transport.write_triggered = value == "write";
        } else if (key == "direct_feedback") {
          transport.direct_feedback = item.second.as<bool>();
        } else if (key == "stream_write_grace_ms") {
          transport.write_grace_ms = item.second.as<double>();
          has_grace = true;
        } else if (key == "stream_first_write_timeout_ms") {
          transport.first_write_timeout_ms = item.second.as<double>();
          has_first_write_timeout = true;
        } else if (key == "tx_frame") {
          const auto value = item.second.as<std::string>();
          if (value != "unicast" && value != "broadcast") {
            RCLCPP_ERROR(logger(), "can_transport.%s.tx_frame: unicast / broadcast が必要 ('%s')",
              bus.c_str(), value.c_str());
            return false;
          }
          transport.broadcast = value == "broadcast";
        } else if (key == "stream_feedback_max_age_ms") {
          transport.stream_feedback_max_age_ms = item.second.as<double>();
          has_max_age = true;
        } else if (key == "stream_command_max_age_ms") {
          transport.stream_command_max_age_ms = item.second.as<double>();
          has_command_max_age = true;
        } else if (key == "send_thread_priority" || key == "receive_thread_priority") {
          const int priority = item.second.as<int>();
          if (priority < 1 || priority > 98) {
            RCLCPP_ERROR(logger(), "can_transport.%s.%s は1〜98（SCHED_FIFO）", bus.c_str(),
              key.c_str());
            return false;
          }
          (key == "send_thread_priority" ? transport.send_thread_priority :
          transport.receive_thread_priority) = priority;
          has_thread_setting = true;
        } else if (key == "thread_cpus") {
          transport.thread_cpus = item.second.as<std::vector<int>>();
          if (transport.thread_cpus.empty() ||
            std::any_of(transport.thread_cpus.begin(), transport.thread_cpus.end(),
            [](int cpu) {return cpu < 0 || cpu >= CPU_SETSIZE;}))
          {
            RCLCPP_ERROR(logger(), "can_transport.%s.thread_cpus は0以上のCPU番号の空でないリスト",
              bus.c_str());
            return false;
          }
          has_thread_setting = true;
        } else if (key == "hil_isolated_id1_broadcast_blocks") {
          const int blocks = item.second.as<int>();
          if (blocks != 3) {
            RCLCPP_ERROR(logger(), "can_transport.%s.hil_isolated_id1_broadcast_blocks は3のみ",
              bus.c_str());
            return false;
          }
          transport.isolated_id1_broadcast_blocks = 3;
        } else {
          RCLCPP_ERROR(logger(), "can_transport.%s: 未知のキー '%s'", bus.c_str(), key.c_str());
          return false;
        }
      }

      if (!transport.separate) {
        if (transport.broadcast || has_max_age || has_command_max_age ||
          transport.isolated_id1_broadcast_blocks != 0 || has_thread_setting ||
          transport.write_triggered || transport.direct_feedback || has_grace)
        {
          RCLCPP_ERROR(logger(), "can_transport.%s: tx_frame: broadcast、"
            "stream_feedback_max_age_ms、stream_command_max_age_ms、"
            "hil_isolated_id1_broadcast_blocks、send/receive_thread_priority、thread_cpus は "
            "rx_mode: separate が必要", bus.c_str());
          return false;
        }
      } else if (!has_max_age || !std::isfinite(transport.stream_feedback_max_age_ms) ||
        transport.stream_feedback_max_age_ms < 2.0 * period_ms ||
        transport.stream_feedback_max_age_ms > 50.0)
      {
        // Required with no default: the sequential timeout settings mean something else.
        RCLCPP_ERROR(logger(), "can_transport.%s.stream_feedback_max_age_ms は必須で、"
          "周期の2倍（%.3f ms）以上 50 ms 以下", bus.c_str(), 2.0 * period_ms);
        return false;
      }
      if (transport.separate) {
        if (!has_command_max_age) {
          transport.stream_command_max_age_ms = std::max(5.0, 2.0 * period_ms);
        }
        if (!std::isfinite(transport.stream_command_max_age_ms) ||
          transport.stream_command_max_age_ms < 2.0 * period_ms ||
          transport.stream_command_max_age_ms > 50.0)
        {
          // write() must run at least every two stream periods; the controller_manager period
          // must be shorter than this value with margin.
          RCLCPP_ERROR(logger(), "can_transport.%s.stream_command_max_age_ms は周期の2倍"
            "（%.3f ms）以上 50 ms 以下", bus.c_str(), 2.0 * period_ms);
          return false;
        }
      }

      if (transport.write_triggered) {
        transport.direct_feedback = true;
        if (!has_grace || !std::isfinite(transport.write_grace_ms) ||
          transport.write_grace_ms <= 0 ||
          period_ms + transport.write_grace_ms >= transport.stream_feedback_max_age_ms ||
          period_ms + transport.write_grace_ms >= transport.stream_command_max_age_ms ||
          !std::isfinite(transport.first_write_timeout_ms) ||
          transport.first_write_timeout_ms <= 0 || transport.first_write_timeout_ms > 1000)
        {
          RCLCPP_ERROR(logger(), "write trigger requires explicit grace with freshness margin "
            "and finite first-write timeout (0,1000] ms");
          return false;
        }
      } else if (has_grace || has_first_write_timeout) {
        RCLCPP_ERROR(logger(), "write grace / first-write timeout require tx_trigger: write");
        return false;
      }

      auto ids = bus_motor_ids[bus];
      std::sort(ids.begin(), ids.end());
      if (transport.isolated_id1_broadcast_blocks != 0) {
        if (!transport.broadcast || bus != "can0" || ids != std::vector<int>{1} ||
          bus_formats_[bus] != CanFrameFormat::kFd)
        {
          RCLCPP_ERROR(logger(), "can_transport.%s.hil_isolated_id1_broadcast_blocks は "
            "can0・ID 1の1軸のみ・can_frame_format: fd・tx_frame: broadcast が必要", bus.c_str());
          return false;
        }
      } else if (transport.broadcast) {
        bool contiguous = !ids.empty();
        for (size_t k = 0; k < ids.size(); ++k) {
          contiguous = contiguous && ids[k] == static_cast<int>(k + 1);
        }
        const size_t length = ids.size() * 8;
        if (bus_formats_[bus] == CanFrameFormat::kClassic || !contiguous ||
          length > kCanFdMaxDataLength || !isValidCanFdDataLength(length))
        {
          // Blocks for unconfigured IDs are not defined until the vendor answers.
          RCLCPP_ERROR(logger(), "can_transport.%s: broadcast はCAN FDで、有効な関節の"
            "CAN IDが1..Nの連続、かつN×8 BがCAN FDの有効長（N=1,2,3,4,6,8）である必要",
            bus.c_str());
          return false;
        }
      }
      bus_transport_[bus] = transport;
    }
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(logger(), "can_transport の値が不正です: %s", e.what());
    return false;
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

// A write/read fault may transition hardware out of ACTIVE without calling on_deactivate.
// Finished monitor threads remain joinable until cleanup; destroying them would terminate.
BxiSystemInterface::~BxiSystemInterface()
{
  releaseResources();
}

hardware_interface::CallbackReturn BxiSystemInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  releaseResources();
  return hardware_interface::CallbackReturn::SUCCESS;
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
  const auto activation_start = std::chrono::steady_clock::now();
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

  // Configs without can_transport use the existing sequential defaults.
  for (const auto & entry : bus_joints) {bus_transport_.try_emplace(entry.first);}
  safe_stopped_ = false;
  if (bus_joints.empty()) {
    if (std::getenv("BXI_STREAM_TRACE_CYCLES") != nullptr) {
      RCLCPP_ERROR(logger(), "BXI_STREAM_TRACE_CYCLES requires an active 1-axis broadcast bus");
      return hardware_interface::CallbackReturn::ERROR;
    }
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

    stream_trace_extended_ = std::getenv("BXI_STREAM_TRACE_CYCLES") != nullptr;
    stream_trace_cycles_ = parseStreamTraceCycles(std::getenv("BXI_STREAM_TRACE_CYCLES"));
    if (stream_trace_extended_) {
      if (worker_timing_dir_.empty() || bus_joints.size() != 1) {
        throw std::invalid_argument("expanded trace requires timing directory and one bus");
      }
      const auto & first = *bus_joints.begin();
      const auto & transport = bus_transport_.at(first.first);
      if (first.second.size() != 1 || !transport.separate || !transport.broadcast) {
        throw std::invalid_argument("expanded trace requires 1-axis separate broadcast");
      }
    }
    control_sequence_ = 0;
    legacy_command_sequence_ = 0;
    legacy_command_published_ns_ = 0;
    std::fill(fb_sequence_.begin(), fb_sequence_.end(), 0);
    direct_joint_.assign(n_joints_, false);
    write_joint_.assign(n_joints_, false);
    control_streams_.clear();
    const bool control_trace = std::getenv("BXI_CONTROL_TRACE") != nullptr;
    for (const auto & [bus_name, indices] : bus_joints) {
      const auto & transport = bus_transport_.at(bus_name);
      if (transport.write_triggered || transport.direct_feedback) {
        prepared_write_storage_[bus_name] = std::make_unique<WriteStreamStorage>(indices.size());
        for (auto i : indices) {
          direct_joint_[i] = transport.direct_feedback;
          write_joint_[i] = transport.write_triggered;
        }
      }
      if (transport.write_triggered || transport.direct_feedback ||
        (control_trace && transport.separate))
      {
        ControlStream control;
        control.indices = indices;
        control.read_diagnostics = transport.direct_feedback;
        control.commands.resize(indices.size());
        control.feedback.resize(indices.size());
        control.failures.resize(indices.size());
        control.diagnostics.resize(256);
        if (!worker_timing_dir_.empty()) {
          control.capacity = stream_trace_cycles_;
          const size_t bytes = stream_trace_cycles_ * indices.size() * sizeof(ControlRecord);
          if (bytes > kStreamTraceMemoryLimit) {throw std::length_error("control trace too large");}
          control.records.resize(stream_trace_cycles_ * indices.size());
          control.output = std::make_unique<TraceBufferedWriter>(std::make_shared<TraceFileOps>());
        }
        control_streams_.push_back(std::move(control));
      }
    }
    // Validate the aggregate before allocating; no bus may enable during this whole pass.
    size_t total_trace_bytes = 0;
    for (const auto & control : control_streams_) {
      const size_t bytes = control.records.capacity() * sizeof(ControlRecord) +
        control.diagnostics.capacity() * sizeof(ControlStream::DiagnosticEvent);
      if (bytes > kStreamTraceMemoryLimit - total_trace_bytes) {
        throw std::length_error("aggregate control trace exceeds 256 MiB");
      }
      total_trace_bytes += bytes;
    }
    for (const auto & [bus_name, indices] : bus_joints) {
      if (!worker_timing_dir_.empty() && bus_transport_.at(bus_name).separate) {
        const auto bytes = streamTracePayloadBytes(stream_trace_cycles_, indices.size());
        if (bytes > kStreamTraceMemoryLimit - total_trace_bytes) {
          throw std::length_error("all stream traces exceed 256 MiB");
        }
        total_trace_bytes += bytes;
      }
    }
    size_t actual_trace_bytes = total_trace_bytes;
    for (const auto & [name, indices] : bus_joints) {
      if (!worker_timing_dir_.empty() && bus_transport_.at(name).separate) {
        actual_trace_bytes -= streamTracePayloadBytes(stream_trace_cycles_, indices.size());
      }
    }
    for (const auto & [bus_name, indices] : bus_joints) {
      if (worker_timing_dir_.empty()) {
        if (bus_transport_.at(bus_name).separate) {
          prepared_stream_traces_.emplace(bus_name, prepareTimingTrace(0, indices.size()));
        }
        continue;
      }
      if (bus_transport_.at(bus_name).separate) {
        auto trace = prepareTimingTrace(stream_trace_cycles_, indices.size());
        if (!trace || trace->payload_bytes > kStreamTraceMemoryLimit - actual_trace_bytes) {
          throw std::length_error("all actual stream traces exceed 256 MiB");
        }
        actual_trace_bytes += trace->payload_bytes;
        RCLCPP_INFO(logger(), "[%s] trace prepared before enable: cycles=%zu rx=%zu bytes=%zu "
          "allocation_ns=%lld sizeof_cycle=%zu sizeof_rx=%zu", bus_name.c_str(),
          trace->cycles.size(), trace->rx.size(), trace->payload_bytes,
          static_cast<long long>(trace->allocation_ns), sizeof(StreamCycleRecord), sizeof(StreamRxRecord));
        prepared_stream_traces_.emplace(bus_name, std::move(trace));
        auto output = prepareTimingOutput(worker_timing_dir_, bus_name);
        if (!output) {throw std::runtime_error("missing trace output");}
        RCLCPP_INFO(logger(), "[%s] trace output prepared before enable: %s", bus_name.c_str(), output->stem().c_str());
        for (auto & control : control_streams_) {
          if (control.indices == indices && control.output) {
            control.output_path = output->stem() + "_control.csv";
            if (std::filesystem::exists(control.output_path)) {
              throw std::runtime_error("control trace output already exists");
            }
          }
        }
        stream_trace_outputs_.emplace(bus_name, std::move(output));
      } else {
        auto trace = std::make_unique<WorkerTimingTrace>();
        trace->cycles.resize(20000);
        trace->transactions.resize(20000 * indices.size());
        trace->period_ns = static_cast<int64_t>(1e9 / can_hz_);
        worker_timing_.emplace(bus_name, std::move(trace));
      }
    }

    if (!worker_timing_dir_.empty()) {
      std::ifstream status("/proc/self/status");
      for (std::string line; std::getline(status, line); ) {
        if (line.rfind("VmRSS:", 0) == 0 || line.rfind("VmLck:", 0) == 0) {
          RCLCPP_INFO(logger(), "trace memory before enable: %s", line.c_str());
        }
      }
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
    // No write() of this activation yet; the send threads skip the command-age check until one.
    last_write_ns_.store(0, std::memory_order_relaxed);
    for (auto & [bus_name, live] : live_joints_) {
      if (bus_transport_[bus_name].separate) {
        if (!startStream(bus_name, live)) {
          return fail_activation();
        }
        continue;
      }
      WorkerTimingTrace * timing = worker_timing_.count(bus_name) ? worker_timing_.at(bus_name).get() : nullptr;
      workers_.emplace_back(&BxiSystemInterface::workerLoop, this, bus_name, live, timing);
    }
    // A streamed bus is active only after every motor has answered a streamed command.
    for (auto & [bus_name, streamer] : streamers_) {
      const auto & transport = bus_transport_[bus_name];
      const auto wait = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double, std::milli>(
          transport.stream_feedback_max_age_ms + 1000.0 / can_hz_));
      if (!streamer->waitForFirstFeedback(wait)) {
        RCLCPP_ERROR(logger(), "[%s] 送受信分離の最初の応答が %.1f ms 以内に揃いません。起動を中止します。",
          bus_name.c_str(), std::chrono::duration<double, std::milli>(wait).count());
        return fail_activation();
      }
      if (safe_stopped_) {
        RCLCPP_ERROR(logger(), "[%s] 最初の応答の確認後に安全停止しました。起動を中止します。",
          bus_name.c_str());
        return fail_activation();
      }
    }
    for (auto & [bus_name, streamer] : streamers_) {
      workers_.emplace_back(
        &BxiSystemInterface::streamMonitorLoop, this, bus_name, live_joints_[bus_name]);
    }
    for (auto & [bus_name, streamer] : streamers_) {
      if (streamer->config().write_triggered) {streamer->armWrites();}
    }
    onStreamFeedbackConfirmed();
    // Any worker or stream may have stopped since its own check; never report a stopped system
    // as active.
    if (safe_stopped_) {
      RCLCPP_ERROR(logger(), "起動中に安全停止しました。起動を中止します。");
      return fail_activation();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "起動中に例外: %s", e.what());
    return fail_activation();
  }
  RCLCPP_INFO(logger(), "on_activate 完了: %zu バスで worker 起動 (activation_ns=%lld)", workers_.size(),
    static_cast<long long>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - activation_start).count()));
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
  // Stop the send threads before joining the monitors. controller_manager has already stopped
  // calling write(); a stream left running while the monitors join (several ms) sees the command
  // age pass stream_command_max_age_ms and would report a stalled control loop during a normal
  // deactivation. Clearing running_ first sends the monitors down their normal-deactivation
  // path; stopStream() is idempotent and serialized with a monitor that is handling a fault.
  running_ = false;
  // Ask every send thread to stop before joining any of them: joining one bus can take several
  // ms, and another bus still sending meanwhile would see its command age grow as well.
  for (const auto & [bus_name, streamer] : streamers_) {
    streamer->requestStop();
  }
  for (const auto & [bus_name, streamer] : streamers_) {
    stopStream(bus_name);
  }
  onBeforeJoinWorkers();
  stopWorkers();
  // Streams stop before any disable so that the send sockets receive acknowledgements again.
  std::set<size_t> streamed_joints;
  for (const auto & [bus_name, live] : live_joints_) {
    const auto transport = bus_transport_.find(bus_name);
    if (transport == bus_transport_.end() || !transport->second.separate) {
      continue;
    }
    stopStream(bus_name);
    disableAxes(bus_name, live, "deactivate");
    streamed_joints.insert(live.begin(), live.end());
  }
  for (size_t i = 0; i < n_joints_; ++i) {
    if (joints_[i].actuator && streamed_joints.count(i) == 0) {
      joints_[i].actuator->disable();
    }
  }
  // Every motor has been disabled; logging and CSV output may take their time now.
  for (const auto & [bus_name, streamer] : streamers_) {
    reportStream(bus_name);
  }
  writeControlTiming();
  control_streams_.clear();
  prepared_write_storage_.clear();
  streamers_.clear();
  prepared_stream_traces_.clear();
  stream_trace_outputs_.clear();
  reported_streams_.clear();
  for (auto & [name, bus] : rx_buses_) {
    bus->close();
  }
  rx_buses_.clear();
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
// Streamed buses (rx_mode: separate)
// ---------------------------------------------------------------------------
bool BxiSystemInterface::startStream(
  const std::string & bus_name, const std::vector<size_t> & indices)
{
  const auto & transport = bus_transport_[bus_name];
  auto rx_bus = makeBus(bus_formats_[bus_name]);
  if (auto * socketcan = dynamic_cast<SocketCanBus *>(rx_bus.get())) {
    // Timing runs also request raw hardware RX timestamps; the kernel software timestamp comes
    // in the same record, and interfaces without hardware stamps (vcan) leave that field zero.
    socketcan->setRxHardwareTimestamping(!worker_timing_dir_.empty());
  }
  if (!rx_bus->open(bus_name)) {
    RCLCPP_ERROR(logger(), "[%s] 受信用CANオープン失敗", bus_name.c_str());
    return false;
  }
  // Registered before the send socket changes so that releaseResources() restores it.
  rx_buses_[bus_name] = rx_bus;

  StreamConfig config;
  config.write_triggered = transport.write_triggered;
  config.direct_feedback = transport.direct_feedback;
  config.write_grace = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double, std::milli>(transport.write_grace_ms));
  config.control_trace = std::getenv("BXI_CONTROL_TRACE") != nullptr;
  config.first_write_timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double, std::milli>(transport.first_write_timeout_ms));
  for (const auto & [name, storage] : prepared_write_storage_) {
    if (storage) {config.stop_wake_fds.push_back(storage->event_fd);}
  }
  // Already-started streamers also own FDs needed by this bus's fault fan-out.
  for (const auto & [name, streamer] : streamers_) {
    const auto & wake_fds = streamer->config().stop_wake_fds;
    config.stop_wake_fds.insert(config.stop_wake_fds.end(), wake_fds.begin(), wake_fds.end());
  }
  std::sort(config.stop_wake_fds.begin(), config.stop_wake_fds.end());
  config.stop_wake_fds.erase(std::unique(config.stop_wake_fds.begin(), config.stop_wake_fds.end()),
    config.stop_wake_fds.end());
  config.period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / can_hz_));
  config.max_feedback_age = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double, std::milli>(transport.stream_feedback_max_age_ms));
  config.max_command_age = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double, std::milli>(transport.stream_command_max_age_ms));
  config.command_written_ns = &last_write_ns_;
  config.broadcast = transport.broadcast;
  config.send_priority = transport.send_thread_priority;
  config.receive_priority = transport.receive_thread_priority;
  config.cpus = transport.thread_cpus;
  if (!worker_timing_dir_.empty()) {
    config.trace_cycles = stream_trace_cycles_;
    config.trace_extended = stream_trace_extended_;
  }
  std::vector<uint32_t> reply_ids;
  std::vector<BxiActuatorDriver *> drivers;
  for (size_t i : indices) {
    auto * driver = dynamic_cast<BxiActuatorDriver *>(joints_[i].actuator.get());
    if (driver == nullptr) {
      RCLCPP_ERROR(logger(), "[%s] %s: 送受信分離はBXIドライバのみ対応", bus_name.c_str(),
        joints_[i].name.c_str());
      return false;
    }
    drivers.push_back(driver);
    reply_ids.push_back(protocol::defaultMasterId(static_cast<uint32_t>(joints_[i].can_id)));
  }
  if (transport.isolated_id1_broadcast_blocks != 0) {
    config.padded_broadcast_blocks = transport.isolated_id1_broadcast_blocks;
    for (int id = 2; id <= static_cast<int>(transport.isolated_id1_broadcast_blocks); ++id) {
      config.forbidden_reply_ids.push_back(id);
      reply_ids.push_back(protocol::defaultMasterId(static_cast<uint32_t>(id)));
    }
    RCLCPP_WARN(logger(), "[%s] 1台HIL専用: ID 1のみ接続の前提で、%zuブロックのブロードキャストを送り、"
      "ID 2以降の応答を検知したら停止します", bus_name.c_str(),
      transport.isolated_id1_broadcast_blocks);
  }

  auto * rx_socket = dynamic_cast<SocketCanBus *>(rx_bus.get());
  if (rx_socket != nullptr && !rx_socket->setReceiveStandardIds(reply_ids)) {
    RCLCPP_ERROR(logger(), "[%s] 受信IDフィルタを設定できません", bus_name.c_str());
    return false;
  }
  rx_bus->flushRx();
  auto & tx_bus = buses_[bus_name];
  auto * tx_socket = dynamic_cast<SocketCanBus *>(tx_bus.get());
  if (tx_socket != nullptr && !tx_socket->setReceiveEnabled(false)) {
    RCLCPP_ERROR(logger(), "[%s] 送信ソケットの受信を停止できません", bus_name.c_str());
    return false;
  }
  tx_bus->flushRx();

  if (auto * socket = dynamic_cast<SocketCanBus *>(tx_bus.get())) {
    socket->setNonblockingSend(config.write_triggered);
  }
  auto storage = prepared_write_storage_.count(bus_name) ?
    std::move(prepared_write_storage_.at(bus_name)) : nullptr;
  auto streamer = std::make_unique<BxiBusStreamer>(
    tx_bus.get(), rx_bus.get(), drivers, config,
    [this, bus_name, indices](
      const std::vector<StreamFeedback> & feedback, std::vector<StreamCommand> & commands) {
      return streamCycle(bus_name, indices, feedback, commands);
    },
    &safe_stopped_, std::move(storage));
  // Even tracing-off uses a prepared empty trace, keeping allocation out of start().
  if (!prepared_stream_traces_.count(bus_name)) {
    RCLCPP_ERROR(logger(), "[%s] missing pre-enable trace storage", bus_name.c_str());
    return false;
  }
  auto prepared = std::move(prepared_stream_traces_.at(bus_name));
  if (!streamer->start(std::move(prepared))) {
    RCLCPP_ERROR(logger(), "[%s] 送受信分離を開始できません: %s", bus_name.c_str(),
      streamer->error().c_str());
    return false;
  }
  for (auto & control : control_streams_) {
    if (control.indices == indices) {control.streamer = streamer.get();}
  }
  streamers_[bus_name] = std::move(streamer);
  RCLCPP_INFO(logger(), "[%s] 送受信分離を開始 (%zu motors, %s, %dHz, 鮮度しきい値 %.1f ms, "
    "指令の古さのしきい値 %.1f ms)",
    bus_name.c_str(), indices.size(), transport.broadcast ? "broadcast" : "unicast", can_hz_,
    transport.stream_feedback_max_age_ms, transport.stream_command_max_age_ms);
  return true;
}

bool BxiSystemInterface::streamCycle(
  const std::string & bus_name, const std::vector<size_t> & indices,
  const std::vector<StreamFeedback> & feedback, std::vector<StreamCommand> & commands)
{
  // Same checks as the sequential worker. Messages are logged after the lock is released.
  int sensor_fault = -1;
  int overheated = -1;
  double overheated_temperature = 0.0;
  {
    std::lock_guard<std::mutex> lk(data_mutex_);
    for (size_t k = 0; k < indices.size(); ++k) {
      const size_t i = indices[k];
      if (feedback[k].updated) {
        const auto & fb = feedback[k].feedback;
        fb_pos_[i] = fb.position;
        fb_vel_[i] = fb.velocity;
        fb_eff_[i] = fb.effort;
        fb_temp_[i] = fb.temperature;
        fb_sequence_[i] = feedback[k].sequence;
        fb_received_ns_[i] = feedback[k].received_ns;
        fb_published_ns_[i] = std::chrono::steady_clock::now().time_since_epoch().count();
        if (!std::isfinite(fb.temperature)) {
          joints_[i].temperature_sensor_fault = true;
          sensor_fault = static_cast<int>(i);
        } else if (fb.temperature > 80.0) {
          joints_[i].overheated = true;
          overheated = static_cast<int>(i);
          overheated_temperature = fb.temperature;
        }
      }
      commands[k] = {tgt_pos_[i], tgt_vel_[i], tgt_kp_[i], tgt_kd_[i], tgt_eff_[i]};
      commands[k].sequence = legacy_command_sequence_;
      commands[k].published_ns = legacy_command_published_ns_;
    }
  }
  if (sensor_fault >= 0) {
    RCLCPP_ERROR(logger(), "[%s] %s 両温度センサー故障。全軸を停止します。",
      bus_name.c_str(), joints_[static_cast<size_t>(sensor_fault)].name.c_str());
  }
  if (overheated >= 0) {
    RCLCPP_ERROR(logger(), "[%s] %s 過熱(%.1f)。全軸を停止します。", bus_name.c_str(),
      joints_[static_cast<size_t>(overheated)].name.c_str(), overheated_temperature);
  }
  return sensor_fault < 0 && overheated < 0;
}

void BxiSystemInterface::streamMonitorLoop(std::string bus_name, std::vector<size_t> indices)
{
  const auto thread_name = ("bxim_" + bus_name).substr(0, 15);
  pthread_setname_np(pthread_self(), thread_name.c_str());
  BxiBusStreamer * streamer = streamers_.at(bus_name).get();
  std::vector<bool> unavailable(indices.size(), false);
  while (running_ && !streamer->sendFinished()) {
    for (size_t axis = 0; axis < indices.size(); ++axis) {
      const bool current = streamer->controlUnavailable(axis);
      if (current != unavailable[axis]) {
        RCLCPP_WARN(logger(), "[%s] %s feedback_snapshot_unavailable=%s (diagnostic only)",
          bus_name.c_str(), joints_[indices[axis]].name.c_str(), current ? "true" : "false");
        unavailable[axis] = current;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!running_ || !safe_stopped_) {
    return;  // Normal deactivation: releaseResources() stops and disables this bus.
  }
  // The send thread stopped for a fault here or on another bus. Disable now instead of waiting
  // for controller_manager to deactivate the component.
  stopStream(bus_name);
  disableAxes(bus_name, indices, "safe-stop");
  reportStream(bus_name);  // log and CSV only after the motors have been told to disable
}

void BxiSystemInterface::stopStream(const std::string & bus_name)
{
  // Only the thread joins and the socket changes that disabling needs happen here. Logging and
  // CSV output wait for reportStream(), after the disable frames have been sent.
  std::lock_guard<std::mutex> lock(stream_stop_mutex_);
  const auto streamer = streamers_.find(bus_name);
  if (streamer != streamers_.end()) {
    streamer->second->stop();
  }
  const auto rx_bus = rx_buses_.find(bus_name);
  if (rx_bus == rx_buses_.end()) {
    return;
  }
  // Sequential disable needs the send socket to receive acknowledgements again.
  const auto tx_bus = buses_.find(bus_name);
  if (tx_bus != buses_.end()) {
    if (auto * socketcan = dynamic_cast<SocketCanBus *>(tx_bus->second.get());
      socketcan != nullptr && !socketcan->setReceiveEnabled(true))
    {
      RCLCPP_ERROR(logger(), "[%s] 送信ソケットの受信を再開できません", bus_name.c_str());
    }
    if (auto * socketcan = dynamic_cast<SocketCanBus *>(tx_bus->second.get())) {
      socketcan->setNonblockingSend(false);
    }
    tx_bus->second->flushRx();
  }
  rx_bus->second->close();
  rx_buses_.erase(rx_bus);
  onStreamStopped(bus_name);
}

void BxiSystemInterface::reportStream(const std::string & bus_name)
{
  const auto report_start = std::chrono::steady_clock::now();
  const BxiBusStreamer * streamer = nullptr;
  {
    // The monitor and releaseResources() can both report a bus; claim it once, then log and
    // write outside the lock so that another bus can stop meanwhile. The streamer stays alive
    // until releaseResources() clears the map after joining the monitors.
    std::lock_guard<std::mutex> lock(stream_stop_mutex_);
    const auto found = streamers_.find(bus_name);
    if (found == streamers_.end() || !reported_streams_.insert(bus_name).second) {
      return;
    }
    streamer = found->second.get();
  }
  onStreamReport(bus_name);
  const auto stats = streamer->stats();
  if (stats.temperature_fault_axis >= 0) {
    const auto i = live_joints_.at(bus_name).at(static_cast<size_t>(stats.temperature_fault_axis));
    const bool sensors_failed = !std::isfinite(stats.temperature_fault_c);
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      joints_[i].temperature_sensor_fault = sensors_failed;
      joints_[i].overheated = !sensors_failed;
    }
    RCLCPP_ERROR(logger(), "[%s] %s motor id=%d thermal fault: temperature=%.3f C "
      "both_sensors_failed=%s received_steady_ns=%lld",
      bus_name.c_str(), joints_[i].name.c_str(), joints_[i].can_id, stats.temperature_fault_c,
      sensors_failed ? "true" : "false", static_cast<long long>(stats.temperature_fault_received_ns));
  }
  onStreamStats(bus_name, stats);
  std::string per_axis;
  for (size_t k = 0; k < stats.replies.size(); ++k) {
    char text[96];
    std::snprintf(text, sizeof(text), " [%zu] replies=%llu no_new=%llu max_age=%.3fms", k,
      static_cast<unsigned long long>(stats.replies[k]),
      static_cast<unsigned long long>(stats.cycles_without_new_reply[k]),
      stats.max_age_ms[k]);
    per_axis += text;
  }
  RCLCPP_INFO(logger(), "[%s] 送受信分離を停止: cycles=%llu skipped=%llu partial=%llu "
    "frames=%llu fault=%s axis=%d external=%s max_cmd_age=%.3fms stale_zero_gain=%zu/%zu%s",
    bus_name.c_str(),
    static_cast<unsigned long long>(stats.cycles),
    static_cast<unsigned long long>(stats.skipped_cycles),
    static_cast<unsigned long long>(stats.partial_cycles),
    static_cast<unsigned long long>(stats.frames_sent), streamFaultName(stats.fault),
    stats.fault_axis, stats.stopped_externally ? "yes" : "no", stats.max_command_age_ms,
    stats.command_stale_zero_gain_frames, stats.command_stale_expected_frames, per_axis.c_str());
  if (!worker_timing_dir_.empty()) {
    writeStreamTiming(bus_name, *streamer, report_start);
  }
}

void BxiSystemInterface::writeStreamTiming(
  const std::string & bus_name, const BxiBusStreamer & streamer,
  std::chrono::steady_clock::time_point report_start)
{
  // After stop/join and disable, including the monitor's safe-stop path.
  try {
    sched_param param{};
    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    if (::sched_getparam(0, &param) != 0 || ::sched_getaffinity(0, sizeof(cpus), &cpus) != 0) {
      throw std::runtime_error("cannot inspect trace writer scheduling");
    }
    std::string cpu_list;
    for (int i = 0; i < CPU_SETSIZE; ++i) {
      if (CPU_ISSET(i, &cpus)) {cpu_list += (cpu_list.empty() ? "" : ";") + std::to_string(i);}
    }
    RCLCPP_INFO(logger(), "[%s] trace writer: policy=%d priority=%d cpus=%s", bus_name.c_str(),
      ::sched_getscheduler(0), param.sched_priority, cpu_list.c_str());
    const auto & output = stream_trace_outputs_.at(bus_name);
    const auto result = output->write(streamer, report_start);
    RCLCPP_INFO(logger(), "[%s] stream timing: %zu cycles, %zu replies, dropped %zu/%zu; %s",
      bus_name.c_str(), result.cycle_rows, result.rx_rows, result.dropped_cycles, result.dropped_rx,
      output->stem().c_str());
    RCLCPP_INFO(logger(), "[%s] trace complete: report_to_complete_ns=%lld csv_write_elapsed_ns=%lld",
      bus_name.c_str(), static_cast<long long>(result.report_to_complete_ns),
      static_cast<long long>(result.csv_write_elapsed_ns));
    if (result.report_to_complete_ns > 2000000000LL) {
      RCLCPP_ERROR(logger(), "[%s] stream timing exceeds 2 second criterion", bus_name.c_str());
    }
    if (result.tmp_cleanup_failed) {RCLCPP_WARN(logger(), "[%s] complete tmp cleanup failed", bus_name.c_str());}
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "[%s] stream timing CSV 書込失敗: %s", bus_name.c_str(), e.what());
  }
}

void BxiSystemInterface::disableAxes(
  const std::string & bus_name, const std::vector<size_t> & indices, const char * reason)
{
  for (size_t i : indices) {
    auto & actuator = joints_[i].actuator;
    if (!actuator) {
      continue;
    }
    auto * driver = dynamic_cast<BxiActuatorDriver *>(actuator.get());
    if (driver != nullptr && driver->state() == ActuatorState::kSafeStopped) {
      // The driver never waits for an acknowledgement once it has latched safe-stop.
      actuator->disable();
      RCLCPP_WARN(logger(), "[%s] %s disable (%s): best effort after safe-stop, "
        "acknowledgement not checked", bus_name.c_str(), joints_[i].name.c_str(), reason);
    } else if (driver != nullptr && driver->state() == ActuatorState::kDisabled) {
      continue;  // Already acknowledged.
    } else if (actuator->disable().valid) {
      RCLCPP_INFO(logger(), "[%s] %s disable (%s): acknowledged", bus_name.c_str(),
        joints_[i].name.c_str(), reason);
    } else {
      RCLCPP_ERROR(logger(), "[%s] %s disable (%s): acknowledgement missing", bus_name.c_str(),
        joints_[i].name.c_str(), reason);
    }
  }
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
  const auto read_start = std::chrono::steady_clock::now().time_since_epoch().count();
  for (auto & control : control_streams_) {
    control.record_index = control.record_count++;
    if (control.record_index < control.capacity) {
      for (size_t axis = 0; axis < control.indices.size(); ++axis) {
        auto & row = control.records[control.record_index * control.indices.size() + axis];
        row.read_start = read_start;
      }
    }
  }
  const auto copy_legacy = [&] {
    for (size_t i = 0; i < n_joints_; ++i) {
      if (!direct_joint_.empty() && direct_joint_[i]) {continue;}
      st_pos_[i] = fb_pos_[i]; st_vel_[i] = fb_vel_[i];
      st_eff_[i] = fb_eff_[i]; st_temp_[i] = fb_temp_[i];
    }
  };
  bool legacy_read = direct_joint_.empty();
  for (size_t i = 0; i < n_joints_ && !legacy_read; ++i) {
    legacy_read = joints_[i].actuator && !direct_joint_[i];
  }
  if (legacy_read)
  {
    std::lock_guard<std::mutex> lk(data_mutex_);
    copy_legacy();
    for (auto & control : control_streams_) {
      if (control.read_diagnostics) {continue;}
      for (size_t axis = 0; axis < control.indices.size(); ++axis) {
        const auto i = control.indices[axis];
        control.feedback[axis].sequence = fb_sequence_[i];
        control.feedback[axis].received_ns = fb_received_ns_[i];
        control.feedback[axis].published_ns = fb_published_ns_[i];
      }
    }
  }
  for (auto & control : control_streams_) {
    for (size_t axis = 0; axis < control.indices.size(); ++axis) {
      const auto i = control.indices[axis];
      PublishedFeedback scratch;
      unsigned attempts = 0;
      const bool got = control.read_diagnostics && control.streamer->readFeedback(axis, scratch, &attempts);
      const bool was_unavailable = control.failures[axis] >= 3;
      if (got) {
        control.feedback[axis] = scratch;
        control.failures[axis] = 0;
        st_pos_[i] = scratch.feedback.position; st_vel_[i] = scratch.feedback.velocity;
        st_eff_[i] = scratch.feedback.effort; st_temp_[i] = scratch.feedback.temperature;
      } else if (control.read_diagnostics && control.failures[axis] < UINT32_MAX) {
        ++control.failures[axis];
      }
      if (control.read_diagnostics) {
        const bool unavailable = control.failures[axis] >= 3;
        control.streamer->setControlUnavailable(axis, unavailable);
        if (unavailable != was_unavailable) {
          if (control.diagnostic_count < control.diagnostics.size()) {
            control.diagnostics[control.diagnostic_count] = {
              std::chrono::steady_clock::now().time_since_epoch().count(),
              control_sequence_ + 1, i, unavailable};
          }
          ++control.diagnostic_count;
        }
      }
      if (control.record_index < control.capacity) {
        auto & row = control.records[control.record_index * control.indices.size() + axis];
        row.acquired_ns = std::chrono::steady_clock::now().time_since_epoch().count();
        row.rx_sequence = control.feedback[axis].sequence;
        row.rx_received = control.feedback[axis].received_ns;
        row.rx_published = control.feedback[axis].published_ns;
        row.setAttempts(attempts); row.failures = control.failures[axis];
      }
    }
  }
  const auto read_end = std::chrono::steady_clock::now().time_since_epoch().count();
  for (auto & control : control_streams_) {
    if (control.record_index < control.capacity) {
      for (size_t axis = 0; axis < control.indices.size(); ++axis) {
        control.records[control.record_index * control.indices.size() + axis].read_done = read_end;
      }
    }
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
  const auto write_start = std::chrono::steady_clock::now().time_since_epoch().count();
  for (auto & control : control_streams_) {
    if (control.record_index == SIZE_MAX) {control.record_index = control.record_count++;}
  }
  ++control_sequence_;
  // Publish write-triggered buses without entering any legacy worker lock.
  for (auto & control : control_streams_) {
    if (!control.streamer->config().write_triggered) {continue;}
    for (size_t axis = 0; axis < control.indices.size(); ++axis) {
      const auto i = control.indices[axis];
      const bool active = std::isfinite(cmd_pos_[i]);
      auto & packet = control.commands[axis];
      packet.command = {active ? cmd_pos_[i] : st_pos_[i],
        active && std::isfinite(cmd_vel_[i]) ? cmd_vel_[i] : 0.0,
        active ? (std::isfinite(cmd_kp_[i]) ? cmd_kp_[i] : joints_[i].default_kp) : 0.0,
        active ? (std::isfinite(cmd_kd_[i]) ? cmd_kd_[i] : joints_[i].default_kd) : 0.0,
        active && std::isfinite(cmd_eff_[i]) ? cmd_eff_[i] : 0.0};
      packet.sequence = control_sequence_;
    }
    const auto published = std::chrono::steady_clock::now().time_since_epoch().count();
    for (auto & packet : control.commands) {packet.published_ns = published;}
    const auto notify_before = std::chrono::steady_clock::now().time_since_epoch().count();
    const bool ok = control.streamer->publishCommands(control.commands.data());
    const auto notify_after = std::chrono::steady_clock::now().time_since_epoch().count();
    if (control.record_index < control.capacity) {
      for (size_t axis = 0; axis < control.indices.size(); ++axis) {
        auto & row = control.records[control.record_index * control.indices.size() + axis];
        row.published = published; row.notify_before = notify_before; row.notify_after = notify_after;
      }
    }
    if (!ok) {return hardware_interface::return_type::ERROR;}
  }
  bool legacy_write = write_joint_.empty();
  for (size_t i = 0; i < n_joints_ && !legacy_write; ++i) {
    legacy_write = joints_[i].actuator && !write_joint_[i];
  }
  if (legacy_write)
  {
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
      if (!write_joint_.empty() && write_joint_[i]) {continue;}
      tgt_pos_[i] = pos;
      tgt_vel_[i] = vel;
      tgt_kp_[i] = kp;
      tgt_kd_[i] = kd;
      tgt_eff_[i] = eff;
    }
    legacy_command_sequence_ = control_sequence_;
    legacy_command_published_ns_ = std::chrono::steady_clock::now().time_since_epoch().count();
  }

  // Streamed buses safe-stop when this stops advancing (stream_command_max_age_ms).
  last_write_ns_.store(
    std::chrono::steady_clock::now().time_since_epoch().count(), std::memory_order_relaxed);
  const auto write_end = std::chrono::steady_clock::now().time_since_epoch().count();
  for (auto & control : control_streams_) {
    if (control.record_index < control.capacity) {
      for (size_t axis = 0; axis < control.indices.size(); ++axis) {
        auto & row = control.records[control.record_index * control.indices.size() + axis];
        row.setCycle(control_sequence_); row.write_start = write_start; row.write_end = write_end;
        if (!row.published) {row.published = write_end;}
      }
    }
    control.record_index = SIZE_MAX;
  }
  return hardware_interface::return_type::OK;
}

void BxiSystemInterface::writeControlTiming()
{
  // Lifecycle cleanup only: controller_manager has stopped read/write. The safe-stop monitor
  // may report TX/RX earlier, but must never serialize a control-thread-owned trace.
  for (auto & control : control_streams_) {
    for (size_t event = 0; event < std::min(control.diagnostic_count, control.diagnostics.size()); ++event) {
      const auto & d = control.diagnostics[event];
      RCLCPP_WARN(logger(), "%s feedback_snapshot_unavailable=%s cycle=%llu steady_ns=%lld",
        joints_[d.joint].name.c_str(), d.unavailable ? "true" : "false",
        static_cast<unsigned long long>(d.cycle), static_cast<long long>(d.at_ns));
    }
    if (control.diagnostic_count > control.diagnostics.size()) {
      RCLCPP_WARN(logger(), "dropped feedback snapshot diagnostic events=%zu",
        control.diagnostic_count - control.diagnostics.size());
    }
    if (!control.output || control.output_path.empty()) {continue;}
    try {
      auto & out = *control.output;
      out.open(control.output_path);
      out.text("#format_version=1,dropped_control_cycles=");
      out.number(control.record_count > control.capacity ? control.record_count - control.capacity : 0);
      out.text(",payload_bytes="); out.number(control.records.capacity() * sizeof(ControlRecord));
      out.text(",diagnostic_payload_bytes=");
      out.number(control.diagnostics.capacity() * sizeof(ControlStream::DiagnosticEvent));
      out.text("\ncycle,joint_index,read_start_ns,read_end_ns,acquired_ns,rx_sequence,rx_received_ns,"
        "rx_published_ns,attempts,consecutive_failures,feedback_snapshot_unavailable,write_start_ns,"
        "command_published_ns,notify_before_ns,notify_after_ns,write_end_ns\n");
      for (size_t cycle = 0; cycle < std::min(control.record_count, control.capacity); ++cycle) {
        for (size_t axis = 0; axis < control.indices.size(); ++axis) {
          const auto & r = control.records[cycle * control.indices.size() + axis];
          out.field(r.cycle()); out.field(control.indices[axis]); out.field(r.read_start);
          out.field(r.read_done); out.field(r.acquired_ns); out.field(r.rx_sequence);
          out.field(r.rx_received); out.field(r.rx_published); out.field(r.attempts());
          out.field(r.failures); out.field(r.failures >= 3 ? 1 : 0); out.field(r.write_start);
          out.field(r.published); out.field(r.notify_before); out.field(r.notify_after);
          out.number(r.write_end); out.text("\n");
        }
      }
      out.close();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger(), "control trace failed: %s", e.what());
    }
  }
}

}  // namespace bxi_hardware

PLUGINLIB_EXPORT_CLASS(
  bxi_hardware::BxiSystemInterface, hardware_interface::SystemInterface)
