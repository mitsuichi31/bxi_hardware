// vcan 専用の模擬BXIモーター。BxiSystemInterface（逐次・送受信分離）を実機なしで動かし、
// スケジューリング条件を比較するために使う。既定では、モーター状態や物理量は模擬しない
// （応答の位置・速度・トルクは中央値のまま）。
//
//   ros2 run bxi_hardware bxi_vcan_motor_emulator --iface vcan0 --ids 1 --delay-us 300
//
// --follow 1 では、ブロードキャストの指令のうち Kp > 0 のものを受けると、応答の位置を指令の位置へ
// 一次遅れ（既定の時定数 50 ms、--follow-time-constant-ms で明示指定可）で近づけ、速度も返す（Kp = 0 ならその場に止まる）。HIL ランナーの判定
// （移動量や往復）を vcan で通すための簡単な追従で、力学は模擬しない。
//
// 個別ID宛ての8 Bフレーム（enable/disable/MIT指令）にはすぐに応答し、ID 0x7FF のブロードキャスト
// には、ブロックごとに --delay-us 後に応答する。--replies-per-command で1指令あたりの応答数を増やす
// と、受信側の負荷を上げられる（重複応答も受信IDフィルタを通る）。
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <cstring>

#include "bxi_hardware/can/socketcan_bus.hpp"
#include "bxi_hardware/protocol/mit_protocol.hpp"

namespace
{

using bxi_hardware::CanFrame;
using bxi_hardware::CanFrameFormat;
using bxi_hardware::SocketCanBus;
namespace protocol = bxi_hardware::protocol;
using clock_type = std::chrono::steady_clock;

std::atomic<bool> g_stop{false};

void onSignal(int) {g_stop = true;}

struct Options
{
  std::string iface = "vcan0";
  std::set<int> ids{1};
  int delay_us = 300;
  int replies_per_command = 1;
  bool follow = false;
  double follow_time_constant_ms = 50.0;
  std::map<int, int> startup_window_ms;
  bool startup_delay = false;
  bool mode_text = false;
  std::set<int> silent_control_ids;
  std::map<int, double> initial_positions;
  std::map<int, std::pair<double, int>> position_steps;
  std::map<int, int> silent_after_ms;
};

// Emulated joint state for --follow (under the shared mutex).
struct Joint
{
  double position = 0.0;
  double velocity = 0.0;
  clock_type::time_point updated{};
};

void usage()
{
  std::printf(
    "usage: bxi_vcan_motor_emulator [--iface vcan0] [--ids 1,2,3] [--delay-us 300]"
    " [--replies-per-command 1] [--follow 0|1] [--follow-time-constant-ms 50]\n"
    " [--startup-window-ms 1:4,2:20,3:0] [--startup-action drop|delay]"
    " [--mode-text 0|1] [--silent-control-ids 1,2,3]\n"
    " [--initial-positions id:rad,...] [--position-step id:rad@ms] [--silent-after-ms id@ms]\n"
    "  vcan interfaces only. Stop with Ctrl+C; counts are printed on exit.\n");
}

bool parse(int argc, char ** argv, Options & o)
{
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string key = argv[i];
    const std::string value = argv[i + 1];
    if (key == "--initial-positions" || key == "--position-step" || key == "--silent-after-ms") {
      std::stringstream list(value);
      for (std::string item; std::getline(list, item, ','); ) {
        try {
          const auto colon = item.find(':');
          const auto at = item.find('@');
          const auto delimiter = key == "--silent-after-ms" ? at : colon;
          if (delimiter == std::string::npos) {return false;}
          size_t used = 0;
          const int id = std::stoi(item.substr(0, delimiter), &used);
          if (used != delimiter || id < 1 || id > int(protocol::kMaxSupportedMotorCanId)) {return false;}
          if (key != "--silent-after-ms") {
            const auto end = key == "--position-step" ? at : item.size();
            if (end == std::string::npos || end <= colon + 1) {return false;}
            const auto text = item.substr(colon + 1, end - colon - 1);
            const double rad = std::stod(text, &used);
            if (used != text.size() || !std::isfinite(rad)) {return false;}
            if (key == "--initial-positions") {o.initial_positions[id] = rad;}
            else {o.position_steps[id].first = rad;}
          }
          if (key != "--initial-positions") {
            if (at == std::string::npos) {return false;}
            const auto text = item.substr(at + 1);
            const int ms = std::stoi(text, &used);
            if (used != text.size() || ms < 0 || ms > 600000) {return false;}
            if (key == "--position-step") {o.position_steps[id].second = ms;}
            else {o.silent_after_ms[id] = ms;}
          }
        } catch (const std::exception &) {return false;}
      }
      if (value.empty() || value.back() == ',') {return false;}
    } else if (key == "--iface") {
      o.iface = value;
    } else if (key == "--startup-window-ms") {
      std::stringstream list(value);
      for (std::string item; std::getline(list, item, ','); ) {
        const auto colon = item.find(':');
        if (colon == std::string::npos) {return false;}
        try {
          size_t used_id = 0, used_ms = 0;
          const int id = std::stoi(item.substr(0, colon), &used_id);
          const int ms = std::stoi(item.substr(colon + 1), &used_ms);
          if (used_id != colon || used_ms != item.size() - colon - 1 ||
            id < 1 || id > int(protocol::kMaxSupportedMotorCanId) || ms < 0 || ms > 60000) {return false;}
          o.startup_window_ms[id] = ms;
        } catch (const std::exception &) {return false;}
      }
    } else if (key == "--startup-action") {
      if (value != "drop" && value != "delay") {return false;}
      o.startup_delay = value == "delay";
    } else if (key == "--mode-text") {
      if (value != "0" && value != "1") {return false;}
      o.mode_text = value == "1";
    } else if (key == "--silent-control-ids") {
      std::stringstream list(value);
      for (std::string item; std::getline(list, item, ','); ) {
        try {
          size_t used = 0; const int id = std::stoi(item, &used);
          if (used != item.size() || id < 1 || id > int(protocol::kMaxSupportedMotorCanId)) {return false;}
          o.silent_control_ids.insert(id);
        } catch (const std::exception &) {return false;}
      }
    } else if (key == "--ids") {
      o.ids.clear();
      std::stringstream list(value);
      for (std::string item; std::getline(list, item, ','); ) {
        o.ids.insert(std::atoi(item.c_str()));
      }
    } else if (key == "--delay-us") {
      o.delay_us = std::atoi(value.c_str());
    } else if (key == "--replies-per-command") {
      o.replies_per_command = std::atoi(value.c_str());
    } else if (key == "--follow-time-constant-ms") {
      char * end = nullptr;
      o.follow_time_constant_ms = std::strtod(value.c_str(), &end);
      if (end == value.c_str() || *end != '\0' ||
        !std::isfinite(o.follow_time_constant_ms) || o.follow_time_constant_ms <= 0.0 ||
        o.follow_time_constant_ms > 1000.0)
      {
        return false;
      }
    } else if (key == "--follow") {
      if (value != "0" && value != "1") {
        return false;
      }
      o.follow = value == "1";
    } else {
      return false;
    }
  }
  const bool ids_ok = !o.ids.empty() && *o.ids.begin() >= 1 &&
    *o.ids.rbegin() <= static_cast<int>(protocol::kMaxSupportedMotorCanId);
  for (const auto & entry : o.initial_positions) {
    const protocol::MitLimits limits;
    if (!o.ids.count(entry.first) || entry.second < limits.p_min || entry.second > limits.p_max) {return false;}
  }
  for (const auto & entry : o.position_steps) {if (!o.ids.count(entry.first)) {return false;}}
  for (const auto & entry : o.silent_after_ms) {if (!o.ids.count(entry.first)) {return false;}}
  return argc % 2 == 1 && o.iface.rfind("vcan", 0) == 0 && ids_ok && o.delay_us >= 0 &&
         o.replies_per_command >= 1 && o.replies_per_command <= 10;
}

CanFrame feedbackFrame(int motor_id, const Joint * joint = nullptr)
{
  CanFrame frame;
  frame.id = protocol::defaultMasterId(static_cast<uint32_t>(motor_id));
  frame.extended = false;
  frame.dlc = 8;
  // Position/velocity/torque at mid-scale; NTC1/NTC2 about 30 degC.
  frame.data = {{static_cast<uint8_t>(motor_id), 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 0x55, 0x5A}};
  if (joint != nullptr) {
    const protocol::MitLimits limits;
    const auto p = protocol::floatToUint(joint->position, limits.p_min, limits.p_max, 16);
    const auto v = protocol::floatToUint(joint->velocity, limits.v_min, limits.v_max, 12);
    if (p && v) {
      frame.data[1] = static_cast<uint8_t>(*p >> 8);
      frame.data[2] = static_cast<uint8_t>(*p & 0xFF);
      frame.data[3] = static_cast<uint8_t>(*v >> 4);
      frame.data[4] = static_cast<uint8_t>(((*v & 0xF) << 4) | (frame.data[4] & 0x0F));
    }
  }
  return frame;
}

// --follow: move toward the commanded position of one broadcast block when Kp > 0.
void follow(Joint & joint, const uint8_t * block, clock_type::time_point now, double time_constant_s)
{
  const protocol::MitLimits limits;
  const uint32_t p_raw = (static_cast<uint32_t>(block[0]) << 8) | block[1];
  const uint32_t kp_raw = (static_cast<uint32_t>(block[3] & 0x0F) << 8) | block[4];
  const double dt = joint.updated == clock_type::time_point{} ? 0.0 :
    std::chrono::duration<double>(now - joint.updated).count();
  joint.updated = now;
  if (kp_raw == 0 || dt <= 0.0) {
    joint.velocity = 0.0;
    return;
  }
  const double target = protocol::uintToFloat(p_raw, limits.p_min, limits.p_max, 16);
  const double step = (target - joint.position) * (1.0 - std::exp(-dt / time_constant_s));
  joint.position += step;
  joint.velocity = step / dt;
}

}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  if (!parse(argc, argv, options)) {
    usage();
    return 2;
  }
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  // Separate sockets: SocketCanBus holds its I/O mutex while recv() waits, which would delay
  // every reply sent on the same socket.
  SocketCanBus rx_bus(CanFrameFormat::kFd);
  SocketCanBus tx_bus(CanFrameFormat::kFd);
  if (!rx_bus.open(options.iface) || !tx_bus.open(options.iface) ||
    !tx_bus.setReceiveEnabled(false))
  {
    std::fprintf(stderr, "%s を CAN FD（mtu 72）で開けません\n", options.iface.c_str());
    return 1;
  }

  std::mutex mutex;
  std::condition_variable cv;
  std::multimap<clock_type::time_point, int> pending;
  // The mid-scale position the fixed reply carries, so --follow starts where it would.
  std::map<int, Joint> joints;
  for (const int id : options.ids) {
    const protocol::MitLimits limits;
    joints[id].position = protocol::uintToFloat(0x7FFF, limits.p_min, limits.p_max, 16);
  }
  for (const auto & entry : options.initial_positions) {joints[entry.first].position = entry.second;}
  const auto started = clock_type::now();
  std::set<int> stepped;
  auto silent = [&](int id, clock_type::time_point now) {
      const auto it = options.silent_after_ms.find(id);
      return it != options.silent_after_ms.end() && now - started >= std::chrono::milliseconds(it->second);
    };
  auto apply_steps = [&](clock_type::time_point now) {
      for (const auto & entry : options.position_steps) {
        if (!stepped.count(entry.first) && now - started >= std::chrono::milliseconds(entry.second.second)) {
          joints[entry.first].position += entry.second.first;
          stepped.insert(entry.first);
        }
      }
    };
  std::atomic<long> unicast_frames{0};
  std::atomic<long> broadcast_frames{0};
  std::atomic<long> replies_sent{0};
  // Receiver-thread-only readiness; no new thread, lock or shared queue is introduced.
  std::map<int, clock_type::time_point> ready;
  unsigned long dropped_controls = 0;

  std::thread sender([&]() {
      std::unique_lock<std::mutex> lock(mutex);
      while (!g_stop) {
        if (pending.empty()) {
          cv.wait_for(lock, std::chrono::milliseconds(5));
          continue;
        }
        const auto due = pending.begin()->first;
        if (clock_type::now() < due) {
          cv.wait_until(lock, due);
          continue;
        }
        const int id = pending.begin()->second;
        pending.erase(pending.begin());
        const auto reply_now = clock_type::now();
        const int reply_id = id < 0 ? (-id - 1) / 16 : id;
        if (silent(reply_id, reply_now)) {continue;}
        apply_steps(reply_now);
        CanFrame reply;
        if (id < 0) {
          // Existing pending queue also carries immutable text chunks encoded as negative IDs.
          const int code = -id - 1, motor_id = code / 16, chunk = code % 16;
          static constexpr char text[] = "STATE : 02 MOTOR\r\nEnter Motor Mode\r\n";
          reply.id = 0x7F0U + static_cast<uint32_t>(motor_id); reply.extended = false;
          reply.dlc = 8;
          const size_t offset = static_cast<size_t>(chunk) * 8;
          std::memcpy(reply.data.data(), text + offset, std::min<size_t>(8, sizeof(text) - 1 - offset));
        } else {reply = feedbackFrame(id, (options.follow || !options.initial_positions.empty() || !options.position_steps.empty()) ? &joints[id] : nullptr);}
        lock.unlock();
        if (tx_bus.send(reply)) {
          ++replies_sent;
        }
        lock.lock();
      }
    });

  std::printf("bxi_vcan_motor_emulator: %s ids=%zu delay=%dus replies/command=%d follow=%d follow_time_constant_ms=%.6g\n",
    options.iface.c_str(), options.ids.size(), options.delay_us, options.replies_per_command,
    options.follow ? 1 : 0, options.follow_time_constant_ms);
  CanFrame frame;
  while (!g_stop) {
    if (!rx_bus.recv(frame, 5) || frame.extended) {
      continue;
    }
    const auto now = clock_type::now();
    const int id = static_cast<int>(frame.id);
    std::lock_guard<std::mutex> lock(mutex);
    apply_steps(now);
    if (options.ids.count(id) != 0 && frame.dlc == 8) {
      ++unicast_frames;
      const bool enable = frame.data8() == protocol::encodeSpecialCommand(protocol::SpecialCommand::kEnable);
      const bool disable = frame.data8() == protocol::encodeSpecialCommand(protocol::SpecialCommand::kDisable);
      if (enable) {
        const auto window = options.startup_window_ms.find(id);
        ready[id] = now + std::chrono::milliseconds(window == options.startup_window_ms.end() ? 0 : window->second);
        if (options.mode_text) {
          constexpr size_t text_size = sizeof("STATE : 02 MOTOR\r\nEnter Motor Mode\r\n") - 1;
          for (size_t chunk = 0; chunk < (text_size + 7) / 8; ++chunk) {
            pending.emplace(now + std::chrono::microseconds(800 + chunk * 500),
              -(id * 16 + static_cast<int>(chunk) + 1));
          }
        }
      }
      if (!enable && !disable && options.silent_control_ids.count(id)) {++dropped_controls; continue;}
      auto due = now;
      if (!enable && !disable && ready.count(id) && now < ready[id]) {
        if (!options.startup_delay) {++dropped_controls; continue;}
        due = ready[id];
      }
      for (int r = 0; r < options.replies_per_command; ++r) {
        pending.emplace(due, id);
      }
    } else if (frame.id == 0x7FFU) {
      ++broadcast_frames;
      for (int block = 0; block < frame.dlc / 8; ++block) {
        if (options.ids.count(block + 1) == 0) {
          continue;
        }
        if (options.silent_control_ids.count(block + 1)) {++dropped_controls; continue;}
        if (options.follow) {
          follow(joints[block + 1], &frame.data[static_cast<size_t>(block) * 8], now,
            options.follow_time_constant_ms / 1000.0);
        }
        for (int r = 0; r < options.replies_per_command; ++r) {
          pending.emplace(now + std::chrono::microseconds(options.delay_us), block + 1);
        }
      }
    }
    cv.notify_all();
  }
  cv.notify_all();
  sender.join();
  std::printf("unicast frames=%ld broadcast frames=%ld replies sent=%ld\n",
    unicast_frames.load(), broadcast_frames.load(), replies_sent.load());
  std::printf("startup_action=%s mode_text=%d dropped_controls=%lu\n",
    options.startup_delay ? "delay" : "drop", options.mode_text ? 1 : 0, dropped_controls);
  rx_bus.close();
  tx_bus.close();
  return 0;
}
