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
    "  vcan interfaces only. Stop with Ctrl+C; counts are printed on exit.\n");
}

bool parse(int argc, char ** argv, Options & o)
{
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string key = argv[i];
    const std::string value = argv[i + 1];
    if (key == "--iface") {
      o.iface = value;
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
  std::atomic<long> unicast_frames{0};
  std::atomic<long> broadcast_frames{0};
  std::atomic<long> replies_sent{0};

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
        const CanFrame reply = feedbackFrame(id, options.follow ? &joints[id] : nullptr);
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
    if (options.ids.count(id) != 0 && frame.dlc == 8) {
      ++unicast_frames;
      for (int r = 0; r < options.replies_per_command; ++r) {
        pending.emplace(now, id);
      }
    } else if (frame.id == 0x7FFU) {
      ++broadcast_frames;
      for (int block = 0; block < frame.dlc / 8; ++block) {
        if (options.ids.count(block + 1) == 0) {
          continue;
        }
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
  rx_bus.close();
  tx_bus.close();
  return 0;
}
