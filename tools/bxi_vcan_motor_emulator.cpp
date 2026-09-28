// vcan 専用の模擬BXIモーター。BxiSystemInterface（逐次・送受信分離）を実機なしで動かし、
// スケジューリング条件を比較するために使う。モーター状態や物理量は模擬しない。
//
//   ros2 run bxi_hardware bxi_vcan_motor_emulator --iface vcan0 --ids 1 --delay-us 300
//
// 個別ID宛ての8 Bフレーム（enable/disable/MIT指令）にはすぐに応答し、ID 0x7FF のブロードキャスト
// には、ブロックごとに --delay-us 後に応答する。--replies-per-command で1指令あたりの応答数を増やす
// と、受信側の負荷を上げられる（重複応答も受信IDフィルタを通る）。
#include <atomic>
#include <chrono>
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
};

void usage()
{
  std::printf(
    "usage: bxi_vcan_motor_emulator [--iface vcan0] [--ids 1,2,3] [--delay-us 300]"
    " [--replies-per-command 1]\n"
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
    } else {
      return false;
    }
  }
  const bool ids_ok = !o.ids.empty() && *o.ids.begin() >= 1 &&
    *o.ids.rbegin() <= static_cast<int>(protocol::kMaxSupportedMotorCanId);
  return argc % 2 == 1 && o.iface.rfind("vcan", 0) == 0 && ids_ok && o.delay_us >= 0 &&
         o.replies_per_command >= 1 && o.replies_per_command <= 10;
}

CanFrame feedbackFrame(int motor_id)
{
  CanFrame frame;
  frame.id = protocol::defaultMasterId(static_cast<uint32_t>(motor_id));
  frame.extended = false;
  frame.dlc = 8;
  // Position/velocity/torque at mid-scale; NTC1/NTC2 about 30 degC.
  frame.data = {{static_cast<uint8_t>(motor_id), 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 0x55, 0x5A}};
  return frame;
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
        lock.unlock();
        if (tx_bus.send(feedbackFrame(id))) {
          ++replies_sent;
        }
        lock.lock();
      }
    });

  std::printf("bxi_vcan_motor_emulator: %s ids=%zu delay=%dus replies/command=%d\n",
    options.iface.c_str(), options.ids.size(), options.delay_us, options.replies_per_command);
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
