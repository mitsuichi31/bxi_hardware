// 実機の前提確認ツール（1軸）。ドライバと同じ SocketCanBus で次を調べる。
//   1. MENU状態で、力を出さないMIT指令（Kp=Kd=トルク=0）への応答と応答時間
//   2. enable特殊フレーム（FF×7 + FC）にMIT応答が返るか、Motor Modeに入るか
//   3. Motor Modeで、力を出さないMIT指令への応答時間
//   4. disable特殊フレーム（FF×7 + FD）にMIT応答が返るか、MENUへ戻るか
// 状態の判定には、モーターの文字出力（ID 0x7F0 + can_id）の "STATE : 0x ..." 行を使う。
// 送るMIT指令はすべて Kp=Kd=トルク=0 なので、モーターは力を出さない。
// 終了時（Ctrl+C を含む）は必ず disable → Esc（文字入力 0x7E0 + can_id に 0x1B）を送る。
//
// 例: ros2 run bxi_hardware bxi_probe --iface can0 --can-id 1 --format classic --count 500

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "bxi_hardware/can/socketcan_bus.hpp"
#include "bxi_hardware/protocol/mit_protocol.hpp"

namespace
{

using bxi_hardware::CanFrame;
using bxi_hardware::CanFrameFormat;
using bxi_hardware::SocketCanBus;
namespace protocol = bxi_hardware::protocol;
using clock_type = std::chrono::steady_clock;

volatile std::sig_atomic_t g_stop = 0;
void onSignal(int) {g_stop = 1;}

struct Options
{
  std::string iface = "can0";
  int can_id = 1;
  CanFrameFormat format = CanFrameFormat::kClassic;
  int count = 500;
  int rate_hz = 200;
  int timeout_ms = 10;
};

class Probe
{
public:
  explicit Probe(const Options & options)
  : options_(options), bus_(options.format),
    reply_id_(protocol::defaultMasterId(static_cast<uint32_t>(options.can_id))),
    text_tx_id_(0x7E0U + static_cast<uint32_t>(options.can_id)),
    text_rx_id_(0x7F0U + static_cast<uint32_t>(options.can_id)) {}

  bool open() {return bus_.open(options_.iface);}
  void close() {bus_.close();}

  // 文字コマンド（CR LF 付き）または Esc を送る。
  void sendText(const std::string & text)
  {
    std::string raw = text == "\x1b" ? text : text + "\r\n";
    for (size_t i = 0; i < raw.size(); i += 8) {
      CanFrame frame;
      frame.id = text_tx_id_;
      frame.extended = false;
      frame.dlc = static_cast<uint8_t>(std::min<size_t>(8, raw.size() - i));
      std::memcpy(frame.data.data(), raw.data() + i, frame.dlc);
      bus_.send(frame);
    }
  }

  // 応答を待たずに、受信したフレームを文字出力として処理する。
  void drain(int milliseconds)
  {
    const auto end = clock_type::now() + std::chrono::milliseconds(milliseconds);
    CanFrame frame;
    while (clock_type::now() < end) {
      if (bus_.recv(frame, 5)) {
        handle(frame);
      }
    }
  }

  // MITフレーム（8バイト）を送り、応答までの時間 [us] を返す。応答がなければ nullopt。
  std::optional<double> transact(const std::array<uint8_t, 8> & data, CanFrame * reply = nullptr)
  {
    CanFrame frame;
    frame.id = static_cast<uint32_t>(options_.can_id);
    frame.extended = false;
    frame.dlc = 8;
    frame.data = data;
    const auto t0 = clock_type::now();
    if (!bus_.send(frame)) {
      ++send_failures_;
      return std::nullopt;
    }
    const auto deadline = t0 + std::chrono::milliseconds(options_.timeout_ms);
    CanFrame in;
    while (clock_type::now() < deadline) {
      if (!bus_.recv(in, 1)) {
        continue;
      }
      if (!in.extended && in.id == reply_id_ && in.dlc == 8) {
        const auto t1 = clock_type::now();
        last_reply_ = in;
        if (reply != nullptr) {
          *reply = in;
        }
        return std::chrono::duration<double, std::micro>(t1 - t0).count();
      }
      handle(in);
    }
    return std::nullopt;
  }

  std::optional<double> zeroCommand()
  {
    return transact(protocol::encodeCommand(protocol::MitCommand{}, protocol::MitLimits{}).value());
  }

  std::optional<double> special(protocol::SpecialCommand command, CanFrame * reply)
  {
    return transact(protocol::encodeSpecialCommand(command), reply);
  }

  // 力を出さないMIT指令を count 回送り、応答時間の統計を表示する。
  void latency(const char * label)
  {
    std::vector<double> samples;
    int timeouts = 0;
    int fd_replies = 0;
    const auto period = std::chrono::microseconds(1000000 / options_.rate_hz);
    auto next = clock_type::now();
    for (int i = 0; i < options_.count && !g_stop; ++i) {
      CanFrame reply;
      const auto us = transact(
        protocol::encodeCommand(protocol::MitCommand{}, protocol::MitLimits{}).value(), &reply);
      if (us) {
        samples.push_back(*us);
        fd_replies += reply.fd ? 1 : 0;
      } else {
        ++timeouts;
      }
      next += period;
      std::this_thread::sleep_until(next);
    }
    std::printf("[%s] 送信 %d / 応答 %zu（うち CAN FD %d）/ timeout %d（%d ms）/ 送信失敗 %d\n",
      label, static_cast<int>(samples.size()) + timeouts, samples.size(), fd_replies, timeouts,
      options_.timeout_ms, send_failures_);
    if (samples.empty()) {
      return;
    }
    std::sort(samples.begin(), samples.end());
    const auto at = [&](double q) {
        return samples[std::min(samples.size() - 1, static_cast<size_t>(q * samples.size()))];
      };
    double sum = 0.0;
    for (double v : samples) {
      sum += v;
    }
    std::printf("[%s] 応答時間 [us]: 最小 %.0f / 平均 %.0f / 中央 %.0f / 99%% %.0f / 最大 %.0f\n",
      label, samples.front(), sum / samples.size(), at(0.5), at(0.99), samples.back());
    printReply(label);
  }

  void printReply(const char * label) const
  {
    if (!last_reply_) {
      return;
    }
    const auto fb = protocol::decodeFeedback(last_reply_->data, protocol::MitLimits{});
    if (fb) {
      std::printf("[%s] 最後の応答: id=0x%02X pos=%+.4f rad vel=%+.3f rad/s tor=%+.3f N·m "
        "NTC1=%.1f degC NTC2=%.1f degC%s\n", label, fb->motor_id, fb->position, fb->velocity,
        fb->torque, fb->mos_temperature, fb->motor_temperature,
        last_reply_->fd ? (last_reply_->brs ? " [FD BRS]" : " [FD]") : "");
    }
  }

  // 最後に見た "STATE : .." 行。
  const std::string & state() const {return state_;}
  void clearState() {state_.clear();}

private:
  void handle(const CanFrame & frame)
  {
    if (frame.extended || frame.id != text_rx_id_) {
      return;
    }
    for (uint8_t i = 0; i < frame.dlc; ++i) {
      const char c = static_cast<char>(frame.data[i]);
      if (c == '\r' || c == '\n') {
        if (!line_.empty()) {
          std::printf("  | %s\n", line_.c_str());
          if (line_.rfind("STATE", 0) == 0) {
            state_ = line_;
          }
          line_.clear();
        }
      } else if (c == '\t' || static_cast<unsigned char>(c) >= 0x20) {
        line_ += c;
      }
    }
  }

  Options options_;
  SocketCanBus bus_;
  uint32_t reply_id_;
  uint32_t text_tx_id_;
  uint32_t text_rx_id_;
  std::string line_;
  std::string state_;
  std::optional<CanFrame> last_reply_;
  int send_failures_ = 0;
};

void usage()
{
  std::printf(
    "usage: bxi_probe [--iface can0] [--can-id 1] [--format classic|fd|fd_brs]\n"
    "                 [--count 500] [--rate 200] [--timeout-ms 10]\n");
}

std::optional<Options> parse(int argc, char ** argv)
{
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "-h" || key == "--help" || i + 1 >= argc) {
      return std::nullopt;
    }
    const std::string value = argv[++i];
    if (key == "--iface") {
      o.iface = value;
    } else if (key == "--can-id") {
      o.can_id = std::atoi(value.c_str());
    } else if (key == "--count") {
      o.count = std::atoi(value.c_str());
    } else if (key == "--rate") {
      o.rate_hz = std::atoi(value.c_str());
    } else if (key == "--timeout-ms") {
      o.timeout_ms = std::atoi(value.c_str());
    } else if (key == "--format") {
      if (value == "classic") {
        o.format = CanFrameFormat::kClassic;
      } else if (value == "fd") {
        o.format = CanFrameFormat::kFd;
      } else if (value == "fd_brs") {
        o.format = CanFrameFormat::kFdBrs;
      } else {
        return std::nullopt;
      }
    } else {
      return std::nullopt;
    }
  }
  if (o.can_id < 1 || o.can_id > 0xF || o.count < 1 || o.rate_hz < 1 || o.rate_hz > 2000 ||
    o.timeout_ms < 1)
  {
    return std::nullopt;
  }
  return o;
}

const char * result(const std::optional<double> & us)
{
  return us ? "応答あり" : "応答なし";
}

}  // namespace

int main(int argc, char ** argv)
{
  const auto options = parse(argc, argv);
  if (!options) {
    usage();
    return 2;
  }
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  Probe probe(*options);
  if (!probe.open()) {
    std::fprintf(stderr, "%s を開けません\n", options->iface.c_str());
    return 1;
  }
  std::printf("bxi_probe: %s can_id=%d reply=0x%03X count=%d rate=%dHz timeout=%dms\n",
    options->iface.c_str(), options->can_id,
    protocol::defaultMasterId(static_cast<uint32_t>(options->can_id)), options->count,
    options->rate_hz, options->timeout_ms);

  // 0. MENUへ戻す
  std::printf("\n[0] Esc を送って MENU へ戻す\n");
  probe.sendText("\x1b");
  probe.drain(300);

  // 1. MENUでの応答時間
  std::printf("\n[1] MENU 状態で、力を出さないMIT指令\n");
  probe.latency("MENU");

  // 2. enable
  std::printf("\n[2] enable 特殊フレーム（FF×7 + FC）\n");
  probe.clearState();
  CanFrame reply;
  const auto enable_us = g_stop ? std::nullopt :
    probe.special(protocol::SpecialCommand::kEnable, &reply);
  std::printf("  MIT応答: %s", result(enable_us));
  if (enable_us) {
    std::printf("（%.0f us）", *enable_us);
  }
  std::printf("\n");
  probe.drain(500);
  const bool motor_mode = probe.state().find("02 MOTOR") != std::string::npos;
  std::printf("  状態: %s → Motor Mode に%s\n",
    probe.state().empty() ? "(STATE 行なし)" : probe.state().c_str(),
    motor_mode ? "入った" : "入ったか不明");

  // 3. Motor Mode での応答時間
  if (!g_stop) {
    std::printf("\n[3] enable 後に、力を出さないMIT指令\n");
    probe.latency("ENABLED");
  }

  // 4. disable
  std::printf("\n[4] disable 特殊フレーム（FF×7 + FD）\n");
  probe.clearState();
  const auto disable_us = probe.special(protocol::SpecialCommand::kDisable, &reply);
  std::printf("  MIT応答: %s", result(disable_us));
  if (disable_us) {
    std::printf("（%.0f us）", *disable_us);
  }
  std::printf("\n");
  probe.drain(500);
  std::printf("  状態: %s\n", probe.state().empty() ? "(STATE 行なし)" : probe.state().c_str());

  // 5. 後始末
  std::printf("\n[5] Esc を送って終了\n");
  probe.sendText("\x1b");
  probe.drain(300);
  probe.close();
  return 0;
}
