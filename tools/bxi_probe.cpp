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
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
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
  bool broadcast = false;  // FD multi-motor command ID, with only motor 1's 8-byte block
  bool menu_only = false;  // Never send the enable command in this mode.
  bool stream = false;  // Scheduled sends and independent receive socket, diagnosis only.
  bool hardware_timestamps = false;  // Request raw RX timestamps in stream diagnosis.
  int broadcast_blocks = 1;  // Three blocks need vcan or bounded hardware opt-in.
  bool isolated_id1_24b = false;  // Explicit opt-in for a bounded single-motor hardware check.
  bool isolated_id1_24b_sweep = false;  // Explicit staged 200/500/1000 Hz check, at most 100 frames.
  bool isolated_id1_24b_extended = false;  // Exactly 1000 frames at 1000 Hz after the short sweep.
  bool vcan_three_replies = false;  // Count IDs 1-3; never accepted on hardware.
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
  std::optional<double> transact(
    const std::array<uint8_t, 8> & data, CanFrame * reply = nullptr, bool broadcast = false,
    clock_type::time_point * sent_at = nullptr, clock_type::time_point * received_at = nullptr)
  {
    CanFrame frame;
    frame.id = broadcast ? 0x7FFU : static_cast<uint32_t>(options_.can_id);
    frame.extended = false;
    frame.dlc = 8;
    frame.data = data;
    const auto t0 = clock_type::now();
    if (sent_at != nullptr) {
      *sent_at = t0;
    }
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
        if (received_at != nullptr) {
          *received_at = t1;
        }
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
    std::vector<clock_type::time_point> send_times;
    std::vector<clock_type::time_point> reply_times;
    samples.reserve(options_.count);
    send_times.reserve(options_.count);
    reply_times.reserve(options_.count);
    int timeouts = 0;
    int fd_replies = 0;
    const auto period = std::chrono::microseconds(1000000 / options_.rate_hz);
    auto next = clock_type::now();
    for (int i = 0; i < options_.count && !g_stop; ++i) {
      CanFrame reply;
      clock_type::time_point sent_at;
      clock_type::time_point received_at;
      const auto us = transact(
        protocol::encodeCommand(protocol::MitCommand{}, protocol::MitLimits{}).value(), &reply,
        options_.broadcast, &sent_at, &received_at);
      send_times.push_back(sent_at);
      if (us) {
        samples.push_back(*us);
        reply_times.push_back(received_at);
        fd_replies += reply.fd ? 1 : 0;
      } else {
        ++timeouts;
        if (options_.menu_only) {
          std::printf("[%s] 無応答または送信失敗を検出したため、追加の指令を停止します\n", label);
          break;
        }
      }
      next += period;
      std::this_thread::sleep_until(next);
    }
    std::printf("[%s] 送信 %d / 応答 %zu（うち CAN FD %d）/ timeout %d（%d ms）/ 送信失敗 %d\n",
      label, static_cast<int>(samples.size()) + timeouts, samples.size(), fd_replies, timeouts,
      options_.timeout_ms, send_failures_);
    const auto print_intervals = [&](const char * name,
        const std::vector<clock_type::time_point> & times) {
        if (times.size() < 2) {
          return;
        }
        std::vector<double> intervals_ms;
        intervals_ms.reserve(times.size() - 1);
        for (size_t i = 1; i < times.size(); ++i) {
          intervals_ms.push_back(
            std::chrono::duration<double, std::milli>(times[i] - times[i - 1]).count());
        }
        const double span_s = std::chrono::duration<double>(times.back() - times.front()).count();
        const double actual_hz = static_cast<double>(times.size() - 1) / span_s;
        const double target_ms = 1000.0 / static_cast<double>(options_.rate_hz);
        const size_t late_intervals = static_cast<size_t>(std::count_if(
          intervals_ms.begin(), intervals_ms.end(),
          [target_ms](double value) {return value > target_ms;}));
        std::sort(intervals_ms.begin(), intervals_ms.end());
        const auto at = [&](double q) {
            return intervals_ms[std::min(intervals_ms.size() - 1,
              static_cast<size_t>(q * intervals_ms.size()))];
          };
        std::printf("[%s] %s: 平均 %.1f Hz / 間隔 中央 %.3f ms / 99%% %.3f ms / "
          "最大 %.3f ms / 目標周期超過 %zu/%zu\n", label, name, actual_hz,
          at(0.5), at(0.99), intervals_ms.back(), late_intervals, intervals_ms.size());
      };
    print_intervals("実送信開始", send_times);
    print_intervals("有効応答取込", reply_times);
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

  // Diagnostic only: schedule zero-torque sends without waiting for each reply. Replies have no
  // sequence number, so this measures counts and timing, not command-to-reply correspondence.
  bool stream()
  {
    last_reply_.reset();
    SocketCanBus receive_bus(options_.format);
    if (options_.hardware_timestamps) {
      receive_bus.setRxHardwareTimestamping(true);
    } else {
      receive_bus.setRxTimestamping(true);
    }
    if (!receive_bus.open(options_.iface)) {
      std::fprintf(stderr, "診断用の受信ソケットを開けません\n");
      return false;
    }
    if (!bus_.setReceiveEnabled(false)) {
      std::fprintf(stderr, "送信ソケットの受信を停止できません\n");
      receive_bus.close();
      return false;
    }

    std::vector<clock_type::time_point> send_times;
    std::vector<clock_type::time_point> reply_times;
    std::vector<std::chrono::system_clock::time_point> kernel_reply_times;
    std::vector<std::chrono::nanoseconds> hardware_reply_times;
    std::vector<double> kernel_to_user_us;
    std::vector<double> feedback_age_ms;
    send_times.reserve(options_.count);
    reply_times.reserve(options_.count);
    kernel_reply_times.reserve(options_.count);
    hardware_reply_times.reserve(options_.count);
    kernel_to_user_us.reserve(options_.count);
    feedback_age_ms.reserve(options_.count);
    std::atomic<int> sent{0};
    std::atomic<int> received{0};
    std::array<std::atomic<int>, 3> received_by_motor{};
    std::array<std::atomic<int64_t>, 3> last_reply_by_motor_ns{};
    for (auto & count : received_by_motor) {
      count.store(0);
    }
    const int motor_count = options_.vcan_three_replies ? 3 : 1;
    std::atomic<bool> sending_done{false};
    std::atomic<bool> fault{false};
    std::atomic<int64_t> last_reply_ns{0};
    std::atomic<int64_t> receive_deadline_ns{0};
    CanFrame final_reply;
    const auto to_ns = [](clock_type::time_point time) {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
      };

    std::thread receiver([&]() {
      CanFrame in;
      while (true) {
        const auto now_ns = to_ns(clock_type::now());
        if (sending_done.load() &&
          (received.load() >= sent.load() * motor_count ||
          now_ns >= receive_deadline_ns.load()))
        {
          break;
        }
        if (!receive_bus.recv(in, 1)) {
          continue;
        }
        int motor_index = -1;
        if (!in.extended && in.dlc == 8) {
          for (int motor = 0; motor < motor_count; ++motor) {
            if (in.id == protocol::defaultMasterId(static_cast<uint32_t>(motor + 1))) {
              motor_index = motor;
              break;
            }
          }
        }
        if (motor_index >= 0) {
          const auto at = clock_type::now();
          const auto fb = protocol::decodeFeedback(in.data, protocol::MitLimits{});
          if (!fb || fb->motor_id != motor_index + 1 ||
            !std::isfinite(fb->mos_temperature) || !std::isfinite(fb->motor_temperature) ||
            fb->mos_temperature > 80.0 || fb->motor_temperature > 80.0)
          {
            fault.store(true);
            continue;
          }
          // Timing columns keep their original meaning: ID 1 only.
          if (motor_index == 0 && reply_times.size() < static_cast<size_t>(options_.count)) {
            reply_times.push_back(at);
            if (in.rx_kernel_realtime_ns > 0) {
              kernel_reply_times.emplace_back(
                std::chrono::nanoseconds(in.rx_kernel_realtime_ns));
            }
            if (in.rx_hardware_raw_ns > 0) {
              hardware_reply_times.emplace_back(in.rx_hardware_raw_ns);
            }
            if (in.rx_kernel_realtime_ns > 0 &&
              in.rx_userspace_realtime_ns >= in.rx_kernel_realtime_ns)
            {
              kernel_to_user_us.push_back(
                static_cast<double>(in.rx_userspace_realtime_ns - in.rx_kernel_realtime_ns) /
                1000.0);
            }
          }
          if (motor_index == 0) {
            final_reply = in;
            last_reply_ns.store(to_ns(at));
          }
          last_reply_by_motor_ns[motor_index].store(to_ns(at));
          received_by_motor[motor_index].fetch_add(1);
          received.fetch_add(1);
        } else if (!in.extended && in.id == text_rx_id_) {
          handle(in);
          if (state_.find("02 MOTOR") != std::string::npos) {
            fault.store(true);
          }
        }
      }
    });

    CanFrame command;
    command.id = 0x7FFU;
    command.extended = false;
    command.dlc = 8;
    command.data = protocol::encodeCommand(protocol::MitCommand{}, protocol::MitLimits{}).value();
    std::array<uint8_t, 24> broadcast_24{};
    if (options_.broadcast_blocks == 3) {
      for (size_t motor = 0; motor < 3; ++motor) {
        std::copy(command.data.begin(), command.data.end(),
          broadcast_24.begin() + motor * command.data.size());
      }
    }
    const auto period = std::chrono::microseconds(1000000 / options_.rate_hz);
    const auto start = clock_type::now() + std::chrono::milliseconds(10);
    last_reply_ns.store(to_ns(start));
    for (auto & last : last_reply_by_motor_ns) {
      last.store(to_ns(start));
    }
    int skipped = 0;
    int feedback_unavailable = 0;
    bool no_reply = false;
    int timed_out_motor_id = 0;
    for (int i = 0; i < options_.count && !g_stop; ++i) {
      const auto target = start + i * period;
      std::this_thread::sleep_until(target);
      const auto now = clock_type::now();
      const auto now_ns = to_ns(now);
      const auto latest_reply_ns = last_reply_ns.load();
      if (received.load() > 0 && latest_reply_ns <= now_ns) {
        feedback_age_ms.push_back(static_cast<double>(now_ns - latest_reply_ns) / 1e6);
      } else {
        ++feedback_unavailable;
      }
      if (fault.load()) {
        break;
      }
      // Skip missed slots and avoid a burst if a late slot is close to the previous send.
      if (now >= target + period ||
        (!send_times.empty() && now - send_times.back() < period / 2))
      {
        ++skipped;
        continue;
      }
      // An idle interval can exceed the response timeout at low rates. Check only
      // motors whose valid reply count is behind the number of sent commands.
      const auto timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::milliseconds(options_.timeout_ms)).count();
      for (int motor = 0; motor < motor_count; ++motor) {
        if (sent.load() > received_by_motor[motor].load() &&
          now_ns - last_reply_by_motor_ns[motor].load() > timeout_ns)
        {
          no_reply = true;
          timed_out_motor_id = motor + 1;
          break;
        }
      }
      if (no_reply) {
        break;
      }
      const bool sent_ok = options_.broadcast_blocks == 3 ?
        bus_.sendFdBroadcast24(broadcast_24) : bus_.send(command);
      if (!sent_ok) {
        ++send_failures_;
        fault.store(true);
        break;
      }
      send_times.push_back(now);
      sent.fetch_add(1);
    }
    receive_deadline_ns.store(to_ns(clock_type::now() +
      std::chrono::milliseconds(options_.timeout_ms)));
    sending_done.store(true);
    receiver.join();
    receive_bus.close();
    const bool receive_restored = bus_.setReceiveEnabled(true);
    if (received_by_motor[0].load() > 0) {
      last_reply_ = final_reply;
    }

    const auto print_intervals = [&](const char * label, const auto & times) {
        if (times.size() < 2) {
          return;
        }
        std::vector<double> intervals_ms;
        intervals_ms.reserve(times.size() - 1);
        for (size_t i = 1; i < times.size(); ++i) {
          intervals_ms.push_back(
            std::chrono::duration<double, std::milli>(times[i] - times[i - 1]).count());
        }
        const double span_s = std::chrono::duration<double>(times.back() - times.front()).count();
        const double actual_hz = static_cast<double>(times.size() - 1) / span_s;
        const size_t over = static_cast<size_t>(std::count_if(
          intervals_ms.begin(), intervals_ms.end(),
          [&](double ms) {return ms > 1000.0 / options_.rate_hz;}));
        std::sort(intervals_ms.begin(), intervals_ms.end());
        const auto at = [&](double q) {
            return intervals_ms[std::min(intervals_ms.size() - 1,
              static_cast<size_t>(q * intervals_ms.size()))];
          };
        std::printf("[STREAM] %s: 平均 %.1f Hz / 間隔 最小 %.3f ms / 中央 %.3f ms / "
          "99%% %.3f ms / 最大 %.3f ms / 目標周期超過 %zu/%zu\n", label, actual_hz,
          intervals_ms.front(), at(0.5), at(0.99), intervals_ms.back(), over,
          intervals_ms.size());
      };
    std::printf("[STREAM] 予定 %d / 送信 %d / 応答 %d / スキップ %d / 送信失敗 %d\n",
      options_.count, sent.load(), received.load(), skipped, send_failures_);
    if (options_.vcan_three_replies) {
      std::printf("[STREAM] ID別有効応答: 1=%d / 2=%d / 3=%d\n",
        received_by_motor[0].load(), received_by_motor[1].load(),
        received_by_motor[2].load());
    }
    print_intervals("実送信開始", send_times);
    print_intervals(options_.vcan_three_replies ? "ID 1有効応答取込" : "有効応答取込",
      reply_times);
    print_intervals(options_.vcan_three_replies ? "ID 1カーネル受信" : "カーネル受信",
      kernel_reply_times);
    if (options_.hardware_timestamps) {
      std::printf("[STREAM] ハードウェア受信時刻: 取得 %zu/%d（ID 1、同一デバイスのraw時計）\n",
        hardware_reply_times.size(), received_by_motor[0].load());
      print_intervals("ハードウェア受信", hardware_reply_times);
    }
    if (!feedback_age_ms.empty()) {
      const size_t over_one_ms = static_cast<size_t>(std::count_if(
        feedback_age_ms.begin(), feedback_age_ms.end(),
        [](double ms) {return ms > 1.0;}));
      std::sort(feedback_age_ms.begin(), feedback_age_ms.end());
      const auto at = [&](double q) {
          return feedback_age_ms[std::min(feedback_age_ms.size() - 1,
            static_cast<size_t>(q * feedback_age_ms.size()))];
        };
      std::printf("[STREAM] 周期開始時のID 1最新受信経過: %zu件 / 欠測 %d / 最小 %.3f ms / "
        "中央 %.3f ms / 99%% %.3f ms / 最大 %.3f ms / 1 ms超過 %zu/%zu\n",
        feedback_age_ms.size(), feedback_unavailable, feedback_age_ms.front(), at(0.5),
        at(0.99), feedback_age_ms.back(), over_one_ms, feedback_age_ms.size());
    }
    if (!kernel_to_user_us.empty()) {
      std::sort(kernel_to_user_us.begin(), kernel_to_user_us.end());
      std::printf("[STREAM] カーネル受信→取込: %zu件 / 中央 %.1f us / 99%% %.1f us / "
        "最大 %.1f us\n", kernel_to_user_us.size(),
        kernel_to_user_us[kernel_to_user_us.size() / 2],
        kernel_to_user_us[std::min(kernel_to_user_us.size() - 1,
          static_cast<size_t>(0.99 * kernel_to_user_us.size()))], kernel_to_user_us.back());
    }
    printReply("STREAM");
    const bool missing_hardware_timestamps = options_.hardware_timestamps &&
      hardware_reply_times.size() != static_cast<size_t>(received_by_motor[0].load());
    if (missing_hardware_timestamps) {
      std::fprintf(stderr, "ハードウェア受信時刻が欠測しました（取得 %zu/%d）\n",
        hardware_reply_times.size(), received_by_motor[0].load());
    }
    if (no_reply) {
      std::fprintf(stderr, "ID %d の有効応答が %d ms 以上ありません\n",
        timed_out_motor_id, options_.timeout_ms);
    }
    bool missing_motor_replies = false;
    for (int motor = 0; motor < motor_count; ++motor) {
      missing_motor_replies |= received_by_motor[motor].load() != sent.load();
    }
    if (!receive_restored || fault.load() || no_reply || g_stop || sent.load() != options_.count ||
      missing_hardware_timestamps || missing_motor_replies ||
      received.load() != sent.load() * motor_count)
    {
      std::fprintf(stderr, "STREAM診断を停止: 送信・応答数、温度、通信状態を確認してください\n");
      return false;
    }
    return true;
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
    "                 [--count 500] [--rate 200] [--timeout-ms 10] [--broadcast]"
    " [--menu-only] [--stream] [--hardware-timestamps] [--broadcast-blocks 1|3]"
    " [--isolated-id1-24b] [--isolated-id1-24b-sweep]"
    " [--isolated-id1-24b-extended] [--vcan-three-replies]\n"
    "  --broadcast: FD ID 0x7FF with motor 1's 8-byte zero-torque block only\n"
    "  --broadcast-blocks 3: 24-byte zero-gain, zero-torque blocks for IDs 1-3;"
    " vcan or bounded can0 check\n"
    "  --isolated-id1-24b: assert only ID 1 is connected; requires can0, FD without BRS,"
    " <=20 frames at <=50 Hz\n"
    "  --isolated-id1-24b-sweep: with the isolated-ID1 assertion, allow only"
    " 200/500/1000 Hz and <=100 frames\n"
    "  --isolated-id1-24b-extended: with the sweep option, allow exactly"
    " 1000 frames at 1000 Hz\n"
    "  --vcan-three-replies: require one valid response per ID 1-3 for each 24-byte send;"
    " vcan only\n"
    "  --stream: menu-only broadcast diagnosis with scheduled sends and separate RX socket\n"
    "  --hardware-timestamps: request raw RX hardware timestamps in --stream mode\n");
}

std::optional<Options> parse(int argc, char ** argv)
{
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--broadcast") {
      o.broadcast = true;
      continue;
    }
    if (key == "--menu-only") {
      o.menu_only = true;
      continue;
    }
    if (key == "--stream") {
      o.stream = true;
      continue;
    }
    if (key == "--hardware-timestamps") {
      o.hardware_timestamps = true;
      continue;
    }
    if (key == "--isolated-id1-24b") {
      o.isolated_id1_24b = true;
      continue;
    }
    if (key == "--isolated-id1-24b-sweep") {
      o.isolated_id1_24b_sweep = true;
      continue;
    }
    if (key == "--isolated-id1-24b-extended") {
      o.isolated_id1_24b_extended = true;
      continue;
    }
    if (key == "--vcan-three-replies") {
      o.vcan_three_replies = true;
      continue;
    }
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
    } else if (key == "--broadcast-blocks") {
      o.broadcast_blocks = std::atoi(value.c_str());
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
  if (o.can_id < 1 || o.can_id > static_cast<int>(protocol::kMaxSupportedMotorCanId) ||
    o.count < 1 || o.rate_hz < 1 || o.rate_hz > 2000 ||
    o.timeout_ms < 1 || (o.broadcast && (o.can_id != 1 || o.format == CanFrameFormat::kClassic)) ||
    (o.stream && (!o.menu_only || !o.broadcast || o.rate_hz > 1000 || o.count > 1000)) ||
    (o.hardware_timestamps && !o.stream) ||
    (o.broadcast_blocks != 1 && o.broadcast_blocks != 3) ||
    (o.broadcast_blocks == 3 && (!o.stream || !o.menu_only || !o.broadcast ||
    o.can_id != 1 || o.format == CanFrameFormat::kClassic ||
    (o.iface.rfind("vcan", 0) != 0 && !o.isolated_id1_24b))) ||
    (o.isolated_id1_24b && (o.broadcast_blocks != 3 || o.iface != "can0" ||
    o.format != CanFrameFormat::kFd ||
    (!o.isolated_id1_24b_sweep && (o.count > 20 || o.rate_hz > 50)))) ||
    (o.isolated_id1_24b_sweep && (!o.isolated_id1_24b ||
    o.count > (o.isolated_id1_24b_extended ? 1000 : 100) ||
    (o.rate_hz != 200 && o.rate_hz != 500 && o.rate_hz != 1000))) ||
    (o.isolated_id1_24b_extended && (!o.isolated_id1_24b_sweep ||
    o.rate_hz != 1000 || o.count != 1000)) ||
    (o.vcan_three_replies && (o.iface.rfind("vcan", 0) != 0 ||
    o.broadcast_blocks != 3 || o.isolated_id1_24b)))
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
  std::printf("bxi_probe: %s can_id=%d reply=0x%03X count=%d rate=%dHz timeout=%dms "
    "broadcast=%s blocks=%d menu_only=%s stream=%s hw_timestamps=%s"
    " isolated_id1_24b=%s isolated_id1_24b_sweep=%s"
    " isolated_id1_24b_extended=%s vcan_three_replies=%s\n",
    options->iface.c_str(), options->can_id,
    protocol::defaultMasterId(static_cast<uint32_t>(options->can_id)), options->count,
    options->rate_hz, options->timeout_ms, options->broadcast ? "yes" : "no",
    options->broadcast_blocks,
    options->menu_only ? "yes" : "no", options->stream ? "yes" : "no",
    options->hardware_timestamps ? "yes" : "no",
    options->isolated_id1_24b ? "yes" : "no",
    options->isolated_id1_24b_sweep ? "yes" : "no",
    options->isolated_id1_24b_extended ? "yes" : "no",
    options->vcan_three_replies ? "yes" : "no");

  // 0. MENUへ戻す
  std::printf("\n[0] Esc を送って MENU へ戻す\n");
  probe.sendText("\x1b");
  probe.drain(300);
  if (options->menu_only) {
    // On hardware, Esc and disable while already in MENU may not print a STATE line.
    // Require the unicast disable reply and reject an explicit MOTOR state.
    std::printf("[0a] disable の応答を確認\n");
    CanFrame menu_disable_reply;
    const auto disable_us = probe.special(protocol::SpecialCommand::kDisable,
      &menu_disable_reply);
    probe.drain(500);
    std::printf("  disable MIT応答: %s / 状態: %s\n", result(disable_us),
      probe.state().empty() ? "(STATE 行なし)" : probe.state().c_str());
    if (!disable_us || probe.state().find("02 MOTOR") != std::string::npos) {
      std::fprintf(stderr,
        "disable acknowledgement missing or MOTOR state reported; no MIT command was sent\n");
      probe.close();
      return 1;
    }
    if (probe.state().empty()) {
      std::printf("  MENU text was not reported; proceeding after disable acknowledgement "
        "with zero-gain, zero-torque commands only\n");
    }
  }

  // 1. Disable acknowledgement precedes zero-gain control when MENU text is unavailable.
  if (options->menu_only && probe.state().empty()) {
    std::printf("\n[1] disable 応答後、力を出さないMIT指令（MENU文字表示は未確認）\n");
  } else {
    std::printf("\n[1] MENU 状態で、力を出さないMIT指令\n");
  }
  const bool stream_ok = options->stream ? probe.stream() : (probe.latency("MENU"), true);

  if (options->menu_only) {
    if (!stream_ok) {
      CanFrame disable_reply;
      const auto disable_us = probe.special(protocol::SpecialCommand::kDisable, &disable_reply);
      std::printf("STREAM停止後のdisable応答: %s\n", result(disable_us));
    }
    std::printf("MENU-only test %s; enable was not sent\n",
      stream_ok ? "complete" : "stopped");
    probe.sendText("\x1b");
    probe.close();
    return stream_ok ? 0 : 1;
  }

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
