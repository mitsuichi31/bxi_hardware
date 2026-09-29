// One-joint sine test over the same ros2_control interface as sine_command.py.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "bxi_hardware/hil_trajectory.hpp"

namespace {
using Clock = std::chrono::steady_clock;
constexpr double kPi = 3.14159265358979323846;
volatile std::sig_atomic_t interrupted = 0;
void on_signal(int) { interrupted = 1; }

struct Options {
  std::string joint = "joint1";
  double amplitude = 0.1, frequency = 0.5, cycles = 5.0, ramp_cycles = 1.0;
  std::optional<double> center;
  double kp = 20.0, kd = 1.0, rate = 200.0, hold = 1.0;
  double lower = -0.5, upper = 0.5, max_kp = 20.0;
  double max_velocity = 2.0, max_torque = 5.0;
  double friction_ff = 0.0, friction_static = 0.0;
  double friction_velocity = 0.05, stribeck_velocity = 0.2, max_friction_ff = 2.0;
  bool static_given = false, release = false;
  std::string log;
};

void usage() {
  std::cout << "Usage: ros2 run bxi_hardware sine_command_cpp [options]\n"
            << "  --amplitude RAD     sine amplitude (default 0.1)\n"
            << "  --frequency HZ      sine frequency (default 0.5)\n"
            << "  --cycles N          full-amplitude cycles (default 5)\n"
            << "  --ramp-cycles N     rise/fall cycles (default 1)\n"
            << "  --kp N --kd N --rate HZ (defaults 20, 1, 200)\n"
            << "  --release           release torque at the end\n"
            << "  --joint NAME --center RAD --hold SEC --log PATH\n"
            << "  --lower RAD --upper RAD --max-kp N --max-velocity RAD_S --max-torque NM\n"
            << "  --friction-ff NM --friction-static NM --friction-velocity RAD_S\n"
            << "  --stribeck-velocity RAD_S --max-friction-ff NM\n";
}

Options parse(int argc, char **argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string key(argv[i]);
    if (key == "--help" || key == "-h") { usage(); std::exit(0); }
    if (key == "--release") { o.release = true; continue; }
    if (i + 1 >= argc) throw std::runtime_error("Missing value for " + key);
    const std::string value(argv[++i]);
    if (key == "--joint") o.joint = value;
    else if (key == "--log") o.log = value;
    else {
      size_t consumed = 0;
      double n = std::stod(value, &consumed);
      if (consumed != value.size() || !std::isfinite(n)) throw std::runtime_error("Invalid number for " + key);
      if (key == "--amplitude") o.amplitude = n;
      else if (key == "--frequency") o.frequency = n;
      else if (key == "--cycles") o.cycles = n;
      else if (key == "--ramp-cycles") o.ramp_cycles = n;
      else if (key == "--center") o.center = n;
      else if (key == "--kp") o.kp = n;
      else if (key == "--kd") o.kd = n;
      else if (key == "--rate") o.rate = n;
      else if (key == "--hold") o.hold = n;
      else if (key == "--lower") o.lower = n;
      else if (key == "--upper") o.upper = n;
      else if (key == "--max-kp") o.max_kp = n;
      else if (key == "--max-velocity") o.max_velocity = n;
      else if (key == "--max-torque") o.max_torque = n;
      else if (key == "--friction-ff") o.friction_ff = n;
      else if (key == "--friction-static") { o.friction_static = n; o.static_given = true; }
      else if (key == "--friction-velocity") o.friction_velocity = n;
      else if (key == "--stribeck-velocity") o.stribeck_velocity = n;
      else if (key == "--max-friction-ff") o.max_friction_ff = n;
      else throw std::runtime_error("Unknown option: " + key);
    }
  }
  if (!o.static_given) o.friction_static = o.friction_ff;
  if (!(o.amplitude > 0 && o.amplitude <= 0.5 && o.frequency > 0 && o.frequency <= 5 &&
        o.cycles >= 0 && o.ramp_cycles >= 0 && o.hold >= 0 &&
        o.kp > 0 && o.kp <= o.max_kp && o.kd >= 0 && o.kd <= 5 &&
        o.rate >= 10 && o.rate <= 500 && o.lower < o.upper && o.max_torque > 0 &&
        o.max_velocity > 0 && o.friction_ff >= 0 &&
        o.friction_ff <= o.friction_static && o.friction_static <= o.max_friction_ff &&
        o.friction_velocity > 0 && o.stribeck_velocity > 0)) {
    throw std::runtime_error("Options outside allowed range (see --help)");
  }
  // An upper bound over the whole trajectory, including the envelope derivative while the
  // amplitude ramps.
  const double bound = bxi_hardware::hil::SineTrajectory(
    0.0, o.amplitude, o.frequency, o.cycles, o.ramp_cycles).velocityUpperBound();
  if (bound > o.max_velocity)
    throw std::runtime_error(
      "Sine velocity bound (including the amplitude ramps) exceeds --max-velocity");
  return o;
}

// Targets come from bxi_hardware::hil::SineTrajectory (the same formulas as sine_command.py);
// the other fields are for summarize().
struct Trajectory {
  double center, amplitude, omega, ramp, flat, duration;
  bxi_hardware::hil::SineTrajectory sine;
  explicit Trajectory(const Options &o, double c)
    : center(c), amplitude(o.amplitude), omega(2 * kPi * o.frequency),
      ramp(o.ramp_cycles / o.frequency), flat(o.cycles / o.frequency),
      duration(2 * ramp + flat),
      sine(c, o.amplitude, o.frequency, o.cycles, o.ramp_cycles) {}

  std::pair<double, double> at(double t) const {
    const auto s = sine.at(t);
    return {s.position, s.velocity};
  }
};

class SineNode : public rclcpp::Node {
public:
  explicit SineNode(const std::string &joint) : Node("bxi_sine_command_cpp"), joint_(joint) {
    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/joint1_command/commands", 10);
    sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        const auto it = std::find(msg->name.begin(), msg->name.end(), joint_);
        if (it == msg->name.end()) return;
        const auto i = static_cast<size_t>(it - msg->name.begin());
        if (i >= msg->position.size() || !std::isfinite(msg->position[i])) return;
        position = msg->position[i];
        velocity = i < msg->velocity.size() ? msg->velocity[i] : std::numeric_limits<double>::quiet_NaN();
        effort = i < msg->effort.size() ? msg->effort[i] : std::numeric_limits<double>::quiet_NaN();
        stamp = Clock::now();
      });
  }
  void send(double p, double v, double kp, double kd, double ff = 0) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {p, v, kp, kd, ff};
    pub_->publish(msg);
  }
  std::optional<double> position;
  double velocity = std::numeric_limits<double>::quiet_NaN();
  double effort = std::numeric_limits<double>::quiet_NaN();
  Clock::time_point stamp{};
private:
  std::string joint_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

struct Row {
  double t, target, target_velocity, friction_ff, position, velocity, effort, lateness_ms;
};

double seconds(Clock::duration d) { return std::chrono::duration<double>(d).count(); }

std::string default_log() {
  const auto base = std::filesystem::is_directory("/ws/log") ? "/ws/log" : ".";
  const std::time_t now = std::time(nullptr);
  std::tm local{};
  localtime_r(&now, &local);
  char name[64];
  std::strftime(name, sizeof(name), "sine_cpp_%Y%m%d_%H%M%S.csv", &local);
  return (std::filesystem::path(base) / name).string();
}

void write_log(const std::string &path, const std::vector<Row> &rows) {
  const std::filesystem::path p(path);
  if (!p.parent_path().empty()) std::filesystem::create_directories(p.parent_path());
  std::ofstream out(path);
  if (!out) throw std::runtime_error("Cannot write CSV: " + path);
  out << "t,target,target_velocity,friction_ff,position,velocity,effort,lateness_ms\n" << std::setprecision(12);
  for (const auto &r : rows)
    out << r.t << ',' << r.target << ',' << r.target_velocity << ',' << r.friction_ff << ','
        << r.position << ',' << r.velocity << ',' << r.effort << ',' << r.lateness_ms << '\n';
  if (!out) throw std::runtime_error("CSV write failed: " + path);
}

void summarize(const std::vector<Row> &rows, const Trajectory &trajectory) {
  double m[3][4]{};
  double square_error = 0, max_error = 0;
  size_t count = 0;
  for (const auto &r : rows) {
    if (r.t < trajectory.ramp || r.t > trajectory.ramp + trajectory.flat) continue;
    const double b[3] = {1, std::sin(trajectory.omega * r.t), std::cos(trajectory.omega * r.t)};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) m[i][j] += b[i] * b[j];
      m[i][3] += b[i] * r.position;
    }
    const double e = r.position - r.target;
    square_error += e * e;
    max_error = std::max(max_error, std::abs(e));
    ++count;
  }
  if (count < 10) return;
  std::cout << std::fixed << std::setprecision(4)
            << "Flat interval: " << count << " samples, error RMS="
            << std::sqrt(square_error / count) << " rad, max=" << max_error << " rad";
  for (int i = 0; i < 3; ++i) {
    int pivot = i;
    for (int j = i + 1; j < 3; ++j)
      if (std::abs(m[j][i]) > std::abs(m[pivot][i])) pivot = j;
    if (std::abs(m[pivot][i]) < 1e-12) { std::cout << '\n'; return; }
    for (int k = i; k < 4; ++k) std::swap(m[i][k], m[pivot][k]);
    const double divisor = m[i][i];
    for (int k = i; k < 4; ++k) m[i][k] /= divisor;
    for (int j = 0; j < 3; ++j) {
      if (j == i) continue;
      const double factor = m[j][i];
      for (int k = i; k < 4; ++k) m[j][k] -= factor * m[i][k];
    }
  }
  const double ratio = std::hypot(m[1][3], m[2][3]) / trajectory.amplitude;
  const double lag_deg = -std::atan2(m[2][3], m[1][3]) * 180 / kPi;
  std::cout << ", amplitude ratio=" << std::setprecision(3) << ratio
            << ", phase lag=" << std::setprecision(1) << lag_deg << " deg"
            << ", center offset=" << std::setprecision(4) << m[0][3] - trajectory.center
            << " rad\n";
}

int run(const Options &o) {
  auto node = std::make_shared<SineNode>(o.joint);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto wait_end = Clock::now() + std::chrono::seconds(2);
  while (!node->position && Clock::now() < wait_end && !interrupted) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  if (!node->position) throw std::runtime_error("/joint_states not received within 2 s");
  const double start = *node->position, center = o.center.value_or(start);
  if (center - o.amplitude < o.lower || center + o.amplitude > o.upper)
    throw std::runtime_error("Center +/- amplitude outside joint limits");
  if (std::abs(center - start) > 0.001)
    throw std::runtime_error("Center differs from current position; move with ramp_command.py first");

  const Trajectory trajectory(o, center);
  const std::string log = o.log.empty() ? default_log() : o.log;
  std::cout << "center=" << center << " rad, amplitude=" << o.amplitude << " rad, frequency="
            << o.frequency << " Hz, duration=" << trajectory.duration << " s, rate=" << o.rate
            << " Hz, log=" << log << std::endl;

  const auto period = std::chrono::duration<double>(1.0 / o.rate);
  const auto t0 = Clock::now();
  auto deadline = t0;
  std::vector<Row> rows;
  rows.reserve(static_cast<size_t>((trajectory.duration + o.hold) * o.rate) + 2);
  std::string aborted;
  size_t missed_deadlines = 0;
  double worst_lateness_ms = 0;
  while (!interrupted) {
    executor.spin_some();
    const auto now = Clock::now();
    const double t = seconds(now - t0);
    if (t > trajectory.duration + o.hold) break;
    if (seconds(now - node->stamp) > 0.2) {
      aborted = "/joint_states stale for over 0.2 s";
      break;
    }
    const auto [target, target_velocity] = trajectory.at(t);
    const double ff = bxi_hardware::hil::frictionTorque(
      target_velocity, o.friction_ff, o.friction_velocity, o.friction_static, o.stribeck_velocity);
    if (std::abs(target_velocity) > o.max_velocity) {
      aborted = "Target velocity exceeds --max-velocity";
      break;
    }
    if (o.kp * std::abs(target - *node->position) + std::abs(ff) > o.max_torque) {
      aborted = "Estimated command torque exceeds --max-torque";
      break;
    }
    const double lateness_ms = std::max(0.0, seconds(now - deadline) * 1000.0);
    worst_lateness_ms = std::max(worst_lateness_ms, lateness_ms);
    node->send(target, target_velocity, o.kp, o.kd, ff);
    rows.push_back({t, target, target_velocity, ff, *node->position,
                    node->velocity, node->effort, lateness_ms});
    deadline += std::chrono::duration_cast<Clock::duration>(period);
    while (deadline <= Clock::now()) {
      deadline += std::chrono::duration_cast<Clock::duration>(period);
      ++missed_deadlines;
    }
    std::this_thread::sleep_until(deadline);
  }
  if (interrupted) aborted = "Interrupted";
  if (!aborted.empty() || o.release) {
    for (int i = 0; i < 5; ++i) {
      node->send(*node->position, 0, 0, 0);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  try { write_log(log, rows); }
  catch (const std::exception &e) { std::cerr << "CSV warning: " << e.what() << '\n'; }
  std::cout << "Samples=" << rows.size() << ", missed deadlines=" << missed_deadlines
            << ", worst lateness=" << worst_lateness_ms << " ms\n";
  if (!aborted.empty()) std::cerr << "Aborted: " << aborted << "; sent zero-gain release\n";
  else {
    summarize(rows, trajectory);
    std::cout << "Finished at " << *node->position << " rad"
              << (o.release ? "; released\n" : "; holding center\n");
  }
  return aborted.empty() ? 0 : 1;
}
}  // namespace

int main(int argc, char **argv) {
  try {
    const auto options = parse(argc, argv);
    rclcpp::init(argc, argv);
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    const int result = run(options);
    rclcpp::shutdown();
    return result;
  } catch (const std::exception &e) {
    std::cerr << "sine_command_cpp: " << e.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
