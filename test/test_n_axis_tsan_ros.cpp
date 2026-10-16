#include <gtest/gtest.h>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>
#include "controller_interface/controller_interface_params.hpp"
#include "bxi_hardware/n_axis_trajectory_controller.hpp"

namespace bxi_hardware
{
namespace
{
using C = NAxisTrajectoryController;
using CB = controller_interface::CallbackReturn;
using Ret = controller_interface::return_type;
constexpr uint64_t kCycles = 100000, kHearts = 10000, kFlood = 100;
constexpr int64_t kNow = 2000000000;
thread_local bool update_thread = false;
uint64_t threadId() {return std::hash<std::thread::id>{}(std::this_thread::get_id());}

enum class Mode {Release, Finish, Lease, Stop};
const char * label(Mode mode)
{
  switch (mode) {
    case Mode::Release: return "release";
    case Mode::Finish: return "finish";
    case Mode::Lease: return "lease";
    case Mode::Stop: return "stop";
  }
  return "unknown";
}
struct TemporaryDirectory
{
  std::string path;
  TemporaryDirectory()
  {
    auto pattern = (std::filesystem::temp_directory_path() / "bxi_ros_tsan_XXXXXX").string();
    const auto result = ::mkdtemp(pattern.data());
    if (!result) {throw std::runtime_error("mkdtemp failed");}
    path = result;
  }
  ~TemporaryDirectory() {std::error_code error; std::filesystem::remove_all(path, error);}
};
class FakeController : public C
{
public:
  size_t axes = 0;
  const uint64_t lifecycle_thread = threadId();
  std::atomic<int64_t> now{kNow};
  std::array<double, 16> commands{};  // update thread only, except activation before it starts
  std::atomic<bool> in_update{false};
  mutable std::atomic<uint64_t> rt_id{0}, callback_id{0}, rt_clock_calls{0};
  mutable std::atomic<uint64_t> callback_clock_calls{0}, callback_during_update{0};
  std::atomic<uint64_t> setters{0};
protected:
  int64_t steadyNow() const noexcept override
  {
    // Relaxed observations introduce no extra record handover happens-before edge.
    const auto id = threadId();
    if (update_thread) {
      rt_id.store(id, std::memory_order_relaxed);
      rt_clock_calls.fetch_add(1, std::memory_order_relaxed);
    } else if (id != lifecycle_thread) {
      callback_id.store(id, std::memory_order_relaxed);
      callback_clock_calls.fetch_add(1, std::memory_order_relaxed);
      if (in_update.load(std::memory_order_relaxed)) {
        callback_during_update.fetch_add(1, std::memory_order_relaxed);
      }
    }
    return now.load(std::memory_order_relaxed);
  }
  bool setCommand(size_t i, double value) override
  {commands[i] = value; setters.fetch_add(1, std::memory_order_relaxed); return true;}
  std::optional<double> getCommand(size_t i) override {return commands[i];}
  std::optional<double> getState(size_t i) override
  {
    if (i == axes * 3) {return commands[axes * 5];}
    const auto axis = i / 3, field = i % 3;
    return field == 0 ? commands[axis * 5] : field == 1 ? commands[axis * 5 + 1] : 0.;
  }
};
struct Rig
{
  TemporaryDirectory directory;
  const size_t n;
  const Mode mode;
  std::string name;
  std::array<double, 16> backing{};
  std::vector<hardware_interface::CommandInterface> commands;
  std::vector<hardware_interface::StateInterface> states;
  std::unique_ptr<FakeController> c;
  rclcpp::Node::SharedPtr client_node;
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions(), 3};
  rclcpp::Client<srv::StartNAxis>::SharedPtr start;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr finish, stop;
  rclcpp::Publisher<msg::NAxisKeepalive>::SharedPtr publisher;
  rclcpp::Subscription<msg::NAxisStatus>::SharedPtr status_sub;
  std::mutex observation_mutex, lifecycle_gate;
  msg::NAxisStatus latest;
  std::atomic<uint64_t> status_count{0}, status_thread{0}, statuses_during_update{0};
  std::atomic<uint64_t> cycles{0}, post_cycles{0}, errors{0}, unexpected{0}, hearts{0};
  std::atomic<uint64_t> service_calls{0}, start_accepted{0}, start_rejected{0};
  std::atomic<uint64_t> finish_calls{0}, finish_accepted{0}, stop_calls{0};
  std::atomic<bool> finish_threads{false}, parked{false}, post{false}, stop_publisher{false};
  std::thread spinner, rt, producer;
  bool closed = false;

  Rig(size_t axes, Mode value) : n(axes), mode(value), c(std::make_unique<FakeController>())
  {
    name = "g2a_ros_" + std::to_string(n) + "_" + label(mode);
    std::cout << "G2aROS BEGIN N=" << n << " mode=" << label(mode) << std::endl;
    c->axes = n;
    std::vector<std::string> joints;
    for (size_t a = 0; a < n; ++a) {joints.push_back("joint" + std::to_string(a + 1));}
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("joints", joints),
      rclcpp::Parameter("output_dir", directory.path), rclcpp::Parameter("record_seconds", 110.),
      rclcpp::Parameter("status_period", .02)});
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = name; params.controller_manager_update_rate = 1000;
    params.update_rate = 1000; params.node_options = options;
    if (c->init(params) != Ret::OK) {throw std::runtime_error("init failed");}
    client_node = rclcpp::Node::make_shared(name + "_client");
    const auto prefix = "/" + name + "/n_axis/";
    start = client_node->create_client<srv::StartNAxis>(prefix + "start");
    finish = client_node->create_client<std_srvs::srv::Trigger>(prefix + "finish");
    stop = client_node->create_client<std_srvs::srv::Trigger>(prefix + "stop");
    publisher = client_node->create_publisher<msg::NAxisKeepalive>(prefix + "keepalive", rclcpp::QoS(1).reliable());
    status_sub = client_node->create_subscription<msg::NAxisStatus>(prefix + "status", rclcpp::QoS(1).reliable(),
      [this](msg::NAxisStatus::ConstSharedPtr status) {
        std::lock_guard<std::mutex> lock(observation_mutex);
        latest = *status;
        status_thread.store(threadId(), std::memory_order_relaxed);
        status_count.fetch_add(1, std::memory_order_relaxed);
        if (c->in_update.load(std::memory_order_relaxed)) {
          statuses_during_update.fetch_add(1, std::memory_order_relaxed);
        }
      });
    // Exercise configure/entity creation while the real executor is already spinning.
    executor.add_node(c->get_node()->get_node_base_interface()); executor.add_node(client_node);
    spinner = std::thread([this] {executor.spin();});
    if (c->on_configure(rclcpp_lifecycle::State()) != CB::SUCCESS) {throw std::runtime_error("configure failed");}
    commands.reserve(16); states.reserve(10);
    std::vector<hardware_interface::LoanedCommandInterface> cl;
    std::vector<hardware_interface::LoanedStateInterface> sl;
    for (const auto & key : c->command_interface_configuration().names) {
      const auto i = key.find('/');
      commands.emplace_back(key.substr(0, i), key.substr(i + 1), &backing[commands.size()]); cl.emplace_back(commands.back());
    }
    for (const auto & key : c->state_interface_configuration().names) {
      const auto i = key.find('/');
      states.emplace_back(key.substr(0, i), key.substr(i + 1), &backing[states.size()]); sl.emplace_back(states.back());
    }
    c->assign_interfaces(std::move(cl), std::move(sl));
    if (!wait([this] {return status_count.load(std::memory_order_relaxed) > 0 &&
      start->service_is_ready() && finish->service_is_ready() && stop->service_is_ready() && publisher->get_subscription_count() > 0;})) {
      throw std::runtime_error("ROS discovery timed out");
    }
    // Lifecycle is concurrent with ROS callbacks, never with an in-flight update.
    if (c->on_activate(rclcpp_lifecycle::State()) != CB::SUCCESS) {throw std::runtime_error("activate failed");}
  }
  ~Rig() {close();}
  template<class Predicate> bool wait(Predicate predicate)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    do {
      if (predicate()) {return true;}
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (std::chrono::steady_clock::now() < end);
    return false;
  }
  msg::NAxisStatus status()
  {std::lock_guard<std::mutex> lock(observation_mutex); return latest;}
  bool request()
  {
    auto r = std::make_shared<srv::StartNAxis::Request>();
    r->nonce = 123 + n; r->id = 7; r->axes = n; r->sent_ns = kNow;
    r->amplitude = C::kPi / 6; r->frequency = .2; r->cycles = 5; r->ramp_cycles = 1; r->hold = 1;
    r->kp.fill(20.); r->kd.fill(1.); r->release = mode == Mode::Release;
    service_calls.fetch_add(1, std::memory_order_relaxed);
    auto future = start->async_send_request(r);
    if (!wait([&] {return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;})) {throw std::runtime_error("start timeout");}
    const bool accepted = future.get()->accepted;
    (accepted ? start_accepted : start_rejected).fetch_add(1, std::memory_order_relaxed);
    return accepted;
  }
  bool trigger(bool stop_request)
  {
    service_calls.fetch_add(1, std::memory_order_relaxed);
    (stop_request ? stop_calls : finish_calls).fetch_add(1, std::memory_order_relaxed);
    auto future = (stop_request ? stop : finish)->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (!wait([&] {return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;})) {throw std::runtime_error("trigger timeout");}
    const bool accepted = future.get()->success;
    if (!stop_request && accepted) {finish_accepted.fetch_add(1, std::memory_order_relaxed);}
    return accepted;
  }
  void startWorkers()
  {
    rt = std::thread([this] {
      update_thread = true;
      const auto period = rclcpp::Duration::from_nanoseconds(mode == Mode::Lease ? 1000 : 1000000);
      for (uint64_t i = 0; i < kCycles && !finish_threads.load(std::memory_order_relaxed); ++i) {
        {
          std::lock_guard<std::mutex> lock(lifecycle_gate);
          c->in_update.store(true, std::memory_order_relaxed);
          const auto result = c->update(rclcpp::Time(c->now.load(std::memory_order_relaxed)), period);
          c->in_update.store(false, std::memory_order_relaxed);
          if (result != Ret::OK) {unexpected.fetch_add(1, std::memory_order_relaxed);}
          cycles.fetch_add(1, std::memory_order_relaxed);
        }
        if (i % 64 == 0) {std::this_thread::sleep_for(std::chrono::microseconds(50));}
      }
      parked.store(true, std::memory_order_relaxed);
      while (!post.load(std::memory_order_relaxed) && !finish_threads.load(std::memory_order_relaxed)) {std::this_thread::yield();}
      if (mode == Mode::Lease) {c->now.store(kNow + C::kLeaseNs, std::memory_order_relaxed);}
      // Continue calling update during saveRecord, without joining at the handover.
      for (uint64_t i = 0; i < kCycles && !finish_threads.load(std::memory_order_relaxed); ++i) {
        std::lock_guard<std::mutex> lock(lifecycle_gate);
        c->in_update.store(true, std::memory_order_relaxed);
        const auto result = c->update(rclcpp::Time(c->now.load(std::memory_order_relaxed)), period);
        c->in_update.store(false, std::memory_order_relaxed);
        if (result == Ret::ERROR) {errors.fetch_add(1, std::memory_order_relaxed);}
        else {unexpected.fetch_add(1, std::memory_order_relaxed);}
        post_cycles.fetch_add(1, std::memory_order_relaxed);
      }
    });
  }
  void heartbeats()
  {
    producer = std::thread([this] {
      for (uint64_t i = 1; i <= kHearts && !stop_publisher.load(std::memory_order_relaxed); ++i) {
        msg::NAxisKeepalive message; message.nonce = 123 + n; message.id = 7; message.sequence = i; message.sent_ns = kNow;
        publisher->publish(message); hearts.fetch_add(1, std::memory_order_relaxed);
        if (i % 32 == 0) {std::this_thread::sleep_for(std::chrono::microseconds(100));}
      }
    });
  }
  void deactivate()
  {
    std::lock_guard<std::mutex> lock(lifecycle_gate);
    EXPECT_EQ(c->on_deactivate(rclcpp_lifecycle::State()), CB::SUCCESS);
  }
  void close()
  {
    if (closed) {return;}
    closed = true;
    stop_publisher.store(true, std::memory_order_relaxed);
    finish_threads.store(true, std::memory_order_relaxed);
    if (producer.joinable()) {producer.join();}
    if (rt.joinable()) {rt.join();}
    // Cancel while requests/status can still be pending, then drain all running callbacks.
    executor.cancel(); if (spinner.joinable()) {spinner.join();}
    executor.remove_node(c->get_node()->get_node_base_interface()); executor.remove_node(client_node);
    const auto final = status();
    const auto count = c->rowCountAfterStop();
    if (!final.csv_path.empty() && std::filesystem::is_regular_file(final.csv_path)) {
      const char * out = std::getenv("BXI_N_AXIS_ROS_TSAN_OUTPUT_DIR");
      if (out) {
        std::filesystem::create_directories(out);
        std::filesystem::copy_file(final.csv_path, std::filesystem::path(out) / (name + ".csv"), std::filesystem::copy_options::overwrite_existing);
      }
      std::ifstream csv(final.csv_path); std::string line; uint64_t lines = 0;
      while (std::getline(csv, line)) {if (!line.empty() && line[0] != '#') {++lines;}}
      EXPECT_EQ(lines, count + 1); EXPECT_EQ(c->droppedAfterStop(), 0U);
    }
    std::cout << "G2aROS N=" << n << " mode=" << label(mode) << " updates=" << cycles.load()
      << " post_updates=" << post_cycles.load() << " errors=" << errors.load() << " heart_published=" << hearts.load()
      << " statuses=" << status_count.load() << " services=" << service_calls.load()
      << " start_accepted=" << start_accepted.load() << " start_rejected=" << start_rejected.load()
      << " finish_calls=" << finish_calls.load() << " finish_accepted=" << finish_accepted.load()
      << " stop_calls=" << stop_calls.load() << " callback_clock_calls=" << c->callback_clock_calls.load()
      << " callback_during_update=" << c->callback_during_update.load() << " status_during_update=" << statuses_during_update.load()
      << " rt_tid=" << c->rt_id.load() << " callback_tid=" << c->callback_id.load()
      << " status_tid=" << status_thread.load() << " lifecycle_tid=" << c->lifecycle_thread
      << " csv_rows=" << count << " record_state=" << static_cast<unsigned>(final.record_state)
      << " executor_drained_before_destroy=1" << std::endl;
    c.reset();  // callback targets die only after executor join/remove, loans still alive
  }
};
class RosParallel : public ::testing::TestWithParam<size_t>
{
public:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};
void runCase(size_t n, Mode mode)
{
  Rig r(n, mode);
  r.startWorkers();
  ASSERT_TRUE(r.wait([&] {return r.status().state == msg::NAxisStatus::IDLE;}));
  ASSERT_TRUE(r.request()); r.heartbeats();
  if (mode == Mode::Finish) {
    ASSERT_TRUE(r.wait([&] {return r.status().state == msg::NAxisStatus::HOLDING;}));
    EXPECT_TRUE(r.trigger(false));
    for (uint64_t i = 1; i < kFlood; ++i) {r.trigger(false); EXPECT_FALSE(r.request());}
  } else if (mode == Mode::Stop) {
    for (uint64_t i = 0; i < kFlood; ++i) {EXPECT_FALSE(r.request()); r.trigger(false); EXPECT_TRUE(r.trigger(true));}
  }
  ASSERT_TRUE(r.wait([&] {return r.parked.load(std::memory_order_relaxed);}));
  if (r.producer.joinable()) {r.producer.join();}
  ASSERT_EQ(r.unexpected.load(), 0U);
  ASSERT_EQ(r.cycles.load(), kCycles);
  ASSERT_EQ(r.hearts.load(), kHearts);
  ASSERT_GT(r.c->callback_clock_calls.load(), 0U);
  ASSERT_GT(r.c->callback_during_update.load(), 0U);
  ASSERT_NE(r.c->rt_id.load(), r.c->callback_id.load());
  ASSERT_NE(r.c->rt_id.load(), r.c->lifecycle_thread);
  if (mode != Mode::Lease) {
    const auto expected = mode == Mode::Stop ? msg::NAxisStatus::STOPPED : msg::NAxisStatus::RELEASED;
    ASSERT_TRUE(r.wait([&] {return r.status().state == expected;}));
    EXPECT_FALSE(r.status().record_complete);
    r.deactivate();  // concurrent with ROS group; serialized against update only
  }
  r.post.store(true, std::memory_order_relaxed);
  ASSERT_TRUE(r.wait([&] {return r.status().record_state == msg::NAxisStatus::RECORD_WRITTEN;}));
  ASSERT_TRUE(r.wait([&] {return r.post_cycles.load(std::memory_order_relaxed) == kCycles;}));
  EXPECT_EQ(r.errors.load(), kCycles); EXPECT_EQ(r.unexpected.load(), 0U);
  EXPECT_TRUE(r.status().record_complete); EXPECT_TRUE(r.status().terminal);
  EXPECT_EQ(r.status().state, mode == Mode::Lease ? msg::NAxisStatus::ABORTED : mode == Mode::Stop ? msg::NAxisStatus::STOPPED : msg::NAxisStatus::RELEASED);
  EXPECT_FALSE(r.request()); EXPECT_FALSE(r.trigger(false));
  EXPECT_GT(r.status_count.load(), 1U);
  // Queue requests immediately before executor cancellation to exercise drained teardown.
  for (int i = 0; i < 8; ++i) {
    r.stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    r.stop_calls.fetch_add(1); r.service_calls.fetch_add(1);
  }
  r.close();
}
TEST_P(RosParallel, AutomaticReleaseHandover) {runCase(GetParam(), Mode::Release);}
TEST_P(RosParallel, FinishFloodHandover) {runCase(GetParam(), Mode::Finish);}
TEST_P(RosParallel, LeaseErrorHandover) {runCase(GetParam(), Mode::Lease);}
TEST_P(RosParallel, StopAndRejectedStartFlood) {runCase(GetParam(), Mode::Stop);}
INSTANTIATE_TEST_SUITE_P(TwoAndThree, RosParallel, ::testing::Values(2U, 3U));
}  // namespace
}  // namespace bxi_hardware
