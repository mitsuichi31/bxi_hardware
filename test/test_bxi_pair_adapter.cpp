#include <gtest/gtest.h>
#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <limits>
#include <new>
#include <filesystem>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include "bxi_hardware/bxi_pair_adapter.hpp"
#include "controller_interface/controller_interface_params.hpp"
namespace
{
thread_local bool measuring = false;
thread_local size_t allocations = 0;
}
[[gnu::noinline]] void * operator new(size_t allocation_size)
{
  if (measuring) ++allocations;
  if (void * allocation = std::malloc(allocation_size ? allocation_size : 1)) return allocation;
  throw std::bad_alloc();
}

[[gnu::noinline]] void operator delete(void * controller_parameters) noexcept
{
  std::free(controller_parameters);
}

[[gnu::noinline]] void operator delete(void * controller_parameters, size_t) noexcept
{
  std::free(controller_parameters);
}

namespace
{
using CB = controller_interface::CallbackReturn;
using Ret = controller_interface::return_type;
struct Controller : bxi_hardware::BxiPairAdapter
{
  int fail_set = -1, fail_read = -1;
  size_t sets = 0, reads = 0, states = 0;
  bool setCommand(size_t i, double value) override
  {
    ++sets;
    if (static_cast<int>(i) == fail_set) return false;
    return BxiPairAdapter::setCommand(i, value);
  }

  std::optional<double> getCommand(size_t i) override
  {
    ++reads;
    if (static_cast<int>(i) == fail_read) return 999.;
    return BxiPairAdapter::getCommand(i);
  }

  std::optional<double> getState(size_t i) override
  {
    ++states;
    return BxiPairAdapter::getState(i);
  }

};
struct Rig
{
  std::array<double, 64> commands{};
  std::array<double, 52> state{};
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  std::vector<hardware_interface::StateInterface> state_interfaces;
  std::vector<hardware_interface::CommandInterface::SharedPtr> references;
  Controller controller;
  size_t hardware_count, axes_count;
  Rig(size_t hardware = 1, size_t axes = 3, int mutation = -1, bool state_mutation = false,
    bool velocity_feedforward = false, const std::string & output = "")
  :hardware_count(hardware), axes_count(axes)
  {
    std::vector<rclcpp::Parameter> params;
    std::vector<std::string> groups;
    for (size_t h = 0; h < hardware_count; ++h) {
      const auto group_name = "h" + std::to_string(h);
      groups.push_back(group_name);
      std::vector<std::string> joint_names;
      params.emplace_back("groups." + group_name + ".gpio", "pair_" + std::to_string(h));
      for (size_t a = 0; a < axes_count; ++a) {
        const auto joint_name = "leg" + std::to_string(h) + "_joint" + std::to_string(a);
        joint_names.push_back(joint_name);
        const std::string parameter_prefix = "joint_settings." + joint_name + ".";
        for (auto parameter_entry: std::vector<std::pair<std::string, double>>{{"position_min",
            -1.},
            {"position_max", 1.}, {"velocity_limit", .7}, {"kp", 10.}, {"kd", 1.}, {"kp_max", 20.},
            {"kd_max", 1.}, {"max_target_delta_per_cycle", .001}, {"torque_limit", 5.},
            {"deviation_limit_rad", .7}, {"temperature_limit_c", 75.}})
          params.emplace_back(parameter_prefix + parameter_entry.first, parameter_entry.second);
        params.emplace_back(parameter_prefix + "deviation_cycles", 15);
      }
      params.emplace_back("groups." + group_name + ".joints", joint_names);
    }
    params.emplace_back("velocity_feedforward", velocity_feedforward);
    params.emplace_back("output_dir", output);
    params.emplace_back("hardware_groups", groups);
    params.emplace_back("record_seconds", .02);
    controller_interface::ControllerInterfaceParams controller_parameters;
    controller_parameters.controller_name = "adapter_unit";
    controller_parameters.update_rate = 1000;
    controller_parameters.node_options.parameter_overrides(params);
    EXPECT_EQ(controller.init(controller_parameters), Ret::OK);
    EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State()), CB::SUCCESS);
    auto command_keys = controller.command_interface_configuration().names,
      state_keys = controller.state_interface_configuration().names;
    auto mutate = [&](auto & keys){
        if (mutation == 0) keys.pop_back();
        if (mutation == 1) keys.push_back("extra/position");
        if (mutation == 2) keys[0] = keys[1];
        if (mutation == 5) std::reverse(keys.begin(), keys.end());
      };
    if (state_mutation) mutate(state_keys);
    else mutate(command_keys);
    command_interfaces.reserve(65);
    state_interfaces.reserve(53);
    std::vector<hardware_interface::LoanedCommandInterface> command_loans;
    std::vector<hardware_interface::LoanedStateInterface> state_loans;
    const auto original_ck = controller.command_interface_configuration().names,
      original_sk = controller.state_interface_configuration().names;
    for (size_t i = 0; i < original_sk.size();
      ++i)
      if (original_sk[i].find("/temperature") != std::string::npos) state[i] = 25.;
    for (size_t i = 0; i < command_keys.size(); ++i) {
      const auto split = command_keys[i].find('/');
      const auto logical = std::find(original_ck.begin(), original_ck.end(),
          command_keys[i]) - original_ck.begin();
      if (mutation == 4 && !state_mutation &&
        i == 0)
        command_interfaces.emplace_back(command_keys[i].substr(0, split),
            command_keys[i].substr(split + 1), "bool");
      else command_interfaces.emplace_back(command_keys[i].substr(0, split),
            command_keys[i].substr(split + 1), &commands[std::min<size_t>(logical, 63)]);
      command_loans.emplace_back(command_interfaces.back());
    }
    for (size_t i = 0; i < state_keys.size(); ++i) {
      const auto split = state_keys[i].find('/');
      const auto logical = std::find(original_sk.begin(), original_sk.end(),
          state_keys[i]) - original_sk.begin();
      if (mutation == 4 && state_mutation &&
        i == 0)
        state_interfaces.emplace_back(state_keys[i].substr(0,
            split), state_keys[i].substr(split + 1), "bool");
      else state_interfaces.emplace_back(state_keys[i].substr(0, split),
            state_keys[i].substr(split + 1), &state[std::min<size_t>(logical, 51)]);
      state_loans.emplace_back(state_interfaces.back());
    }
    controller.assign_interfaces(std::move(command_loans), std::move(state_loans));
    references = controller.export_reference_interfaces();
  }

  CB activate()
  {
    return controller.on_activate(rclcpp_lifecycle::State());
  }

  Ret step(int64_t period_ns = 1000000)
  {
    return controller.update(rclcpp::Time(1000000000),
        rclcpp::Duration::from_nanoseconds(period_ns));
  }

  void ack()
  {
    for (size_t h = 0; h < hardware_count;
      ++h)
      state[h * (4 * axes_count + 1) + 4 * axes_count] = commands[h * (5 * axes_count + 1) +
          5 * axes_count];
  }

};
class Adapter : public ::testing::Test {
public:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

};
TEST_F(Adapter, OneThroughFourGroupsTwoOrThreeAxesAndBootRelease)
{
  for (size_t hardware_count = 1; hardware_count <= 4;
    ++hardware_count)
    for (size_t axes_count: {2U, 3U}) {
      Rig rig(hardware_count, axes_count);
      ASSERT_EQ(rig.activate(), CB::SUCCESS);
      EXPECT_EQ(rig.references.size(), hardware_count * axes_count);
      EXPECT_EQ(rig.step(), Ret::OK);
      for (size_t h = 0; h < hardware_count; ++h) {
        EXPECT_EQ(rig.commands[h * (5 * axes_count + 1) + 5 * axes_count], 1.);
        for (size_t a = 0; a < axes_count;
          ++a)
          for (size_t f = 1; f < 5;
            ++f)
            EXPECT_EQ(rig.commands[h * (5 * axes_count + 1) + a * 5 + f], 0.);



      }
      rig.ack();
      EXPECT_EQ(rig.step(), Ret::OK);
      for (size_t h = 0; h < hardware_count; ++h) {
        EXPECT_EQ(rig.commands[h * (5 * axes_count + 1) + 5 * axes_count], 2.);
        EXPECT_EQ(rig.commands[h * (5 * axes_count + 1) + 2], 10.);
      }
      rig.controller.on_deactivate(rclcpp_lifecycle::State());
      EXPECT_EQ(rig.activate(), CB::ERROR);
    }
}

TEST_F(Adapter, ExactCommandAndStateSetsRejectMutationsButAllowReordering)
{
  for (bool state: {false, true}) for (int mutation: {0, 1, 2, 4, 5}) {
      Rig rig(1, 3, mutation, state);
      EXPECT_EQ(rig.activate(), mutation == 5 ? CB::SUCCESS : CB::ERROR);
    }
}

TEST_F(Adapter, RejectsInitialPositionAndAcceptedGeneration)
{
  for (double initial_position: {-1.0001, 1.0001, std::numeric_limits<double>::quiet_NaN()}) {
    Rig rig;
    rig.state[0] = initial_position;
    EXPECT_EQ(rig.activate(), CB::ERROR);
  }
  Rig rig;
  rig.state[12] = 1.;
  EXPECT_EQ(rig.activate(), CB::ERROR);
}

TEST_F(Adapter, AllSetterAndReadbackFailuresLatchAndNeverRetry)
{
  for (bool read: {false, true}) for (int key = 0; key < 16; ++key) {
      Rig rig;
      ASSERT_EQ(rig.activate(), CB::SUCCESS);
      if (read) rig.controller.fail_read = key;
      else rig.controller.fail_set = key;
      EXPECT_EQ(rig.step(), Ret::ERROR);
      const auto calls = rig.controller.sets + rig.controller.reads;
      EXPECT_EQ(rig.step(), Ret::ERROR);
      EXPECT_EQ(calls, rig.controller.sets + rig.controller.reads);
    }
}

TEST_F(Adapter, FrozenAcceptedAndRangeDeltaTorqueTemperatureAndNonfiniteStop)
{
  {
    Rig rig;
    ASSERT_EQ(rig.activate(), CB::SUCCESS);
    ASSERT_EQ(rig.step(), Ret::OK);
    EXPECT_EQ(rig.step(), Ret::ERROR);
  }
  for (int fault = 0; fault < 5; ++fault) {
    Rig rig;
    ASSERT_EQ(rig.activate(), CB::SUCCESS);
    ASSERT_EQ(rig.step(), Ret::OK);
    rig.ack();
    if (fault == 0) rig.references[0]->set_value(1.1);
    if (fault == 1) rig.references[0]->set_value(.002);
    if (fault == 2) rig.state[0] = -.6;
    if (fault == 3) rig.state[3] = 75.;
    if (fault == 4) rig.references[0]->set_value(std::numeric_limits<double>::infinity());
    EXPECT_EQ(rig.step(), Ret::ERROR);
  }
}

TEST_F(Adapter, FiniteThenNanIsAnError)
{
  Rig rig;
  ASSERT_EQ(rig.activate(), CB::SUCCESS);
  ASSERT_EQ(rig.step(), Ret::OK);
  rig.ack();
  rig.references[0]->set_value(.0005);
  ASSERT_EQ(rig.step(), Ret::OK);
  rig.ack();
  rig.references[0]->set_value(std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(rig.step(), Ret::ERROR);
}

TEST_F(Adapter, UpdateHasNoAllocationAndStaysWithinApiBound)
{
  Rig rig(4, 3);
  ASSERT_EQ(rig.activate(), CB::SUCCESS);
  rig.controller.sets = rig.controller.reads = rig.controller.states = 0;
  allocations = 0;
  measuring = true;
  const auto result = rig.step();
  measuring = false;
  EXPECT_EQ(result, Ret::OK);
  EXPECT_EQ(allocations, 0U);
  EXPECT_LE(rig.controller.sets + rig.controller.reads + rig.controller.states, 308U);
}

TEST_F(Adapter, MissingVelocityFeedforwardFailsConfigure)
{
  Controller controller;
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "missing_velocity_parameter";
  params.update_rate = 1000;
  ASSERT_EQ(controller.init(params), Ret::OK);
  EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State()), CB::ERROR);
}

TEST_F(Adapter, VelocityReferencesOrderAndBootAndSignedTargets)
{
  for (bool enabled : {false, true}) {
    Rig rig(1, 3, -1, false, enabled);
    ASSERT_EQ(rig.activate(), CB::SUCCESS);
    ASSERT_EQ(rig.references.size(), enabled ? 6U : 3U);
    for (size_t a = 0; a < 3; ++a) {
      EXPECT_EQ(rig.references[a]->get_name(), "adapter_unit/leg0_joint" + std::to_string(a) + "/position");
      if (enabled) {
        EXPECT_EQ(rig.references[3 + a]->get_name(), "adapter_unit/leg0_joint" + std::to_string(a) + "/velocity");
        rig.references[3 + a]->set_value(.5);
      }
    }
    ASSERT_EQ(rig.step(), Ret::OK);
    EXPECT_EQ(rig.commands[1], 0.);
    for (double value : {-.5, .5, 0.}) {
      rig.ack();
      if (enabled) rig.references[3]->set_value(value);
      ASSERT_EQ(rig.step(), Ret::OK);
      EXPECT_EQ(rig.commands[1], enabled ? value : 0.);
    }
  }
}

TEST_F(Adapter, VelocityInitialNanAndFiniteThenNanAndInfinitiesRelease)
{
  for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
    Rig rig(1, 3, -1, false, true);
    ASSERT_EQ(rig.activate(), CB::SUCCESS);
    ASSERT_EQ(rig.step(), Ret::OK);
    rig.ack();
    ASSERT_EQ(rig.step(), Ret::OK);
    EXPECT_EQ(rig.commands[1], 0.);
    rig.ack();
    rig.references[3]->set_value(.5);
    ASSERT_EQ(rig.step(), Ret::OK);
    rig.ack();
    rig.references[3]->set_value(invalid);
    EXPECT_EQ(rig.step(), Ret::ERROR);
    for (size_t a = 0; a < 3; ++a) EXPECT_EQ(rig.commands[5 * a + 1], 0.);
  }
  for (double invalid : {std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
    Rig rig(1, 3, -1, false, true);
    ASSERT_EQ(rig.activate(), CB::SUCCESS);
    rig.references[3]->set_value(invalid);
    EXPECT_EQ(rig.step(), Ret::ERROR);
  }
}

TEST_F(Adapter, VelocitySignedLimitInclusiveAndExcessRelease)
{
  for (double sign : {-1., 1.}) {
    Rig rig(1, 3, -1, false, true);
    ASSERT_EQ(rig.activate(), CB::SUCCESS);
    // A large finite velocity at g=1 must still send zero, without reason=6.
    rig.references[3]->set_value(sign * 1.);
    ASSERT_EQ(rig.step(), Ret::OK);
    EXPECT_EQ(rig.commands[1], 0.);
    rig.ack();
    rig.references[3]->set_value(sign * .7);
    ASSERT_EQ(rig.step(), Ret::OK);
    EXPECT_EQ(rig.commands[1], sign * .7);
    rig.ack();
    rig.references[3]->set_value(sign * .700001);
    EXPECT_EQ(rig.step(), Ret::ERROR);
    EXPECT_EQ(rig.commands[1], 0.);
  }
}

std::vector<std::vector<std::string>> readCsv(const std::filesystem::path & path)
{
  std::ifstream file(path);
  std::vector<std::vector<std::string>> rows;
  std::string line;
  while (std::getline(file, line)) {
    std::vector<std::string> fields;
    std::istringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) fields.push_back(field);
    rows.push_back(fields);
  }
  return rows;
}

TEST_F(Adapter, VelocityCsvValuesAndReasons)
{
  for (bool enabled : {false, true}) for (int fault : {0, 5, 6}) {
    const auto directory = std::filesystem::temp_directory_path() /
      ("bxi_stage8v_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    Rig rig(1, 3, -1, false, enabled, directory.string());
    ASSERT_EQ(rig.activate(), CB::SUCCESS);
    ASSERT_EQ(rig.step(), Ret::OK);
    rig.ack();
    rig.state[1] = .2;
    rig.references[0]->set_value(.0005);
    if (enabled) rig.references[3]->set_value(.5);
    ASSERT_EQ(rig.step(), Ret::OK);
    rig.ack();
    if (fault && enabled) {
      rig.references[3]->set_value(fault == 5 ? std::numeric_limits<double>::quiet_NaN() : .8);
      EXPECT_EQ(rig.step(), Ret::ERROR);
    }
    rig.controller.on_deactivate(rclcpp_lifecycle::State());
    ASSERT_EQ(rig.controller.on_cleanup(rclcpp_lifecycle::State()), CB::SUCCESS);
    const auto rows = readCsv(directory / "joint_cycles.csv");
    ASSERT_GE(rows.size(), 7U);
    ASSERT_EQ(rows[0].size(), 16U);
    EXPECT_EQ(rows[0][13], "reference_velocity_raw");
    EXPECT_EQ(rows[0][14], "target_velocity");
    EXPECT_EQ(rows[0][15], "torque_proxy_with_kd");
    EXPECT_EQ(std::stod(rows[1][14]), 0.);
    if (enabled) EXPECT_EQ(std::stod(rows[4][13]), .5);
    else EXPECT_TRUE(std::isnan(std::stod(rows[4][13])));
    EXPECT_EQ(std::stod(rows[4][14]), enabled ? .5 : 0.);
    EXPECT_NEAR(std::stod(rows[4][15]), std::abs(.005 + (enabled ? .5 : 0.) - .2), 1e-14);
    const auto cycles = readCsv(directory / "cycles.csv");
    if (fault && enabled) EXPECT_EQ(std::stoi(cycles.back()[6]), fault);
    const auto manifest = readCsv(directory / "manifest.csv");
    EXPECT_TRUE(std::any_of(manifest.begin(), manifest.end(), [enabled](const auto & row) {
      return row.size() == 4 && row[2] == "velocity_feedforward" && row[3] == (enabled ? "true" : "false");
    }));
    std::filesystem::remove_all(directory);
  }
}

TEST_F(Adapter, FullRecordingContinuesNormalOutputAndCountsDroppedCycles)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("bxi_capacity_normal_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  Rig rig(1, 3, -1, false, false, directory.string());
  ASSERT_EQ(rig.activate(), CB::SUCCESS);
  for (int cycle = 1; cycle <= 30; ++cycle) {
    if (cycle > 1) rig.references[0]->set_value((cycle - 1) * .0001);
    ASSERT_EQ(rig.step(), Ret::OK);
    EXPECT_NEAR(rig.commands[0], (cycle - 1) * .0001, 1e-14);
    EXPECT_EQ(rig.commands[15], cycle);
    if (cycle > 1) EXPECT_EQ(rig.commands[2], 10.);
    rig.ack();
  }
  rig.controller.on_deactivate(rclcpp_lifecycle::State());
  ASSERT_EQ(rig.controller.on_cleanup(rclcpp_lifecycle::State()), CB::SUCCESS);
  const auto cycles = readCsv(directory / "cycles.csv");
  ASSERT_EQ(cycles.size(), 22U);
  for (size_t i = 1; i < cycles.size(); ++i) {
    EXPECT_EQ(std::stoi(cycles[i][6]), 0);
    EXPECT_EQ(std::stoi(cycles[i][9]), 10);
    EXPECT_EQ(std::stoi(cycles[i][2]), i <= 20 ? static_cast<int>(i) : 30);
  }
  std::filesystem::remove_all(directory);
}

TEST_F(Adapter, FullRecordingStillReleasesOnVelocityLimit)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("bxi_capacity_limit_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  Rig rig(1, 3, -1, false, true, directory.string());
  ASSERT_EQ(rig.activate(), CB::SUCCESS);
  for (int cycle = 1; cycle < 25; ++cycle) {
    ASSERT_EQ(rig.step(), Ret::OK);
    rig.ack();
  }
  rig.references[3]->set_value(.700001);
  EXPECT_EQ(rig.step(), Ret::ERROR);
  for (size_t axis = 0; axis < 3; ++axis) {
    for (size_t field = 1; field < 5; ++field) EXPECT_EQ(rig.commands[5 * axis + field], 0.);
  }
  rig.controller.on_deactivate(rclcpp_lifecycle::State());
  ASSERT_EQ(rig.controller.on_cleanup(rclcpp_lifecycle::State()), CB::SUCCESS);
  const auto cycles = readCsv(directory / "cycles.csv");
  ASSERT_EQ(cycles.size(), 22U);
  EXPECT_EQ(std::stoi(cycles.back()[2]), 25);
  EXPECT_EQ(std::stoi(cycles.back()[6]), 6);
  EXPECT_EQ(std::stoi(cycles.back()[9]), 5);
  std::filesystem::remove_all(directory);
}

}
