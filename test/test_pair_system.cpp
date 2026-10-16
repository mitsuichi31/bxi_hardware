#include <gtest/gtest.h>
#include <fstream>
#include <functional>
#include <algorithm>
#include <limits>
#include <cstdlib>
#include <filesystem>
#include "bxi_hardware/bxi_system_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
namespace bxi_hardware
{
struct PairCommitTestAccess {static void last(PairCommit & pair, uint64_t last) {pair.last_ = last;}};
struct PairSystemTestAccess
{
  static void arm(BxiSystemInterface & s) {
    s.pair_commit_.reset(1000000000, 20000000);
    s.pair_safety_.reset(1, s.n_joints_);
    s.pair_records_.resize(2); s.pair_record_count_ = 0;
  }
  static void save(BxiSystemInterface & s, const std::string & directory) {
    std::filesystem::create_directories(directory);
    s.worker_timing_dir_ = directory; s.pair_records_saved_ = false;
    s.savePairRecords();
  }
  static void capacity(BxiSystemInterface & s, size_t capacity) {s.pair_records_.resize(capacity);}
  static auto & safety(BxiSystemInterface & s) {return s.pair_safety_;}
  static void last(BxiSystemInterface & s, uint64_t value) {PairCommitTestAccess::last(s.pair_commit_, value);}
  static auto write(BxiSystemInterface & s, int64_t now) {return s.writePair(now);}
  static auto reason(const BxiSystemInterface & s) {return s.pair_records_[0].reason;}
  static size_t count(const BxiSystemInterface & s) {return s.pair_record_count_;}
  static auto latestReason(const BxiSystemInterface & s) {return s.pair_records_[s.pair_record_count_ - 1].reason;}
  static bool latched(const BxiSystemInterface & s) {return s.pair_commit_.latched();}
  static double accepted(const BxiSystemInterface & s) {return s.pair_accepted_;}
  static uint64_t last(const BxiSystemInterface & s) {return s.pair_commit_.last();}
};
namespace
{
using H = hardware_interface::CallbackReturn;
using Ret = hardware_interface::return_type;
hardware_interface::HardwareInfo info(bool pair = true, bool gpio = true, size_t axes = 2)
{
  const auto path = ::testing::TempDir() + "pair_system.yaml";
  std::ofstream out(path);
  out << "active_can_buses: [can0]\ncan_frame_format: {can0: fd_brs}\n"
    "defaults: {can_hz: 1000}\ncan_transport:\n"
    "  can0: {rx_mode: separate, tx_frame: broadcast, tx_trigger: write, "
    "stream_feedback_max_age_ms: 3, stream_command_max_age_ms: 5, "
    "stream_write_grace_ms: 0.2, stream_first_write_timeout_ms: 20}\njoints:\n";
  for (size_t axis = 1; axis <= axes; ++axis) {
    out << "  joint" << axis << ": {enabled: true, can_bus: can0, can_id: " << axis <<
      ", motor_dir: 1, motor_type: MOTOR_70, lower: -1.0, upper: 1.0}\n";
  }
  out.close();
  hardware_interface::HardwareInfo h;
  h.name = "pair_test"; h.type = "system";
  h.hardware_parameters["config_file"] = path;
  h.hardware_parameters["spec_dir"] = std::string(BXI_TEST_SOURCE_DIR) + "/config/specs";
  if (pair) {h.hardware_parameters["command_commit_mode"] = "pair_generation_v1";}
  for (size_t axis = 1; axis <= axes; ++axis) {
    hardware_interface::ComponentInfo joint; joint.name = "joint" + std::to_string(axis); joint.type = "joint";
    for (const auto & field : {"position", "velocity", "kp", "kd", "effort"}) {
      hardware_interface::InterfaceInfo interface;
      interface.name = field; interface.data_type = "double";
      interface.min = (interface.name == "position" || interface.name == "velocity") ? "-0.5" : "0";
      interface.max = interface.name == "effort" ? "0" : "0.5";
      joint.command_interfaces.push_back(interface);
    }
    h.joints.push_back(joint);
  }
  if (gpio) {
    hardware_interface::ComponentInfo group; group.name = "bxi_pair"; group.type = "gpio";
    hardware_interface::InterfaceInfo command; command.name = "commit_generation"; command.data_type = "double";
    hardware_interface::InterfaceInfo state; state.name = "accepted_generation"; state.data_type = "double";
    group.command_interfaces.push_back(command); group.state_interfaces.push_back(state);
    h.gpios.push_back(group);
  }
  return h;
}
std::vector<std::string> keys(BxiSystemInterface & system)
{
  std::vector<std::string> result;
  for (const auto & interface : system.export_command_interfaces()) {result.push_back(interface.get_name());}
  return result;
}
TEST(PairSystem, ExportElevenCommandsNineStatesAndZeroGeneration)
{
  BxiSystemInterface system; ASSERT_EQ(system.on_init(info()), H::SUCCESS);
  const auto commands = system.export_command_interfaces(); const auto states = system.export_state_interfaces();
  ASSERT_EQ(commands.size(), 11U); ASSERT_EQ(states.size(), 9U);
  EXPECT_EQ(commands.back().get_name(), "bxi_pair/commit_generation");
  EXPECT_EQ(commands.back().get_optional().value(), 0);
  EXPECT_EQ(states.back().get_optional().value(), 0);
}
TEST(PairSystem, LegacyDoesNotRequireGroupAndPreservesNaN)
{
  BxiSystemInterface system; ASSERT_EQ(system.on_init(info(false, false)), H::SUCCESS);
  const auto commands = system.export_command_interfaces();
  EXPECT_EQ(commands.size(), 10U); EXPECT_EQ(system.export_state_interfaces().size(), 8U);
  for (const auto & c : commands) {EXPECT_TRUE(std::isnan(c.get_optional().value()));}
}
TEST(PairSystem, ModeAndGpioMustAgree)
{
  for (const auto pair : {false, true}) {
    BxiSystemInterface system; EXPECT_EQ(system.on_init(info(pair, !pair)), H::ERROR);
  }
}
TEST(PairSystem, InvalidTypeAndMissingProfileRejected)
{
  auto h = info(); h.gpios[0].command_interfaces[0].data_type = "uint32";
  BxiSystemInterface wrong; EXPECT_EQ(wrong.on_init(h), H::ERROR);
  h = info(); h.joints[1].command_interfaces[4].max = "";
  BxiSystemInterface missing; EXPECT_EQ(missing.on_init(h), H::ERROR);
  h = info(); h.joints[0].command_interfaces[2].max = "501";
  BxiSystemInterface codec; EXPECT_EQ(codec.on_init(h), H::ERROR);
}
TEST(PairSystem, EveryPartialStartAndStopRejectedWithoutChangingOwnership)
{
  BxiSystemInterface system; ASSERT_EQ(system.on_init(info()), H::SUCCESS); auto full = keys(system);
  for (size_t missing = 0; missing < 11; ++missing) {
    auto partial = full; partial.erase(partial.begin() + missing);
    EXPECT_EQ(system.prepare_command_mode_switch(partial, {}), Ret::ERROR);
    EXPECT_EQ(system.perform_command_mode_switch(partial, {}), Ret::ERROR);
  }
  ASSERT_EQ(system.perform_command_mode_switch(full, {}), Ret::OK);
  for (size_t missing = 0; missing < 11; ++missing) {
    auto partial = full; partial.erase(partial.begin() + missing);
    EXPECT_EQ(system.prepare_command_mode_switch({}, partial), Ret::ERROR);
    EXPECT_EQ(system.perform_command_mode_switch({}, partial), Ret::ERROR);
  }
  EXPECT_EQ(system.perform_command_mode_switch({}, full), Ret::OK);
  EXPECT_EQ(system.prepare_command_mode_switch(full, {}), Ret::ERROR);
}
TEST(PairSystem, ActiveReplacementAndDuplicateKeysRejected)
{
  BxiSystemInterface system; ASSERT_EQ(system.on_init(info()), H::SUCCESS); auto full = keys(system);
  auto duplicate = full; duplicate[0] = duplicate[1];
  EXPECT_EQ(system.prepare_command_mode_switch(duplicate, {}), Ret::ERROR);
  EXPECT_EQ(system.perform_command_mode_switch(full, {}), Ret::OK);
  EXPECT_EQ(system.prepare_command_mode_switch(full, full), Ret::ERROR);
  EXPECT_EQ(system.prepare_command_mode_switch(full, {}), Ret::ERROR);
}
class PairHarness : public BxiSystemInterface
{
public:
  int64_t now = 1000000100;
  std::function<void()> time_hook;
  unsigned publications = 0;
  std::array<PublishedCommand, PairCommit::kMaxAxes> last{};
  size_t axes = 2;
  CommandPublishResult result = CommandPublishResult::kSnapshotPublished;
protected:
  int64_t pairSteadyNow() const noexcept override {if (time_hook) {time_hook();} return now;}
  CommandPublishResult publishPairBatch(const PublishedCommand * packets) override {
    ++publications; std::copy(packets, packets + axes, last.begin()); return result;
  }
};
TEST(PairSystem, SameGenerationDoesNotPublishOrRefreshTimestamp)
{
  PairHarness s; ASSERT_EQ(s.on_init(info()), H::SUCCESS); PairSystemTestAccess::arm(s);
  ASSERT_EQ(s.perform_command_mode_switch(keys(s), {}), Ret::OK);
  auto commands = s.export_command_interfaces();
  for (size_t i = 0; i < 10; ++i) {ASSERT_TRUE(commands[i].set_value(0.));}
  ASSERT_TRUE(commands[10].set_value(1.));
  EXPECT_EQ(PairSystemTestAccess::write(s, s.now), Ret::OK); ASSERT_EQ(s.publications, 1U);
  const auto stamp = s.last[0].published_ns;
  ASSERT_TRUE(commands[2].set_value(.25)); s.now += 1000000;
  PairSystemTestAccess::write(s, s.now);
  EXPECT_EQ(s.publications, 1U); EXPECT_EQ(s.last[0].published_ns, stamp);
  EXPECT_EQ(s.last[1].published_ns, stamp);
  EXPECT_EQ(PairSystemTestAccess::last(s), 1U);
}
TEST(PairSystem, PublishFailureConsumesGenerationAndLatchesWithoutNormalRecovery)
{
  for (auto result : {CommandPublishResult::kNotPublished, CommandPublishResult::kSnapshotPublishedNotifyFailed}) {
    PairHarness s; ASSERT_EQ(s.on_init(info()), H::SUCCESS); PairSystemTestAccess::arm(s);
    ASSERT_EQ(s.perform_command_mode_switch(keys(s), {}), Ret::OK);
    auto commands = s.export_command_interfaces();
    for (size_t i = 0; i < 10; ++i) {ASSERT_TRUE(commands[i].set_value(0.));}
    ASSERT_TRUE(commands[10].set_value(1.)); s.result = result;
    PairSystemTestAccess::write(s, s.now);
    EXPECT_EQ(PairSystemTestAccess::reason(s), PairCommit::Reason::kPublishFailed);
    EXPECT_EQ(PairSystemTestAccess::last(s), 1U);
    ASSERT_TRUE(commands[2].set_value(.25)); ASSERT_TRUE(commands[10].set_value(2.));
    PairSystemTestAccess::write(s, s.now + 1); EXPECT_EQ(s.publications, 1U);
  }
}
TEST(PairSystem, FixedRecordCapacityCountsMissingRowsWithoutAllocation)
{
  PairHarness s; ASSERT_EQ(s.on_init(info()), H::SUCCESS); PairSystemTestAccess::arm(s);
  for (int i = 0; i < 5; ++i) {PairSystemTestAccess::write(s, s.now + i);}
  EXPECT_EQ(PairSystemTestAccess::count(s), 5U); EXPECT_EQ(s.publications, 0U);
}
TEST(PairSystem, RejectedSetterSeamDoesNotProduceNormalCommit)
{
  for (size_t refused = 0; refused <= 10; ++refused) {
    PairHarness s; ASSERT_EQ(s.on_init(info()), H::SUCCESS); PairSystemTestAccess::arm(s);
    ASSERT_EQ(s.perform_command_mode_switch(keys(s), {}), Ret::OK);
    auto commands = s.export_command_interfaces();
    auto setter = [&](size_t i, double value) {return i != refused && commands[i].set_value(value);};
    bool valid = true;
    for (size_t i = 0; i < 10; ++i) {if (!setter(i, 0.)) {valid = false;}}
    if (valid) {setter(10, 1.);}
    PairSystemTestAccess::write(s, s.now); EXPECT_EQ(s.publications, 0U);
    EXPECT_EQ(PairSystemTestAccess::last(s), 0U);
  }
}
// This same test is run against f4bd290 before production changes and after the refactor.
TEST(PairN2Regression, DeterministicPublicationAndCsvFixtures)
{
  const char * directory = std::getenv("BXI_N_AXIS_OUTPUT_DIR");
  const std::string output = directory ? directory : ::testing::TempDir() + "pair_regression";
  std::filesystem::create_directories(output);
  for (int scenario = 0; scenario < 10; ++scenario) {
    PairHarness s; ASSERT_EQ(s.on_init(info()), H::SUCCESS); PairSystemTestAccess::arm(s);
    PairSystemTestAccess::capacity(s, scenario == 9 ? 2 : 8);
    ASSERT_EQ(s.perform_command_mode_switch(keys(s), {}), Ret::OK);
    auto commands = s.export_command_interfaces();
    for (size_t i = 0; i < 10; ++i) {ASSERT_TRUE(commands[i].set_value(0.));}
    ASSERT_TRUE(commands[10].set_value(1.));
    if (scenario == 1) {s.result = CommandPublishResult::kNotPublished;}
    if (scenario == 2) {s.result = CommandPublishResult::kSnapshotPublishedNotifyFailed;}
    if (scenario == 3) {s.now = 1020000000;}
    PairSystemTestAccess::write(s, s.now);
    for (int step = 0; step < 4; ++step) {
      s.now += 1000;
      const double generation = scenario == 4 ? 1 : scenario == 5 ? 3 :
        scenario == 6 ? 0 : scenario == 7 ? 1.5 : 2 + step;
      ASSERT_TRUE(commands[10].set_value(generation));
      if (scenario == 8) {ASSERT_TRUE(commands[9].set_value(std::numeric_limits<double>::quiet_NaN()));}
      PairSystemTestAccess::write(s, s.now);
    }
    PairSystemTestAccess::save(s, output + "/scenario" + std::to_string(scenario));
    std::ofstream trace(output + "/publication" + std::to_string(scenario) + ".txt");
    trace << s.publications << ',' << PairSystemTestAccess::last(s) << '\n';
    for (size_t i = 0; i < 2; ++i) {
      const auto & packet = s.last[i];
      trace << packet.sequence << ',' << packet.published_ns << '\n';
    }
  }
}
TEST(PairN3System, ExportAndPublishAllAxesWithOneGenerationAndClock)
{
  PairHarness s; s.axes = 3; ASSERT_EQ(s.on_init(info(true, true, 3)), H::SUCCESS);
  ASSERT_EQ(s.on_configure(rclcpp_lifecycle::State()), H::SUCCESS);
  EXPECT_EQ(s.export_state_interfaces().size(), 13U);
  auto commands = s.export_command_interfaces(); ASSERT_EQ(commands.size(), 16U);
  PairSystemTestAccess::arm(s); ASSERT_EQ(s.perform_command_mode_switch(keys(s), {}), Ret::OK);
  for (size_t i = 0; i < 15; ++i) {ASSERT_TRUE(commands[i].set_value(i % 5 == 0 ? .01 * i : 0));}
  ASSERT_TRUE(commands[15].set_value(1.)); PairSystemTestAccess::write(s, s.now);
  ASSERT_EQ(s.publications, 1U);
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_DOUBLE_EQ(s.last[i].command.position, .05 * i);
    EXPECT_EQ(s.last[i].sequence, 1U); EXPECT_EQ(s.last[i].published_ns, s.now);
  }
  EXPECT_DOUBLE_EQ(s.export_state_interfaces().back().get_optional().value(), 1);
  s.now += 1000; ASSERT_TRUE(commands[10].set_value(.25));
  PairSystemTestAccess::write(s, s.now); EXPECT_EQ(s.publications, 1U);
  ASSERT_TRUE(commands[15].set_value(2.)); PairSystemTestAccess::write(s, s.now);
  ASSERT_EQ(s.publications, 2U); EXPECT_DOUBLE_EQ(s.last[2].command.position, .25);
  for (const auto & packet : s.last) {EXPECT_EQ(packet.sequence, 2U);}
  const char * directory = std::getenv("BXI_N_AXIS_OUTPUT_DIR");
  const auto output = directory ? std::string(directory) + "/n3" : ::testing::TempDir() + "pair_n3_csv";
  PairSystemTestAccess::save(s, output);
  std::ifstream csv(output + "/pair_commits.csv");
  std::string line; ASSERT_TRUE(std::getline(csv, line)); EXPECT_EQ(line, "#dropped=1");
  ASSERT_TRUE(std::getline(csv, line)); EXPECT_EQ(std::count(line.begin(), line.end(), ','), 23);
  EXPECT_NE(line.find(",value14"), std::string::npos); EXPECT_EQ(line.find("value15"), std::string::npos);
  for (int i = 0; i < 2; ++i) {ASSERT_TRUE(std::getline(csv, line)); EXPECT_EQ(std::count(line.begin(), line.end(), ','), 23);}
}
TEST(PairN3System, EveryFieldAndCommitSetterRefusalPreventsPublication)
{
  for (size_t refused = 0; refused < 16; ++refused) {
    PairHarness s; s.axes = 3; ASSERT_EQ(s.on_init(info(true, true, 3)), H::SUCCESS);
    PairSystemTestAccess::arm(s); ASSERT_EQ(s.perform_command_mode_switch(keys(s), {}), Ret::OK);
    auto commands = s.export_command_interfaces(); bool valid = true;
    for (size_t i = 0; i < 15; ++i) {
      if (i == refused || !commands[i].set_value(0.)) {valid = false;}
    }
    if (valid && refused != 15) {ASSERT_TRUE(commands[15].set_value(1.));}
    PairSystemTestAccess::write(s, s.now); EXPECT_EQ(s.publications, 0U);
    EXPECT_EQ(PairSystemTestAccess::last(s), 0U);
  }
}
TEST(PairN3System, EveryStoredFieldInvalidityRejectsWholeBatch)
{
  for (size_t field = 0; field < 15; ++field) {
    PairHarness s; s.axes = 3; ASSERT_EQ(s.on_init(info(true, true, 3)), H::SUCCESS);
    PairSystemTestAccess::arm(s); ASSERT_EQ(s.perform_command_mode_switch(keys(s), {}), Ret::OK);
    auto commands = s.export_command_interfaces();
    for (size_t i = 0; i < 15; ++i) {ASSERT_TRUE(commands[i].set_value(0.));}
    ASSERT_TRUE(commands[field].set_value(std::numeric_limits<double>::quiet_NaN()));
    ASSERT_TRUE(commands[15].set_value(1.)); PairSystemTestAccess::write(s, s.now);
    EXPECT_EQ(s.publications, 0U); EXPECT_EQ(PairSystemTestAccess::reason(s), PairCommit::Reason::kInvalidBatch);
  }
}
TEST(PairN3System, CompleteSetsRejectEveryMissingExtraDuplicateAndTerminate)
{
  BxiSystemInterface s; ASSERT_EQ(s.on_init(info(true, true, 3)), H::SUCCESS);
  auto full = keys(s); ASSERT_EQ(full.size(), 16U);
  EXPECT_EQ(s.perform_command_mode_switch({}, {}), Ret::OK);
  for (size_t index = 0; index < 16; ++index) {
    auto partial = full; partial.erase(partial.begin() + index);
    auto duplicate = full; duplicate[index] = full[(index + 1) % 16];
    auto extra = full; extra.push_back("other/position");
    for (const auto & invalid : {partial, duplicate, extra}) {
      EXPECT_EQ(s.prepare_command_mode_switch(invalid, {}), Ret::ERROR);
      EXPECT_EQ(s.perform_command_mode_switch(invalid, {}), Ret::ERROR);
    }
  }
  ASSERT_EQ(s.perform_command_mode_switch(full, {}), Ret::OK);
  EXPECT_EQ(s.prepare_command_mode_switch(full, full), Ret::ERROR);
  EXPECT_EQ(s.prepare_command_mode_switch(full, {}), Ret::ERROR);
  for (size_t index = 0; index < 16; ++index) {
    auto partial = full; partial.erase(partial.begin() + index);
    auto duplicate = full; duplicate[index] = full[(index + 1) % 16];
    auto extra = full; extra.push_back("other/position");
    for (const auto & invalid : {partial, duplicate, extra}) {
      EXPECT_EQ(s.prepare_command_mode_switch({}, invalid), Ret::ERROR);
      EXPECT_EQ(s.perform_command_mode_switch({}, invalid), Ret::ERROR);
    }
  }
  EXPECT_EQ(s.perform_command_mode_switch({}, full), Ret::OK);
  EXPECT_EQ(s.perform_command_mode_switch(full, {}), Ret::ERROR);
}
TEST(PairN3System, RejectsWrongCountsJointOrderAndIds)
{
  for (size_t count : {0U, 1U, 4U}) {
    BxiSystemInterface s; EXPECT_EQ(s.on_init(info(true, true, count)), H::ERROR);
  }
  auto h = info(true, true, 3); std::swap(h.joints[1], h.joints[2]);
  BxiSystemInterface wrong_order; EXPECT_EQ(wrong_order.on_init(h), H::ERROR);
  h = info(true, true, 3);
  // Give the third configured joint a different, nonduplicate CAN ID.
  std::ifstream input(h.hardware_parameters.at("config_file"));
  std::string yaml((std::istreambuf_iterator<char>(input)), {}); input.close();
  const auto position = yaml.find("can_id: 3"); ASSERT_NE(position, std::string::npos);
  yaml.replace(position, std::string("can_id: 3").size(), "can_id: 4");
  std::ofstream output(h.hardware_parameters.at("config_file")); output << yaml; output.close();
  BxiSystemInterface wrong_id; EXPECT_EQ(wrong_id.on_init(h), H::ERROR);
  h = info(true, true, 3); h.joints[2].command_interfaces[4].max = "";
  BxiSystemInterface missing; EXPECT_EQ(missing.on_init(h), H::ERROR);
}
TEST(PairN3System, LegacyJointCountsRemainUnrestrictedByCommitMode)
{
  for (size_t count : {1U, 2U, 3U, 4U}) {
    BxiSystemInterface s; ASSERT_EQ(s.on_init(info(false, false, count)), H::SUCCESS);
    EXPECT_EQ(s.export_command_interfaces().size(), 5 * count);
    EXPECT_EQ(s.export_state_interfaces().size(), 4 * count);
  }
}
// Limit observations deliberately use SKIP, never the successful acceptance count.
TEST(PairLimits, AggregatedOwnersCannotBeDistinguished)
{
  BxiSystemInterface s; ASSERT_EQ(s.on_init(info()), H::SUCCESS);
  auto aggregate = keys(s); std::reverse(aggregate.begin(), aggregate.end());
  ASSERT_EQ(s.prepare_command_mode_switch(aggregate, {}), Ret::OK);
  GTEST_SKIP() << "Observed limitation: flattened A/B/C keys accepted; not contract acceptance.";
}
TEST(PairLimits, TransformedValidGenerationCannotBeDistinguished)
{
  PairHarness s; ASSERT_EQ(s.on_init(info()), H::SUCCESS); PairSystemTestAccess::arm(s);
  ASSERT_EQ(s.perform_command_mode_switch(keys(s), {}), Ret::OK);
  auto commands = s.export_command_interfaces();
  for (size_t i = 0; i < 10; ++i) {ASSERT_TRUE(commands[i].set_value(0.));}
  // Model a transform before storage. HWIF sees only the resulting exact integer.
  ASSERT_TRUE(commands[10].set_value(std::floor(1.5)));
  PairSystemTestAccess::write(s, s.now); ASSERT_EQ(s.publications, 1U);
  GTEST_SKIP() << "Observed limitation: transformed 1.5->1 accepted; not contract acceptance.";
}
TEST(PairLimits, OldFiniteValueWithDishonestCommitCannotBeDistinguished)
{
  PairHarness s; ASSERT_EQ(s.on_init(info()), H::SUCCESS); PairSystemTestAccess::arm(s);
  ASSERT_EQ(s.perform_command_mode_switch(keys(s), {}), Ret::OK);
  auto commands = s.export_command_interfaces();
  for (size_t i = 0; i < 10; ++i) {ASSERT_TRUE(commands[i].set_value(0.));}
  ASSERT_TRUE(commands[10].set_value(1.)); PairSystemTestAccess::write(s, s.now);
  // A refused new position write leaves the finite old position, but faulty code advances g.
  ASSERT_TRUE(commands[10].set_value(2.)); PairSystemTestAccess::write(s, s.now + 1);
  ASSERT_EQ(s.publications, 2U);
  GTEST_SKIP() << "Observed limitation: old finite field plus dishonest commit accepted; not acceptance.";
}
TEST(PairN3Limits, AggregatedOwnersCannotBeDistinguished)
{
  BxiSystemInterface s; ASSERT_EQ(s.on_init(info(true, true, 3)), H::SUCCESS);
  auto aggregate = keys(s); std::reverse(aggregate.begin(), aggregate.end());
  ASSERT_EQ(s.prepare_command_mode_switch(aggregate, {}), Ret::OK);
  GTEST_SKIP() << "Observed N=3 limitation: flattened A/B/C/D keys accepted; not acceptance.";
}
TEST(PairN3Limits, TransformedValidGenerationCannotBeDistinguished)
{
  PairHarness s; s.axes = 3; ASSERT_EQ(s.on_init(info(true, true, 3)), H::SUCCESS);
  PairSystemTestAccess::arm(s); ASSERT_EQ(s.perform_command_mode_switch(keys(s), {}), Ret::OK);
  auto commands = s.export_command_interfaces();
  for (size_t i = 0; i < 15; ++i) {ASSERT_TRUE(commands[i].set_value(0.));}
  ASSERT_TRUE(commands[15].set_value(std::floor(1.5)));
  PairSystemTestAccess::write(s, s.now); ASSERT_EQ(s.publications, 1U);
  GTEST_SKIP() << "Observed N=3 limitation: transformed 1.5->1 accepted; not acceptance.";
}
TEST(PairN3Limits, OldFiniteValueWithDishonestCommitCannotBeDistinguished)
{
  PairHarness s; s.axes = 3; ASSERT_EQ(s.on_init(info(true, true, 3)), H::SUCCESS);
  PairSystemTestAccess::arm(s); ASSERT_EQ(s.perform_command_mode_switch(keys(s), {}), Ret::OK);
  auto commands = s.export_command_interfaces();
  for (size_t i = 0; i < 15; ++i) {ASSERT_TRUE(commands[i].set_value(0.));}
  ASSERT_TRUE(commands[15].set_value(1.)); PairSystemTestAccess::write(s, s.now);
  // No third-axis field is newly written, but a dishonest writer advances the generation.
  ASSERT_TRUE(commands[15].set_value(2.)); PairSystemTestAccess::write(s, s.now + 1);
  ASSERT_EQ(s.publications, 2U);
  GTEST_SKIP() << "Observed N=3 limitation: old finite third-axis values accepted; not acceptance.";
}
TEST(PairSystem, ResourceManagerRejectsDuplicateKeyAndReleasesLoan)
{
  // No hardware plugin is loaded: only a command handle is imported.
  hardware_interface::ResourceManager manager(std::make_shared<rclcpp::Clock>(), rclcpp::get_logger("pair_key_test"));
  double value = 0;
  std::vector<hardware_interface::CommandInterface::SharedPtr> interfaces;
  interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>("bxi_pair", "commit_generation", &value));
  manager.import_controller_reference_interfaces("test_export", interfaces);
  manager.make_controller_reference_interfaces_available("test_export");
  {
    auto loan = manager.claim_command_interface("bxi_pair/commit_generation");
    EXPECT_TRUE(manager.command_interface_is_claimed("bxi_pair/commit_generation"));
    EXPECT_THROW(manager.claim_command_interface("bxi_pair/commit_generation"), std::runtime_error);
  }
  EXPECT_FALSE(manager.command_interface_is_claimed("bxi_pair/commit_generation"));
  EXPECT_NO_THROW(manager.claim_command_interface("bxi_pair/commit_generation"));
}
TEST(PairB2, FaultRegistersBeforeErrorAndObservationIsSeparate)
{
  for (size_t n : {2U, 3U}) {
    for (double generation : {3., -1., 1.5, std::numeric_limits<double>::infinity(), 9007199254740992.}) {
      PairHarness s; s.axes = n; ASSERT_EQ(s.on_init(info(true, true, n)), H::SUCCESS);
      PairSystemTestAccess::arm(s); ASSERT_EQ(s.perform_command_mode_switch(keys(s), {}), Ret::OK);
      auto commands = s.export_command_interfaces();
      for (size_t i = 0; i < n * 5; ++i) {ASSERT_TRUE(commands[i].set_value(0.));}
      ASSERT_TRUE(commands.back().set_value(generation));
      EXPECT_EQ(PairSystemTestAccess::write(s, s.now), Ret::ERROR);
      auto & pending = PairSystemTestAccess::safety(s);
      EXPECT_TRUE(pending.requested()); EXPECT_FALSE(pending.ready());
      EXPECT_EQ(pending.registered_ns, s.now); EXPECT_EQ(pending.error_observed_ns, 0);
      ++s.now; s.on_error(rclcpp_lifecycle::State());
      EXPECT_TRUE(pending.ready()); EXPECT_EQ(pending.error_observed_ns, s.now);
    }
  }
}
TEST(PairB2, UpperLimitAloneRetainsOkAndWatchdogWhileOtherFaultRetainsRequest)
{
  for (size_t n : {2U, 3U}) {
    PairHarness s; s.axes = n; ASSERT_EQ(s.on_init(info(true, true, n)), H::SUCCESS);
    PairSystemTestAccess::arm(s); ASSERT_EQ(s.perform_command_mode_switch(keys(s), {}), Ret::OK);
    auto commands = s.export_command_interfaces();
    for (size_t i = 0; i < n * 5; ++i) {ASSERT_TRUE(commands[i].set_value(0.));}
    PairSystemTestAccess::last(s, PairCommit::kMaxGeneration);
    ASSERT_TRUE(commands.back().set_value(9007199254740992.));
    EXPECT_EQ(PairSystemTestAccess::write(s, s.now), Ret::OK);
    EXPECT_FALSE(PairSystemTestAccess::safety(s).requested()); EXPECT_EQ(s.publications, 0U);
    // A subsequent rollback is a separate error even after the upper-limit rejection.
    ASSERT_TRUE(commands.back().set_value(0.));
    EXPECT_EQ(PairSystemTestAccess::write(s, ++s.now), Ret::ERROR);
    auto & pending = PairSystemTestAccess::safety(s); ASSERT_TRUE(pending.requested());
    const auto registered = pending.registered_ns.load();
    ASSERT_TRUE(commands.back().set_value(9007199254740992.));
    EXPECT_EQ(PairSystemTestAccess::write(s, ++s.now), Ret::ERROR);
    EXPECT_TRUE(pending.requested()); EXPECT_EQ(pending.registered_ns, registered);
  }
}
TEST(PairB2, CompleteModeStopRequestsOnceAtObservedBoundary)
{
  for (size_t n : {2U, 3U}) {
    PairHarness s; s.axes = n; ASSERT_EQ(s.on_init(info(true, true, n)), H::SUCCESS);
    PairSystemTestAccess::arm(s); auto full = keys(s);
    ASSERT_EQ(s.perform_command_mode_switch(full, {}), Ret::OK);
    auto partial = full; partial.pop_back();
    EXPECT_EQ(s.perform_command_mode_switch({}, partial), Ret::ERROR);
    EXPECT_FALSE(PairSystemTestAccess::safety(s).requested());
    EXPECT_EQ(s.perform_command_mode_switch({}, full), Ret::OK);
    auto & pending = PairSystemTestAccess::safety(s);
    EXPECT_TRUE(pending.ready()); EXPECT_EQ(pending.error_observed_ns, 0);
    EXPECT_GT(pending.mode_stop_observed_ns, 0);
    EXPECT_EQ(pending.sources, static_cast<uint32_t>(SafetySource::kModeStop));
    EXPECT_EQ(pending.requests, 1U);
    EXPECT_EQ(s.perform_command_mode_switch({}, full), Ret::ERROR);
    EXPECT_EQ(pending.requests, 1U);
  }
}
}  // namespace
TEST(PairSystem, FeedbackPendingFreezesGenerationWithoutEarlyErrorAndRejectsRestart)
{
  for (size_t n : {2U, 3U}) {
    for (bool second_check : {false, true}) {
      PairHarness s; s.axes = n; ASSERT_EQ(s.on_init(info(true, true, n)), H::SUCCESS);
      PairSystemTestAccess::arm(s);
      ASSERT_EQ(s.perform_command_mode_switch(keys(s), {}), Ret::OK);
      auto commands = s.export_command_interfaces();
      for (size_t i = 0; i < n * 5; ++i) {ASSERT_TRUE(commands[i].set_value(0.));}
      ASSERT_TRUE(commands.back().set_value(1.));
      auto & safety = PairSystemTestAccess::safety(s);
      if (second_check) {
        s.time_hook = [&] {safety.feedback_stopped = true; s.now += 20000000;};
      }
      else {safety.feedback_stopped = true;}
      EXPECT_EQ(PairSystemTestAccess::write(s, s.now), Ret::OK);
      EXPECT_EQ(PairSystemTestAccess::last(s), 0U); EXPECT_EQ(s.publications, 0U);
      EXPECT_EQ(PairSystemTestAccess::reason(s), PairCommit::Reason::kFeedbackPending);
      EXPECT_EQ(safety.sources.load(), 0U); // No synthetic HWIF ERROR requirement.
      EXPECT_EQ(s.perform_command_mode_switch({}, keys(s)), Ret::OK);
      EXPECT_EQ(s.prepare_command_mode_switch(keys(s), {}), Ret::ERROR);
    }
  }
}
TEST(PairSystem, FeedbackPendingDoesNotSuppressIndependentHwifFault)
{
  for (size_t n : {2U, 3U}) {
    PairHarness s; s.axes = n; ASSERT_EQ(s.on_init(info(true, true, n)), H::SUCCESS);
    PairSystemTestAccess::arm(s); ASSERT_EQ(s.perform_command_mode_switch(keys(s), {}), Ret::OK);
    auto commands = s.export_command_interfaces();
    for (size_t i = 0; i < n * 5; ++i) {ASSERT_TRUE(commands[i].set_value(0.));}
    auto & safety = PairSystemTestAccess::safety(s);
    // A genuinely independent gap must be detected BEFORE stale freezes L.
    ASSERT_TRUE(commands.back().set_value(2.));
    EXPECT_EQ(PairSystemTestAccess::write(s, s.now), Ret::ERROR);
    EXPECT_EQ(safety.sources.load(), 1U); EXPECT_TRUE(PairSystemTestAccess::latched(s));
    safety.feedback_stopped = true;
    safety.request(1, SafetySource::kFeedbackStale, 1, 2);
    ASSERT_TRUE(commands.back().set_value(3.));
    EXPECT_EQ(PairSystemTestAccess::write(s, s.now), Ret::ERROR);
    EXPECT_EQ(safety.sources.load(), 9U); EXPECT_FALSE(safety.ready());
    EXPECT_EQ(safety.requests.load(), 2U); // No new HWIF request during feedback pending.
    EXPECT_EQ(PairSystemTestAccess::latestReason(s), PairCommit::Reason::kFeedbackPending);
    EXPECT_EQ(PairSystemTestAccess::last(s), 0U);
  }
}
TEST(PairSystem, FeedbackPendingAdvancingControllerGenerationsNeverCreateGapError)
{
  for (size_t n : {2U, 3U}) {
    for (bool already_committed : {false, true}) {
      PairHarness s; s.axes = n; ASSERT_EQ(s.on_init(info(true, true, n)), H::SUCCESS);
      PairSystemTestAccess::arm(s); PairSystemTestAccess::capacity(s, 64);
      ASSERT_EQ(s.perform_command_mode_switch(keys(s), {}), Ret::OK);
      auto commands = s.export_command_interfaces();
      for (size_t i = 0; i < n * 5; ++i) {ASSERT_TRUE(commands[i].set_value(0.));}
      if (already_committed) {
        ASSERT_TRUE(commands.back().set_value(1.));
        ASSERT_EQ(PairSystemTestAccess::write(s, s.now), Ret::OK);
      }
      const auto last = PairSystemTestAccess::last(s);
      const auto accepted = PairSystemTestAccess::accepted(s);
      const auto published = s.publications;
      auto & safety = PairSystemTestAccess::safety(s);
      safety.feedback_stopped = true; safety.request(1, SafetySource::kFeedbackStale, 1, 2);
      for (uint64_t step = 1; step <= 16; ++step) {
        ASSERT_TRUE(commands.back().set_value(static_cast<double>(last + step)));
        // Also cross the initial commit deadline: inspect must not run even at L=0.
        EXPECT_EQ(PairSystemTestAccess::write(s, s.now + step * 2000000), Ret::OK);
        EXPECT_EQ(PairSystemTestAccess::latestReason(s), PairCommit::Reason::kFeedbackPending);
        EXPECT_EQ(safety.sources.load(), 8U); EXPECT_EQ(safety.requests.load(), 1U);
        EXPECT_EQ(PairSystemTestAccess::last(s), last);
        EXPECT_EQ(PairSystemTestAccess::accepted(s), accepted);
        EXPECT_EQ(s.publications, published); EXPECT_FALSE(PairSystemTestAccess::latched(s));
      }
      ASSERT_EQ(s.perform_command_mode_switch({}, keys(s)), Ret::OK);
      EXPECT_EQ(s.prepare_command_mode_switch(keys(s), {}), Ret::ERROR);
    }
  }
}
}  // namespace bxi_hardware

// Generated-file acceptance: parser -> real on_init, without activation or CAN I/O.
#include <spawn.h>
#include <unistd.h>
#include <sys/wait.h>
#include <fcntl.h>
#include "hardware_interface/component_parser.hpp"
extern char ** environ;
namespace bxi_hardware
{
namespace
{
class GeneratedBoundsHwif : public BxiSystemInterface
{
public:
  unsigned bus_requests = 0;
protected:
  std::shared_ptr<CanBus> makeBus(CanFrameFormat) override
  {
    ++bus_requests;
    throw std::runtime_error("on_init unexpectedly requested CAN bus construction");
  }
};
struct GeneratedBoundsInput
{
  std::filesystem::path directory;
  bool preserve = false;
  hardware_interface::HardwareInfo hardware;
  static void command(std::vector<std::string> arguments, const std::string & output = "")
  {
    std::vector<char *> argv;
    for (auto & argument : arguments) {argv.push_back(argument.data());}
    argv.push_back(nullptr);
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    if (!output.empty()) {
      posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO, output.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
    }
    pid_t pid;
    const auto spawned = posix_spawnp(&pid, argv[0], &actions, nullptr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    if (spawned != 0) {throw std::runtime_error("cannot spawn " + arguments[0]);}
    int status = 0;
    if (waitpid(pid, &status, 0) != pid || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
      throw std::runtime_error("generated-file command failed: " + arguments[0]);
    }
  }
  explicit GeneratedBoundsInput(size_t n)
  {
    const char * out = std::getenv("BXI_N_AXIS_BOUNDS_OUTPUT_DIR");
    const auto base = out ? std::filesystem::path(out) : std::filesystem::temp_directory_path();
    std::filesystem::create_directories(base);
    auto pattern = (base / ("bounds_n" + std::to_string(n) + "_XXXXXX")).string();
    if (!::mkdtemp(pattern.data())) {throw std::runtime_error("mkdtemp failed");}
    directory = pattern; preserve = out != nullptr;
    const std::string root = BXI_TEST_SOURCE_DIR;
    const auto config = (directory / "config").string();
    std::vector<std::string> generate{root + "/hil/n_axis/make_n_axis_config.py", "--axes", std::to_string(n), "--ids"};
    for (size_t i = 1; i <= n; ++i) {generate.push_back(std::to_string(i));}
    generate.insert(generate.end(), {"--can-bus", "can0", "--destination", config, "--output-dir", directory.string()});
    command(generate);
    const auto urdf = (directory / "robot.urdf").string();
    command({"xacro", root + "/hil/n_axis/n_axis.urdf.xacro", "axes:=" + std::to_string(n),
      "config_file:=" + config + "/hardware.yaml", "spec_dir:=" + root + "/config/specs"}, urdf);
    command({root + "/hil/n_axis/n_axis_check_config.py", config, urdf}, (directory / "check.json").string());
    std::ifstream in(urdf);
    const std::string xml((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const auto parsed = hardware_interface::parse_control_resources_from_urdf(xml);
    if (parsed.size() != 1) {throw std::runtime_error("expected one hardware system");}
    hardware = parsed.front();
  }
  ~GeneratedBoundsInput()
  {
    if (!preserve) {std::error_code error; std::filesystem::remove_all(directory, error);}
  }
};
class GeneratedBoundsAcceptance : public ::testing::TestWithParam<size_t> {};
TEST_P(GeneratedBoundsAcceptance, GeneratedUrdfIsAcceptedWithoutBusConstruction)
{
  const auto n = GetParam();
  GeneratedBoundsInput input(n);
  ASSERT_EQ(input.hardware.joints.size(), n);
  const double pi = 3.14159265358979323846;
  const std::array<std::array<double, 2>, 5> expected{{{{-pi/6., pi/6.}}, {{-.70, .70}}, {{0., 20.}}, {{0., 1.}}, {{0., 0.}}}};
  const std::array<std::string, 5> fields{{"position", "velocity", "kp", "kd", "effort"}};
  for (const auto & joint : input.hardware.joints) {
    ASSERT_EQ(joint.command_interfaces.size(), fields.size());
    for (size_t i = 0; i < fields.size(); ++i) {
      const auto found = std::find_if(joint.command_interfaces.begin(), joint.command_interfaces.end(),
        [&](const auto & value) {return value.name == fields[i];});
      ASSERT_NE(found, joint.command_interfaces.end());
      EXPECT_DOUBLE_EQ(std::stod(found->min), expected[i][0]);
      EXPECT_DOUBLE_EQ(std::stod(found->max), expected[i][1]);
    }
  }
  GeneratedBoundsHwif hwif;
  EXPECT_EQ(hwif.on_init(input.hardware), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(hwif.bus_requests, 0U);
}
TEST_P(GeneratedBoundsAcceptance, ExistingHwifRejectsInvalidBounds)
{
  GeneratedBoundsInput input(GetParam());
  // Mutate parsed InterfaceInfo to isolate on_init's rejection, rather than parser errors.
  // Every copy starts from the same accepted generated-file baseline.
  GeneratedBoundsHwif baseline;
  ASSERT_EQ(baseline.on_init(input.hardware), hardware_interface::CallbackReturn::SUCCESS);
  for (size_t axis = 0; axis < GetParam(); ++axis) {
    for (size_t field = 0; field < 5; ++field) {
      for (const auto mutation : {"missing_min", "missing_max", "nonnumeric_min", "nonnumeric_max", "inverted"}) {
        SCOPED_TRACE(std::to_string(axis) + "/" + std::to_string(field) + "/" + mutation);
        auto info = input.hardware;
        auto & value = info.joints[axis].command_interfaces[field];
        if (std::string(mutation) == "missing_min") {value.min.clear();}
        else if (std::string(mutation) == "missing_max") {value.max.clear();}
        else if (std::string(mutation) == "nonnumeric_min") {value.min = "not-a-number";}
        else if (std::string(mutation) == "nonnumeric_max") {value.max = "not-a-number";}
        else {value.min = "1"; value.max = "0";}
        GeneratedBoundsHwif hwif;
        EXPECT_EQ(hwif.on_init(info), hardware_interface::CallbackReturn::ERROR);
        EXPECT_EQ(hwif.bus_requests, 0U);
      }
    }
    for (const auto mutation : {"position_low", "position_high", "codec_low", "codec_high", "zero_below", "zero_above"}) {
      SCOPED_TRACE(std::to_string(axis) + "/" + mutation);
      auto info = input.hardware;
      auto & values = info.joints[axis].command_interfaces;
      if (std::string(mutation) == "position_low") {values[0].min = "-0.6";}
      else if (std::string(mutation) == "position_high") {values[0].max = "0.6";}
      else if (std::string(mutation) == "codec_low") {values[1].min = "-46";}
      else if (std::string(mutation) == "codec_high") {values[1].max = "46";}
      else if (std::string(mutation) == "zero_below") {values[1].min = "0.1";}
      else {values[1].max = "-0.1";}
      GeneratedBoundsHwif hwif;
      EXPECT_EQ(hwif.on_init(info), hardware_interface::CallbackReturn::ERROR);
      EXPECT_EQ(hwif.bus_requests, 0U);
    }
  }
}
INSTANTIATE_TEST_SUITE_P(TwoAndThree, GeneratedBoundsAcceptance, ::testing::Values(2U, 3U));
}  // namespace
}  // namespace bxi_hardware
