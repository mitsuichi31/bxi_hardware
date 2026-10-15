#include <gtest/gtest.h>
#include <fstream>
#include <algorithm>
#include <limits>
#include "bxi_hardware/bxi_system_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
namespace bxi_hardware
{
struct PairSystemTestAccess
{
  static void arm(BxiSystemInterface & s) {
    s.pair_commit_.reset(1000000000, 20000000);
    s.pair_records_.resize(2); s.pair_record_count_ = 0;
  }
  static auto write(BxiSystemInterface & s, int64_t now) {return s.writePair(now);}
  static auto reason(const BxiSystemInterface & s) {return s.pair_records_[0].reason;}
  static size_t count(const BxiSystemInterface & s) {return s.pair_record_count_;}
  static uint64_t last(const BxiSystemInterface & s) {return s.pair_commit_.last();}
};
namespace
{
using H = hardware_interface::CallbackReturn;
using Ret = hardware_interface::return_type;
hardware_interface::HardwareInfo info(bool pair = true, bool gpio = true)
{
  const auto path = ::testing::TempDir() + "pair_system.yaml";
  std::ofstream out(path);
  out << "active_can_buses: [can0]\ncan_frame_format: {can0: fd_brs}\n"
    "defaults: {can_hz: 1000}\ncan_transport:\n"
    "  can0: {rx_mode: separate, tx_frame: broadcast, tx_trigger: write, "
    "stream_feedback_max_age_ms: 3, stream_command_max_age_ms: 5, "
    "stream_write_grace_ms: 0.2, stream_first_write_timeout_ms: 20}\njoints:\n";
  for (int axis = 1; axis <= 2; ++axis) {
    out << "  joint" << axis << ": {enabled: true, can_bus: can0, can_id: " << axis <<
      ", motor_dir: 1, motor_type: MOTOR_70, lower: -1.0, upper: 1.0}\n";
  }
  out.close();
  hardware_interface::HardwareInfo h;
  h.name = "pair_test"; h.type = "system";
  h.hardware_parameters["config_file"] = path;
  h.hardware_parameters["spec_dir"] = std::string(BXI_TEST_SOURCE_DIR) + "/config/specs";
  if (pair) {h.hardware_parameters["command_commit_mode"] = "pair_generation_v1";}
  for (const auto & name : {"joint1", "joint2"}) {
    hardware_interface::ComponentInfo joint; joint.name = name; joint.type = "joint";
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
  unsigned publications = 0;
  std::array<PublishedCommand, 2> last{};
  CommandPublishResult result = CommandPublishResult::kSnapshotPublished;
protected:
  int64_t pairSteadyNow() const noexcept override {return now;}
  CommandPublishResult publishPairBatch(const PublishedCommand * packets) override {
    ++publications; std::copy(packets, packets + 2, last.begin()); return result;
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
}  // namespace
}  // namespace bxi_hardware
