#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <unistd.h>
#include "bxi_hardware/stream_trace_io.hpp"

namespace bxi_hardware
{
namespace
{
using Clock = std::chrono::steady_clock;
std::string readFile(const std::string & path)
{
  std::ifstream file(path);
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}
std::vector<std::string> fields(const std::string & line)
{
  std::vector<std::string> out;
  size_t start = 0;
  for (;;) {
    const auto end = line.find(',', start);
    if (end == std::string::npos) {out.push_back(line.substr(start)); return out;}
    out.push_back(line.substr(start, end - start)); start = end + 1;
  }
}
struct TempDir
{
  std::string path;
  TempDir()
  {
    std::string name = ::testing::TempDir() + "bxi_trace_XXXXXX";
    auto p = ::mkdtemp(name.data());
    if (!p) {throw std::runtime_error("mkdtemp");}
    path = p;
  }
  ~TempDir() {std::filesystem::remove_all(path);}
};
struct FaultOps : TraceFileOps
{
  std::string stage, target;
  bool short_writes = false, interrupt = false;
  std::map<int, std::string> paths;
  bool matches(const std::string & path) const {return path.find(target) != std::string::npos;}
  int open(const std::string & path) override
  {
    if (stage == "open" && matches(path)) {errno = EACCES; return -1;}
    const auto fd = TraceFileOps::open(path);
    if (fd >= 0) {paths[fd] = path;}
    return fd;
  }
  ssize_t write(int fd, const void * data, size_t size) override
  {
    if (interrupt) {interrupt = false; errno = EINTR; return -1;}
    if (stage == "write" && matches(paths.at(fd))) {errno = ENOSPC; return -1;}
    if (stage == "zero" && matches(paths.at(fd))) {return 0;}
    return TraceFileOps::write(fd, data, short_writes ? std::min(size, size_t{7}) : size);
  }
  int close(int fd) override
  {
    const auto path = paths.at(fd);
    paths.erase(fd);
    const int result = TraceFileOps::close(fd);
    if (stage == "close" && matches(path)) {errno = EIO; return -1;}
    return result;
  }
  int link(const std::string & from, const std::string & to) override
  {
    if (stage == "link" && matches(to)) {errno = EINVAL; return -1;}
    if (stage == "race" && matches(to)) {
      std::ofstream(to) << "existing competing complete";
    }
    return TraceFileOps::link(from, to);
  }
  int unlink(const std::string & path) override
  {
    if (stage == "unlink" && matches(path)) {errno = EACCES; return -1;}
    return TraceFileOps::unlink(path);
  }
};

std::unique_ptr<StreamTrace> example()
{
  auto trace = prepareStreamTrace(3, 1);
  trace->origin = Clock::time_point(std::chrono::nanoseconds(123000000));
  trace->send_loop_start_steady_ns = 123100000;
  trace->stream_stop_steady_ns = 130000000;
  trace->axis_metadata[0] = {1, -1, -6.283185307179586, -0.3, 0.3, {}};
  trace->cycle_count = 3; trace->rx_count = 1;
  auto & sent = trace->cycles[0];
  sent.command = {-0.0, std::numeric_limits<double>::denorm_min(),
    std::numeric_limits<double>::max(), 1.0, -1e-200};
  sent.command_valid = sent.encoded_valid = sent.send_attempted = sent.send_success = true;
  sent.command_block = {0, 1, 2, 3, 4, 5, 254, 255};
  sent.send_before_steady_ns = 124000000; sent.send_after_steady_ns = 124010000;
  trace->cycles[1].cycle = 1;
  trace->cycles[1].outcome = StreamCycleOutcome::kSkippedLate;
  trace->cycles[2] = sent; trace->cycles[2].cycle = 2;
  trace->cycles[2].outcome = StreamCycleOutcome::kStopped;
  trace->cycles[2].command_stale = true;
  trace->cycles[2].send_success = false;
  trace->rx[0].received_ns = 5000;
  trace->rx[0].motor_position = 0.23; trace->rx[0].motor_velocity = -0.1;
  trace->rx[0].motor_torque = 0.52;
  return trace;
}
StreamConfig config()
{
  StreamConfig config;
  config.broadcast = true; config.trace_cycles = 3;
  return config;
}

TEST(StreamTracePreparation, ValidatesSettingsAndCheckedSizes)
{
  EXPECT_EQ(parseStreamTraceCycles(nullptr), 20000U);
  EXPECT_EQ(parseStreamTraceCycles("20001"), 20001U);
  EXPECT_EQ(parseStreamTraceCycles("620000"), 620000U);
  for (const char * value : {"", "0", "20000", "-1", "+620000", " 20001", "20001 ",
      "20001.0", "620001", "18446744073709551616000"}) {
    EXPECT_THROW(parseStreamTraceCycles(value), std::invalid_argument) << value;
  }
  EXPECT_THROW(streamTracePayloadBytes(std::numeric_limits<size_t>::max(), 1), std::length_error);
  EXPECT_THROW(streamTracePayloadBytes(1, std::numeric_limits<size_t>::max()), std::length_error);
  EXPECT_THROW(prepareStreamTrace(620000, 3), std::length_error);
  auto trace = prepareStreamTrace(620000, 1);
  EXPECT_EQ(trace->cycles.size(), 620000U);
  EXPECT_EQ(trace->rx.size(), 1240000U);
  EXPECT_EQ(trace->payload_bytes, streamTracePayloadBytes(620000, 1));
  EXPECT_EQ(trace->origin, Clock::time_point{});
  EXPECT_EQ(trace->send_loop_start_steady_ns, -1);
}

TEST(StreamTraceOutput, PreservesHyphenForRunnerFileDiscovery)
{
  TempDir dir;
  StreamTraceOutput output(dir.path, "vcan-a");
  const auto filename = std::filesystem::path(output.stem()).filename().string();
  EXPECT_EQ(filename.find("bxi_stream_vcan-a_"), 0U);
}

TEST(StreamTraceOutput, RoundTripValuesValidityFaultAndAbsoluteClock)
{
  TempDir dir;
  StreamTraceOutput output(dir.path, "can0");
  auto trace = example();
  StreamStats stats; stats.fault = StreamFault::kSendFailed; stats.fault_axis = 0;
  const auto result = output.write(*trace, config(), stats, 3, {}, Clock::now());
  EXPECT_FALSE(result.tmp_cleanup_failed);
  const auto complete = YAML::LoadFile(output.stem() + ".complete");
  EXPECT_EQ(complete["cycle_rows"].as<size_t>(), 3U);
  EXPECT_EQ(complete["rx_rows"].as<size_t>(), 1U);
  EXPECT_EQ(complete["fault"].as<std::string>(), streamFaultName(StreamFault::kSendFailed));
  EXPECT_EQ(complete["stop_reason"].as<std::string>(), "fault");
  std::istringstream csv(readFile(output.stem() + "_cycles.csv"));
  std::string line;
  std::getline(csv, line);
  EXPECT_NE(line.find("offset_rad_effective=-6.283185307179586"), std::string::npos);
  std::getline(csv, line);
  auto header = fields(line); EXPECT_EQ(header.size(), 23U);
  std::getline(csv, line); auto row = fields(line); ASSERT_EQ(row.size(), 23U);
  const auto & c = trace->cycles[0].command;
  const std::array<double, 5> expected{c.position, c.velocity, c.kp, c.kd, c.effort};
  for (size_t i = 0; i < 5; ++i) {
    double v;
    auto parsed = std::from_chars(row[i + 12].data(), row[i + 12].data() + row[i + 12].size(), v);
    ASSERT_EQ(parsed.ec, std::errc{});
    EXPECT_EQ(std::memcmp(&v, &expected[i], sizeof(double)), 0);
  }
  EXPECT_EQ(row[18], "000102030405feff");
  std::getline(csv, line); row = fields(line); ASSERT_EQ(row.size(), 23U);
  EXPECT_EQ(row[10], "0"); EXPECT_EQ(row[11], "none");
  for (size_t i : {12, 13, 14, 15, 16, 18, 20, 21, 22}) {EXPECT_TRUE(row[i].empty()) << i;}
  std::getline(csv, line); row = fields(line);
  EXPECT_EQ(row[11], "command_stale"); EXPECT_EQ(row[19], "1"); EXPECT_EQ(row[20], "0");
  std::istringstream rx(readFile(output.stem() + "_rx.csv"));
  std::getline(rx, line); std::getline(rx, line); std::getline(rx, line);
  row = fields(line); ASSERT_EQ(row.size(), 9U);
  EXPECT_EQ(row[5], "123005000"); EXPECT_DOUBLE_EQ(std::stod(row[6]), 0.23);
  EXPECT_THROW(output.write(*trace, config(), stats, 3, {}, Clock::now()), std::runtime_error);
}

TEST(StreamTraceOutput, CompleteReportsTruncationAndExternalStop)
{
  TempDir dir;
  StreamTraceOutput output(dir.path, "can0");
  auto trace = example(); trace->cycle_count = 7; trace->rx_count = 10;
  StreamStats stats; stats.stopped_externally = true;
  const auto result = output.write(*trace, config(), stats, 3, {}, Clock::now());
  EXPECT_EQ(result.dropped_cycles, 4U); EXPECT_EQ(result.dropped_rx, 4U);
  auto complete = YAML::LoadFile(output.stem() + ".complete");
  EXPECT_EQ(complete["stop_reason"].as<std::string>(), "external");
  EXPECT_EQ(complete["fault"].as<std::string>(), streamFaultName(StreamFault::kNone));
  EXPECT_EQ(complete["dropped_cycles"].as<size_t>(), 4U);
}

TEST(StreamTraceOutput, ShortWritesAndInterruptAreRetried)
{
  TempDir dir;
  auto ops = std::make_shared<FaultOps>();
  StreamTraceOutput output(dir.path, "can0", ops);
  ops->short_writes = true; ops->interrupt = true;
  auto trace = example();
  EXPECT_NO_THROW(output.write(*trace, config(), {}, 3, {}, Clock::now()));
  EXPECT_TRUE(std::filesystem::exists(output.stem() + ".complete"));
}

TEST(StreamTraceOutput, FileFailuresNeverPublishComplete)
{
  for (const std::string stage : {"open", "write", "close", "zero"}) {
    for (const std::string target : {"_cycles.csv", "_rx.csv", ".complete.tmp"}) {
      SCOPED_TRACE(stage + target);
      TempDir dir;
      auto ops = std::make_shared<FaultOps>();
      StreamTraceOutput output(dir.path, "can0", ops);
      ops->stage = stage; ops->target = target;
      auto trace = example();
      EXPECT_THROW(output.write(*trace, config(), {}, 3, {}, Clock::now()), std::runtime_error);
      EXPECT_FALSE(std::filesystem::exists(output.stem() + ".complete"));
    }
  }
}

TEST(StreamTraceOutput, UnsupportedLinksAreRejectedBeforeEnable)
{
  TempDir dir;
  auto ops = std::make_shared<FaultOps>(); ops->stage = "link"; ops->target = ".probe.link";
  EXPECT_THROW(StreamTraceOutput(dir.path, "can0", ops), std::runtime_error);
  EXPECT_TRUE(std::filesystem::is_empty(dir.path));
}

TEST(StreamTraceOutput, PublishFailureAndRaceDoNotOverwrite)
{
  for (const std::string stage : {"link", "race"}) {
    TempDir dir;
    auto ops = std::make_shared<FaultOps>();
    StreamTraceOutput output(dir.path, "can0", ops);
    ops->stage = stage; ops->target = ".complete";
    auto trace = example();
    EXPECT_THROW(output.write(*trace, config(), {}, 3, {}, Clock::now()), std::runtime_error);
    if (stage == "race") {EXPECT_EQ(readFile(output.stem() + ".complete"), "existing competing complete");}
    else {EXPECT_FALSE(std::filesystem::exists(output.stem() + ".complete"));}
  }
}

TEST(StreamTraceOutput, PublishedCompleteSurvivesTmpCleanupFailure)
{
  TempDir dir;
  auto ops = std::make_shared<FaultOps>();
  StreamTraceOutput output(dir.path, "can0", ops);
  ops->stage = "unlink"; ops->target = ".complete.tmp";
  auto trace = example();
  EXPECT_TRUE(output.write(*trace, config(), {}, 3, {}, Clock::now()).tmp_cleanup_failed);
  EXPECT_EQ(YAML::LoadFile(output.stem() + ".complete")["cycle_rows"].as<size_t>(), 3U);
}

// Tests the very same serializer used by HWIF, with fully populated maximum-capacity vectors.
TEST(StreamTraceOutput, MaximumCapacityWritesWithinTwoSeconds)
{
  TempDir dir;
  auto trace = prepareStreamTrace(620000, 1);
  trace->origin = Clock::now(); trace->axis_metadata[0].can_id = 1;
  trace->cycle_count = trace->cycles.size(); trace->rx_count = trace->rx.size();
  for (size_t i = 0; i < trace->cycles.size(); ++i) {
    auto & r = trace->cycles[i]; r.cycle = i;
    r.command = {std::sin(i * 0.001) * 0.1, std::cos(i * 0.001) * 0.3,
      20.0 + std::sin(i * 0.0001) * 0.1, 1.0, std::sin(i * 0.002) * 0.01};
    r.command_valid = r.encoded_valid = r.send_attempted = r.send_success = true;
    r.send_before_steady_ns = 40000000000000LL + i * 1000000;
    r.send_after_steady_ns = r.send_before_steady_ns + 5000;
  }
  for (size_t i = 0; i < trace->rx.size(); ++i) {
    auto & r = trace->rx[i]; r.received_ns = i * 500000;
    r.rx_kernel_realtime_ns = 1790800000000000000LL + r.received_ns;
    r.rx_userspace_realtime_ns = r.rx_kernel_realtime_ns + 10000;
    r.motor_position = std::sin(i * 0.0005) * 0.1;
    r.motor_velocity = std::cos(i * 0.0005) * 0.3;
    r.motor_torque = std::sin(i * 0.001) * 0.1;
  }
  StreamTraceOutput output(dir.path, "can0");
  auto c = config(); c.trace_cycles = 620000; c.trace_extended = true;
  const auto result = output.write(*trace, c, {}, 3, {}, Clock::now());
  RecordProperty("report_to_complete_ns", std::to_string(result.report_to_complete_ns));
  RecordProperty("payload_bytes", std::to_string(trace->payload_bytes));
  RecordProperty("sizeof_cycle", std::to_string(sizeof(StreamCycleRecord)));
  RecordProperty("sizeof_rx", std::to_string(sizeof(StreamRxRecord)));
  EXPECT_LE(result.report_to_complete_ns, 2000000000LL);
  EXPECT_EQ(result.cycle_rows, 620000U); EXPECT_EQ(result.rx_rows, 1240000U);
}
}  // namespace
}  // namespace bxi_hardware
