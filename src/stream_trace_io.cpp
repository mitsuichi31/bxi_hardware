#include "bxi_hardware/stream_trace_io.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <filesystem>
#include <limits>
#include <fcntl.h>
#include <unistd.h>

namespace bxi_hardware
{
namespace
{
using Clock = std::chrono::steady_clock;
int64_t ns(Clock::duration d)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(d).count();
}
[[noreturn]] void fail(const std::string & what)
{
  throw std::runtime_error(what + ": " + std::strerror(errno));
}
size_t mul(size_t a, size_t b)
{
  if (b != 0 && a > std::numeric_limits<size_t>::max() / b) {
    throw std::length_error("trace size overflow");
  }
  return a * b;
}
size_t add(size_t a, size_t b)
{
  if (a > std::numeric_limits<size_t>::max() - b) {
    throw std::length_error("trace size overflow");
  }
  return a + b;
}
std::string hexBlock(const std::array<uint8_t, 8> & block)
{
  constexpr char hex[] = "0123456789abcdef";
  std::string result(16, '0');
  for (size_t j = 0; j < 8; ++j) {
    result[j * 2] = hex[block[j] >> 4]; result[j * 2 + 1] = hex[block[j] & 15];
  }
  return result;
}
std::string decimal(double value)
{
  char chars[128];
  auto r = std::to_chars(chars, chars + sizeof(chars), value, std::chars_format::general);
  if (r.ec != std::errc{}) {throw std::runtime_error("metadata to_chars");}
  return {chars, r.ptr};
}
std::string describe(std::string value)
{
  std::replace(value.begin(), value.end(), ',', '|');
  std::replace(value.begin(), value.end(), ' ', '_');
  return value;
}
}

size_t parseStreamTraceCycles(const char * setting)
{
  if (setting == nullptr) {return 20000;}
  std::string_view value(setting);
  if (value.empty() || !std::all_of(value.begin(), value.end(),
      [](char ch) {return ch >= '0' && ch <= '9';})) {
    throw std::invalid_argument("BXI_STREAM_TRACE_CYCLES must contain ASCII digits only");
  }
  size_t count = 0;
  const auto r = std::from_chars(value.data(), value.data() + value.size(), count);
  if (r.ec != std::errc{} || r.ptr != value.data() + value.size() || count <= 20000 || count > 620000) {
    throw std::invalid_argument("BXI_STREAM_TRACE_CYCLES must be 20001..620000");
  }
  return count;
}

size_t streamTracePayloadBytes(size_t cycles, size_t axes)
{
  size_t total = mul(cycles, sizeof(StreamCycleRecord));
  total = add(total, mul(mul(cycles, axes), sizeof(int64_t)));
  total = add(total, mul(mul(cycles, axes), sizeof(uint8_t)));
  total = add(total, mul(mul(mul(2, cycles), axes), sizeof(StreamRxRecord)));
  if (total > kStreamTraceMemoryLimit) {throw std::length_error("trace exceeds 256 MiB");}
  return total;
}

std::unique_ptr<StreamTrace> prepareStreamTrace(size_t cycles, size_t axes)
{
  const auto before = Clock::now();
  const auto bytes = streamTracePayloadBytes(cycles, axes);
  auto trace = std::make_unique<StreamTrace>();
  trace->axes = axes;
  trace->cycles.resize(cycles);
  trace->age_ns.resize(mul(cycles, axes));
  trace->fresh.resize(mul(cycles, axes));
  trace->rx.resize(mul(mul(2, cycles), axes));
  trace->axis_metadata.resize(axes);
  trace->payload_bytes = add(mul(trace->cycles.capacity(), sizeof(StreamCycleRecord)),
    add(mul(trace->age_ns.capacity(), sizeof(int64_t)),
    add(mul(trace->fresh.capacity(), sizeof(uint8_t)), mul(trace->rx.capacity(), sizeof(StreamRxRecord)))));
  if (trace->payload_bytes > kStreamTraceMemoryLimit || trace->payload_bytes < bytes) {
    throw std::length_error("trace actual allocation exceeds limit");
  }
  trace->allocation_ns = ns(Clock::now() - before);
  return trace;
}

int TraceFileOps::open(const std::string & path)
{
  return ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
}
ssize_t TraceFileOps::write(int fd, const void * data, size_t size) {return ::write(fd, data, size);}
int TraceFileOps::close(int fd) {return ::close(fd);}
int TraceFileOps::link(const std::string & from, const std::string & to) {return ::link(from.c_str(), to.c_str());}
int TraceFileOps::unlink(const std::string & path) {return ::unlink(path.c_str());}

TraceBufferedWriter::TraceBufferedWriter(std::shared_ptr<TraceFileOps> ops)
: ops_(std::move(ops)), buffer_(1024 * 1024) {}
TraceBufferedWriter::~TraceBufferedWriter() {if (fd_ >= 0) {ops_->close(fd_);}}
void TraceBufferedWriter::open(const std::string & path)
{
  if (fd_ >= 0) {throw std::logic_error("writer already open");}
  fd_ = ops_->open(path);
  if (fd_ < 0) {fail("open " + path);}
}
void TraceBufferedWriter::flush() {
    size_t offset = 0;
    while (offset < used_) {
      auto count = ops_->write(fd_, buffer_.data() + offset, used_ - offset);
      if (count < 0 && errno == EINTR) continue;
      if (count < 0) fail("write");
      if (count == 0) throw std::runtime_error("write returned zero");
      offset += static_cast<size_t>(count);
    }
    used_ = 0;
  }
void TraceBufferedWriter::close() {
    flush();
    int fd = fd_;
    fd_ = -1; // Do not retry close after EINTR on Linux.
    if (ops_->close(fd) != 0) fail("close");
  }
void TraceBufferedWriter::text(std::string_view s) {
    while (!s.empty()) {
      size_t n = std::min(s.size(), buffer_.size() - used_);
      std::memcpy(buffer_.data() + used_, s.data(), n);
      used_ += n;
      s.remove_prefix(n);
      if (used_ == buffer_.size()) flush();
    }
  }
void TraceBufferedWriter::number(double value) {
    char chars[128];
    auto result = std::to_chars(chars, chars + sizeof(chars), value, std::chars_format::general);
    if (result.ec != std::errc{}) throw std::runtime_error("double to_chars");
    text({chars, static_cast<size_t>(result.ptr - chars)});
  }

StreamTraceOutput::StreamTraceOutput(const std::string & directory, const std::string & bus,
  std::shared_ptr<TraceFileOps> ops)
: ops_(std::move(ops)), writer_(ops_)
{
  static std::atomic<uint64_t> sequence{0};
  std::string safe = bus;
  for (char & ch : safe) {
    if (!((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
      (ch >= '0' && ch <= '9') || ch == '_' || ch == '-')) {ch = '_';}
  }
  const auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  stem_ = (std::filesystem::path(directory) / ("bxi_stream_" + safe + "_" +
    std::to_string(timestamp) + "_" + std::to_string(::getpid()) + "_" +
    std::to_string(sequence.fetch_add(1)))).string();
  checkAbsent();
  const auto probe = stem_ + ".probe", linked = probe + ".link";
  bool owned = false, link_owned = false;
  try {
    const int fd = ops_->open(probe);
    if (fd < 0) {fail("probe open");}
    owned = true;
    if (ops_->close(fd) != 0) {fail("probe close");}
    if (ops_->link(probe, linked) != 0) {fail("probe link");}
    link_owned = true;
    if (ops_->unlink(linked) != 0) {fail("probe unlink link");}
    link_owned = false;
    if (ops_->unlink(probe) != 0) {fail("probe unlink");}
  } catch (...) {
    if (link_owned) {ops_->unlink(linked);}
    if (owned) {ops_->unlink(probe);}
    throw;
  }
}

void StreamTraceOutput::checkAbsent() const
{
  for (const auto * suffix : {"_cycles.csv", "_rx.csv", ".complete", ".complete.tmp"}) {
    // symlink_status detects dangling symlinks too. Actual creation is O_EXCL/link NOREPLACE.
    if (std::filesystem::exists(std::filesystem::symlink_status(stem_ + suffix))) {
      throw std::runtime_error("existing trace artifact: " + stem_ + suffix);
    }
  }
}

StreamTraceWriteResult StreamTraceOutput::write(const BxiBusStreamer & streamer,
  Clock::time_point report_start)
{
  return write(streamer.trace(), streamer.config(), streamer.stats(),
    streamer.broadcastBlockCount(), streamer.paddingBlock(), report_start);
}

StreamTraceWriteResult StreamTraceOutput::write(const StreamTrace & trace,
  const StreamConfig & config, const StreamStats & stats, size_t blocks,
  const std::array<uint8_t, 8> & padding, Clock::time_point report_start)
{
  checkAbsent();
  StreamTraceWriteResult result;
  result.cycle_rows = std::min(trace.cycle_count, trace.cycles.size());
  result.rx_rows = std::min(trace.rx_count, trace.rx.size());
  result.dropped_cycles = trace.cycle_count - result.cycle_rows;
  result.dropped_rx = trace.rx_count - result.rx_rows;
  if (trace.axis_metadata.size() != trace.axes ||
    trace.age_ns.size() != mul(trace.cycles.size(), trace.axes) ||
    trace.fresh.size() != trace.age_ns.size()) {
    throw std::runtime_error("invalid trace shape");
  }
  const bool detail = config.broadcast && trace.axes == 1;
  std::string ids;
  for (const auto & a : trace.axis_metadata) {
    ids += (ids.empty() ? "" : ";") + std::to_string(a.can_id);
  }
  const auto origin = ns(trace.origin.time_since_epoch());
  std::string metadata = "#period_ns=" + std::to_string(config.period.count()) +
    ",max_feedback_age_ns=" + std::to_string(config.max_feedback_age.count()) +
    ",max_command_age_ns=" + std::to_string(config.max_command_age.count()) +
    ",broadcast=" + (config.broadcast ? "1" : "0") + ",can_ids=" + ids +
    ",dropped_cycles=" + std::to_string(result.dropped_cycles) +
    ",dropped_rx=" + std::to_string(result.dropped_rx) +
    ",tx_thread=" + describe(trace.tx_thread) + ",rx_thread=" + describe(trace.rx_thread) +
    ",format_version=" + std::string(config.write_triggered || config.direct_feedback || config.control_trace ? "3" : "2") +
    ",tx_trigger=" + (config.write_triggered ? "write" : "periodic") +
    ",direct_feedback=" + (config.direct_feedback ? "1" : "0") +
    ",write_grace_ns=" + std::to_string(config.write_grace.count()) +
    ",trace_synthetic=" + (trace.synthetic ? "1" : "0") +
    ",trace_origin_steady_ns=" + std::to_string(origin) +
    ",send_loop_start_steady_ns=" + std::to_string(trace.send_loop_start_steady_ns) +
    ",writes_armed_steady_ns=" + std::to_string(trace.writes_armed_steady_ns) +
    ",stream_stop_steady_ns=" + std::to_string(trace.stream_stop_steady_ns) +
    ",trace_cycles_capacity=" + std::to_string(trace.cycles.size()) +
    ",trace_rx_capacity=" + std::to_string(trace.rx.size()) +
    ",trace_extended=" + (config.trace_extended ? "1" : "0") +
    ",trace_payload_bytes=" + std::to_string(trace.payload_bytes) +
    ",trace_allocation_ns=" + std::to_string(trace.allocation_ns) +
    ",broadcast_block_count=" + std::to_string(blocks) + ",padding_block_hex=" + hexBlock(padding);
  for (const auto & a : trace.axis_metadata) {
    const std::string suffix = trace.axes == 1 ? "" : "_" + std::to_string(a.can_id);
    metadata += ",can_id" + suffix + "=" + std::to_string(a.can_id) +
      ",motor_dir" + suffix + "=" + std::to_string(a.motor_dir);
    const auto field = [&](const char * key, double value) {
        metadata += "," + std::string(key) + suffix + "=" + decimal(value);
      };
    field("offset_rad_effective", a.offset_rad_effective);
    field("joint_lower_rad_effective", a.lower_rad_effective);
    field("joint_upper_rad_effective", a.upper_rad_effective);
    field("p_min", a.limits.p_min); field("p_max", a.limits.p_max);
    field("v_min", a.limits.v_min); field("v_max", a.limits.v_max);
    field("kp_max", a.limits.kp_max); field("kd_max", a.limits.kd_max);
    field("torque_min", a.limits.torque_min); field("torque_max", a.limits.torque_max);
  }
  metadata += '\n';
  const auto csv_start = Clock::now();
  writer_.open(stem_ + "_cycles.csv"); writer_.text(metadata);
  writer_.text("cycle,outcome,wake_lateness_ns,hook_ns,send_lateness_ns,frames");
  for (const auto & a : trace.axis_metadata) {
    writer_.text(",age_ns_" + std::to_string(a.can_id) + ",fresh_" + std::to_string(a.can_id));
  }
  writer_.text(",command_age_ns,tx_detail_supported,command_valid,command_source,command_position_rad,"
    "command_velocity_rad_s,command_kp,command_kd,command_effort_nm,encoded_valid,command_block_hex,"
    "send_attempted,send_success,send_before_steady_ns,send_after_steady_ns");
  if (config.write_triggered || config.direct_feedback || config.control_trace) {
    writer_.text(",command_sequence,command_published_ns,wake_steady_ns,resend_due_ns,hold_ns,send_kind,snapshot_failed");
  }
  writer_.text("\n");
  for (size_t i = 0; i < result.cycle_rows; ++i) {
    const auto & r = trace.cycles[i];
    writer_.field(r.cycle); writer_.text(streamCycleOutcomeName(r.outcome)); writer_.text(",");
    writer_.field(r.wake_lateness_ns); writer_.field(r.hook_ns); writer_.field(r.send_lateness_ns);
    writer_.number(r.frames);
    for (size_t a = 0; a < trace.axes; ++a) {
      writer_.text(","); writer_.field(trace.age_ns[i * trace.axes + a]);
      writer_.number(trace.fresh[i * trace.axes + a]);
    }
    writer_.text(","); writer_.field(r.command_age_ns); writer_.field(detail ? 1 : 0);
    writer_.field(detail && r.command_valid ? 1 : 0);
    writer_.text(detail && r.command_valid ? (r.command_stale ? "command_stale," : "owner,") : "none,");
    if (detail && r.command_valid) {
      writer_.field(r.command.position); writer_.field(r.command.velocity); writer_.field(r.command.kp);
      writer_.field(r.command.kd); writer_.field(r.command.effort);
    } else {writer_.text(",,,,,");}
    writer_.field(detail && r.encoded_valid ? 1 : 0);
    if (detail && r.encoded_valid) {writer_.text(hexBlock(r.command_block));}
    const bool send_timing = detail || config.write_triggered || config.control_trace;
    writer_.text(","); writer_.field(send_timing && r.send_attempted ? 1 : 0);
    if (send_timing && r.send_attempted) {
      writer_.field(r.send_success ? 1 : 0); writer_.field(r.send_before_steady_ns);
      writer_.number(r.send_after_steady_ns);
    } else {writer_.text(",,");}
    if (config.write_triggered || config.direct_feedback || config.control_trace) {
      writer_.text(","); writer_.field(r.command_sequence); writer_.field(r.command_published_ns);
      writer_.field(r.wake_steady_ns); writer_.field(r.resend_due_ns); writer_.field(r.hold_ns);
      writer_.field(r.send_kind); writer_.number(r.snapshot_failed ? 1 : 0);
    }
    writer_.text("\n");
  }
  writer_.close();
  writer_.open(stem_ + "_rx.csv"); writer_.text(metadata);
  writer_.text("can_id,received_ns,rx_kernel_realtime_ns,rx_userspace_realtime_ns,rx_hardware_raw_ns,"
    "received_steady_ns,motor_position_rad,motor_velocity_rad_s,motor_torque_nm");
  if (config.direct_feedback || config.write_triggered || config.control_trace) {writer_.text(",rx_sequence,published_steady_ns");}
  writer_.text("\n");
  for (size_t i = 0; i < result.rx_rows; ++i) {
    const auto & r = trace.rx[i];
    if (r.axis >= trace.axes) {throw std::runtime_error("invalid RX axis");}
    writer_.field(trace.axis_metadata[r.axis].can_id); writer_.field(r.received_ns);
    writer_.field(r.rx_kernel_realtime_ns); writer_.field(r.rx_userspace_realtime_ns);
    writer_.field(r.rx_hardware_raw_ns); writer_.field(origin + r.received_ns);
    writer_.field(r.motor_position); writer_.field(r.motor_velocity); writer_.number(r.motor_torque);
    if (config.direct_feedback || config.write_triggered || config.control_trace) {
      writer_.text(","); writer_.field(r.sequence); writer_.number(r.published_ns);
    }
    writer_.text("\n");
  }
  writer_.close();
  result.csv_write_elapsed_ns = ns(Clock::now() - csv_start);
  const auto basename = std::filesystem::path(stem_).filename().string();
  writer_.open(stem_ + ".complete.tmp");
  writer_.text("{\"format_version\":" + std::string(config.write_triggered || config.direct_feedback || config.control_trace ? "3" : "2") +
    ",\"resends\":" + std::to_string(stats.resends) +
    ",\"feedback_stale_at_stop\":" + (stats.feedback_stale_at_stop ? "true" : "false") +
    ",\"received_fault_at_stop\":\"" + streamFaultName(stats.received_fault_at_stop) + "\"" +
    ",\"minimum_interval_holds\":" + std::to_string(stats.minimum_interval_holds) +
    ",\"writes_after_resend\":" + std::to_string(stats.writes_after_resend) +
    ",\"resend_command_holds\":" + std::to_string(stats.resend_command_holds) +
    ",\"new_command_holds\":" + std::to_string(stats.new_command_holds) +
    ",\"bootstrap_command_holds\":" + std::to_string(stats.bootstrap_command_holds) +
    ",\"command_snapshot_failures\":" + std::to_string(stats.command_snapshot_failures) +
    ",\"max_hold_ns\":" + std::to_string(stats.max_hold_ns) +
    ",\"cycles_file\":\"" + basename + "_cycles.csv\",\"rx_file\":\"" +
    basename + "_rx.csv\",\"cycle_rows\":");
  writer_.number(result.cycle_rows); writer_.text(",\"rx_rows\":"); writer_.number(result.rx_rows);
  writer_.text(",\"temperature_fault_axis\":"); writer_.number(stats.temperature_fault_axis);
  writer_.text(",\"temperature_fault_c\":");
  if (stats.temperature_fault_axis >= 0 && std::isfinite(stats.temperature_fault_c)) {
    writer_.number(stats.temperature_fault_c);
  } else {writer_.text("null");}
  writer_.text(",\"temperature_fault_received_ns\":"); writer_.number(stats.temperature_fault_received_ns);
  writer_.text(",\"temperature_sensors_failed\":");
  writer_.text(stats.temperature_fault_axis >= 0 && !std::isfinite(stats.temperature_fault_c) ? "true" : "false");
  writer_.text(",\"dropped_cycles\":"); writer_.number(result.dropped_cycles);
  writer_.text(",\"dropped_rx\":"); writer_.number(result.dropped_rx);
  writer_.text(",\"fault\":\""); writer_.text(streamFaultName(stats.fault));
  writer_.text("\",\"fault_axis\":"); writer_.number(stats.fault_axis);
  writer_.text(",\"stopped_externally\":"); writer_.text(stats.stopped_externally ? "true" : "false");
  writer_.text(",\"stop_reason\":\"");
  writer_.text(stats.fault != StreamFault::kNone ? "fault" :
    (stats.stopped_externally ? "external" : "requested_stop"));
  writer_.text("\",\"trace_origin_steady_ns\":"); writer_.number(origin);
  writer_.text(",\"send_loop_start_steady_ns\":"); writer_.number(trace.send_loop_start_steady_ns);
  writer_.text(",\"stream_stop_steady_ns\":"); writer_.number(trace.stream_stop_steady_ns);
  writer_.text(",\"csv_write_elapsed_ns\":"); writer_.number(result.csv_write_elapsed_ns);
  writer_.text("}\n"); writer_.close();
  if (ops_->link(stem_ + ".complete.tmp", stem_ + ".complete") != 0) {fail("complete link");}
  result.report_to_complete_ns = ns(Clock::now() - report_start);
  result.tmp_cleanup_failed = ops_->unlink(stem_ + ".complete.tmp") != 0;
  return result;
}
}  // namespace bxi_hardware
