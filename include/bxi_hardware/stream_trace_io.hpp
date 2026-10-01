#ifndef BXI_HARDWARE__STREAM_TRACE_IO_HPP_
#define BXI_HARDWARE__STREAM_TRACE_IO_HPP_

#include <charconv>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>
#include <sys/types.h>
#include "bxi_hardware/bus_streamer.hpp"

namespace bxi_hardware
{
constexpr size_t kStreamTraceMemoryLimit = 256 * 1024 * 1024;
size_t streamTracePayloadBytes(size_t cycles, size_t axes);
size_t parseStreamTraceCycles(const char * setting);
std::unique_ptr<StreamTrace> prepareStreamTrace(size_t cycles, size_t axes);

// Virtual syscalls only for deterministic failure/short-write tests. Production uses POSIX.
struct TraceFileOps
{
  virtual ~TraceFileOps() = default;
  virtual int open(const std::string & path);
  virtual ssize_t write(int fd, const void * data, size_t size);
  virtual int close(int fd);
  virtual int link(const std::string & from, const std::string & to);
  virtual int unlink(const std::string & path);
};

// Serialization path moved from the validated standalone prototype.
class TraceBufferedWriter
{
public:
  explicit TraceBufferedWriter(std::shared_ptr<TraceFileOps> ops);
  ~TraceBufferedWriter();
  TraceBufferedWriter(const TraceBufferedWriter &) = delete;
  TraceBufferedWriter & operator=(const TraceBufferedWriter &) = delete;
  void open(const std::string & path);
  void flush();
  void close();
  void text(std::string_view value);
  template<class T> void number(T value)
  {
    char chars[128];
    const auto result = std::to_chars(chars, chars + sizeof(chars), value);
    if (result.ec != std::errc{}) {throw std::runtime_error("integer to_chars");}
    text({chars, static_cast<size_t>(result.ptr - chars)});
  }
  void number(double value);
  template<class T> void field(T value) {number(value); text(",");}
private:
  std::shared_ptr<TraceFileOps> ops_;
  int fd_ = -1;
  std::vector<char> buffer_;
  size_t used_ = 0;
};

struct StreamTraceWriteResult
{
  size_t cycle_rows = 0, rx_rows = 0;
  size_t dropped_cycles = 0, dropped_rx = 0;
  int64_t csv_write_elapsed_ns = 0;
  int64_t report_to_complete_ns = 0;
  bool tmp_cleanup_failed = false;
};

// Construct before enable: allocate buffer, choose a unique stem, check collisions and links.
class StreamTraceOutput
{
public:
  StreamTraceOutput(const std::string & directory, const std::string & bus,
    std::shared_ptr<TraceFileOps> ops = std::make_shared<TraceFileOps>());
  virtual ~StreamTraceOutput() = default;
  const std::string & stem() const {return stem_;}
  virtual StreamTraceWriteResult write(const BxiBusStreamer & streamer,
    std::chrono::steady_clock::time_point report_start);
  // Same writer for maximum-capacity benchmarks and fault-injection tests after stop/join.
  StreamTraceWriteResult write(const StreamTrace & trace, const StreamConfig & config,
    const StreamStats & stats, size_t blocks, const std::array<uint8_t, 8> & padding,
    std::chrono::steady_clock::time_point report_start);
private:
  void checkAbsent() const;
  std::string stem_;
  std::shared_ptr<TraceFileOps> ops_;
  TraceBufferedWriter writer_;
};
}  // namespace bxi_hardware
#endif
