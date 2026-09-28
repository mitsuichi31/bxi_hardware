// トランスポート層（実装）: Linux SocketCAN を raw socket で叩く。
// mevius2_hardware から複製（namespace のみ変更）。
#ifndef BXI_HARDWARE__CAN__SOCKETCAN_BUS_HPP_
#define BXI_HARDWARE__CAN__SOCKETCAN_BUS_HPP_

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "bxi_hardware/can/can_bus.hpp"

namespace bxi_hardware
{

class SocketCanBus : public CanBus
{
public:
  explicit SocketCanBus(CanFrameFormat format = CanFrameFormat::kClassic)
  : format_(format) {}
  ~SocketCanBus() override;

  bool open(const std::string & ifname) override;
  void close() override;
  bool isOpen() const override { return fd_ >= 0; }

  // Classic buses send at most 8 bytes; FD buses send any valid CAN FD length up to 64 bytes.
  bool send(const CanFrame & frame) override;
  bool recv(CanFrame & out, int timeout_ms) override;
  void flushRx() override;
  // Configure before open(). Normal runs keep the original read() path.
  void setRxTimestamping(bool enabled) {
    rx_timestamp_mode_ = enabled ? RxTimestampMode::kSoftware : RxTimestampMode::kOff;
  }
  void setRxHardwareTimestamping(bool enabled) {
    rx_timestamp_mode_ = enabled ? RxTimestampMode::kHardware : RxTimestampMode::kOff;
  }
  // Disable the receive filter on a send-only diagnostic socket, then restore it afterward.
  bool setReceiveEnabled(bool enabled);
  // Accept only these standard (11-bit) data-frame IDs on an open socket. An empty list is
  // rejected; use setReceiveEnabled(false) for a send-only socket.
  bool setReceiveStandardIds(const std::vector<uint32_t> & ids);

  const std::string & name() const override { return ifname_; }
  CanFrameFormat format() const { return format_; }

private:
  CanFrameFormat format_;
  int fd_ = -1;
  std::string ifname_;
  std::mutex io_mutex_;
  enum class RxTimestampMode { kOff, kSoftware, kHardware };
  RxTimestampMode rx_timestamp_mode_ = RxTimestampMode::kOff;
};

}  // namespace bxi_hardware

#endif  // BXI_HARDWARE__CAN__SOCKETCAN_BUS_HPP_
