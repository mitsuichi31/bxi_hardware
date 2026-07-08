// トランスポート層（実装）: Linux SocketCAN を raw socket で叩く。
// mevius2_hardware から複製（namespace のみ変更）。
#ifndef BXI_HARDWARE__CAN__SOCKETCAN_BUS_HPP_
#define BXI_HARDWARE__CAN__SOCKETCAN_BUS_HPP_

#include <mutex>
#include <string>

#include "bxi_hardware/can/can_bus.hpp"

namespace bxi_hardware
{

class SocketCanBus : public CanBus
{
public:
  SocketCanBus() = default;
  ~SocketCanBus() override;

  bool open(const std::string & ifname) override;
  void close() override;
  bool isOpen() const override { return fd_ >= 0; }

  bool send(const CanFrame & frame) override;
  bool recv(CanFrame & out, int timeout_ms) override;
  void flushRx() override;

  const std::string & name() const override { return ifname_; }

private:
  int fd_ = -1;
  std::string ifname_;
  std::mutex io_mutex_;
};

}  // namespace bxi_hardware

#endif  // BXI_HARDWARE__CAN__SOCKETCAN_BUS_HPP_
