// トランスポート層（抽象）: CANフレームの送受信のみを担う。
// mevius2_hardware から複製（namespace のみ変更）。
#ifndef BXI_HARDWARE__CAN__CAN_BUS_HPP_
#define BXI_HARDWARE__CAN__CAN_BUS_HPP_

#include <array>
#include <cstdint>
#include <string>

namespace bxi_hardware
{

struct CanFrame
{
  uint32_t id = 0;
  bool extended = true;
  uint8_t dlc = 0;
  std::array<uint8_t, 8> data{};
};

class CanBus
{
public:
  virtual ~CanBus() = default;

  virtual bool open(const std::string & ifname) = 0;
  virtual void close() = 0;
  virtual bool isOpen() const = 0;

  virtual bool send(const CanFrame & frame) = 0;
  virtual bool recv(CanFrame & out, int timeout_ms) = 0;
  virtual void flushRx() = 0;

  virtual const std::string & name() const = 0;
};

}  // namespace bxi_hardware

#endif  // BXI_HARDWARE__CAN__CAN_BUS_HPP_
