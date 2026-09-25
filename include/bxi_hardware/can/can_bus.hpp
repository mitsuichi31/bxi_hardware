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
  // Set on received frames. Outgoing frames use the bus-wide format (see SocketCanBus).
  bool fd = false;
  bool brs = false;
};

// Frame format used for every transmitted frame on a bus. BXI motors reply in the format they
// received (confirmed on hardware 2026-09-25), so the whole bus uses one format.
enum class CanFrameFormat
{
  kClassic,
  kFd,       // CAN FD, nominal bit rate for the data phase
  kFdBrs,    // CAN FD with bit rate switch
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
