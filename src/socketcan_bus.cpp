// mevius2_hardware から複製（namespace を bxi_hardware に変更）。CAN FD 送受信を追加。
#include "bxi_hardware/can/socketcan_bus.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <rclcpp/rclcpp.hpp>

namespace bxi_hardware
{

static rclcpp::Logger logger() { return rclcpp::get_logger("SocketCanBus"); }

SocketCanBus::~SocketCanBus() { close(); }

bool SocketCanBus::open(const std::string & ifname)
{
  std::lock_guard<std::mutex> lk(io_mutex_);
  if (fd_ >= 0) {
    return true;
  }
  ifname_ = ifname;

  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    RCLCPP_ERROR(logger(), "[%s] socket() failed: %s", ifname.c_str(), std::strerror(errno));
    return false;
  }

  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(logger(), "[%s] ioctl(SIOCGIFINDEX) failed: %s (CAN未接続?)",
                 ifname.c_str(), std::strerror(errno));
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // ifr_ifindex and ifr_mtu share a union: keep the index before querying the MTU.
  const int ifindex = ifr.ifr_ifindex;
  if (format_ != CanFrameFormat::kClassic) {
    // The interface must be configured with "fd on" (MTU 72) before FD frames can be sent.
    if (::ioctl(fd_, SIOCGIFMTU, &ifr) < 0 || ifr.ifr_mtu != CANFD_MTU) {
      RCLCPP_ERROR(logger(), "[%s] CAN FD requested but the interface MTU is not %d "
                   "(ip link set %s type can ... fd on)", ifname.c_str(), static_cast<int>(CANFD_MTU),
                   ifname.c_str());
      ::close(fd_);
      fd_ = -1;
      return false;
    }
  }
  // Receive FD frames even on a classic bus so that an FD reply is never silently dropped.
  const int enable_fd = 1;
  if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) < 0 &&
    format_ != CanFrameFormat::kClassic)
  {
    RCLCPP_ERROR(logger(), "[%s] setsockopt(CAN_RAW_FD_FRAMES) failed: %s",
                 ifname.c_str(), std::strerror(errno));
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifindex;
  if (::bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(logger(), "[%s] bind() failed: %s", ifname.c_str(), std::strerror(errno));
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  static constexpr const char * kFormatNames[] = {"classic", "fd", "fd_brs"};
  RCLCPP_INFO(logger(), "[%s] SocketCAN open (%s)", ifname.c_str(),
              kFormatNames[static_cast<int>(format_)]);
  return true;
}

void SocketCanBus::close()
{
  std::lock_guard<std::mutex> lk(io_mutex_);
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SocketCanBus::send(const CanFrame & frame)
{
  std::lock_guard<std::mutex> lk(io_mutex_);
  if (fd_ < 0) {
    return false;
  }
  if (frame.dlc > frame.data.size()) {
    return false;
  }
  struct canfd_frame cf{};
  cf.can_id = frame.id & CAN_EFF_MASK;
  if (frame.extended) {
    cf.can_id |= CAN_EFF_FLAG;
  }
  cf.len = frame.dlc;
  std::memcpy(cf.data, frame.data.data(), frame.dlc);
  size_t size = CAN_MTU;
  if (format_ != CanFrameFormat::kClassic) {
    cf.flags = format_ == CanFrameFormat::kFdBrs ? CANFD_BRS : 0;
    size = CANFD_MTU;
  }

  const ssize_t n = ::write(fd_, &cf, size);
  if (n != static_cast<ssize_t>(size)) {
    static rclcpp::Clock clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(logger(), clock, 1000,
                         "[%s] write() failed: %s", ifname_.c_str(), std::strerror(errno));
    return false;
  }
  return true;
}

bool SocketCanBus::recv(CanFrame & out, int timeout_ms)
{
  std::lock_guard<std::mutex> lk(io_mutex_);
  if (fd_ < 0) {
    return false;
  }

  // Frames longer than 8 bytes (e.g. 64-byte FD broadcasts from another node) do not fit
  // CanFrame and are skipped until the deadline, like any other unrelated frame.
  using clock = std::chrono::steady_clock;
  const auto deadline = clock::now() + std::chrono::milliseconds(std::max(timeout_ms, 0));
  while (true) {
    if (timeout_ms >= 0) {
      const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(
        deadline - clock::now()).count();
      fd_set rfds;
      FD_ZERO(&rfds);
      FD_SET(fd_, &rfds);
      struct timeval tv;
      tv.tv_sec = std::max<int64_t>(remaining, 0) / 1000000;
      tv.tv_usec = std::max<int64_t>(remaining, 0) % 1000000;
      const int r = ::select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
      if (r <= 0) {
        return false;
      }
    }

    struct canfd_frame cf{};
    const ssize_t n = ::read(fd_, &cf, sizeof(cf));
    if (n != static_cast<ssize_t>(CAN_MTU) && n != static_cast<ssize_t>(CANFD_MTU)) {
      return false;
    }
    if (cf.len > out.data.size()) {
      if (timeout_ms == 0) {
        return false;
      }
      continue;
    }
    out.extended = (cf.can_id & CAN_EFF_FLAG) != 0;
    out.id = cf.can_id & (out.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    out.dlc = cf.len;
    out.fd = n == static_cast<ssize_t>(CANFD_MTU);
    out.brs = out.fd && (cf.flags & CANFD_BRS) != 0;
    std::memset(out.data.data(), 0, out.data.size());
    std::memcpy(out.data.data(), cf.data, cf.len);
    return true;
  }
}

void SocketCanBus::flushRx()
{
  CanFrame dummy;
  int guard = 0;
  while (recv(dummy, 0) && guard < 64) {
    ++guard;
  }
}

}  // namespace bxi_hardware
