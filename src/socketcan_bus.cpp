// mevius2_hardware から複製（namespace を bxi_hardware に変更）。
#include "bxi_hardware/can/socketcan_bus.hpp"

#include <cerrno>
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

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(logger(), "[%s] bind() failed: %s", ifname.c_str(), std::strerror(errno));
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  RCLCPP_INFO(logger(), "[%s] SocketCAN open", ifname.c_str());
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
  struct can_frame cf{};
  cf.can_id = frame.id & CAN_EFF_MASK;
  if (frame.extended) {
    cf.can_id |= CAN_EFF_FLAG;
  }
  cf.can_dlc = frame.dlc;
  std::memcpy(cf.data, frame.data.data(), frame.dlc);

  const ssize_t n = ::write(fd_, &cf, sizeof(cf));
  if (n != static_cast<ssize_t>(sizeof(cf))) {
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

  if (timeout_ms >= 0) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    const int r = ::select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
    if (r <= 0) {
      return false;
    }
  }

  struct can_frame cf{};
  const ssize_t n = ::read(fd_, &cf, sizeof(cf));
  if (n != static_cast<ssize_t>(sizeof(cf))) {
    return false;
  }
  out.extended = (cf.can_id & CAN_EFF_FLAG) != 0;
  out.id = cf.can_id & (out.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
  out.dlc = cf.can_dlc;
  std::memset(out.data.data(), 0, out.data.size());
  std::memcpy(out.data.data(), cf.data, cf.can_dlc);
  return true;
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
