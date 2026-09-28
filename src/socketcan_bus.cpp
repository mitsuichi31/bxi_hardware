// mevius2_hardware から複製（namespace を bxi_hardware に変更）。CAN FD 送受信を追加。
#include "bxi_hardware/can/socketcan_bus.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <ctime>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/errqueue.h>
#include <linux/net_tstamp.h>

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
  if (rx_timestamp_mode_ == RxTimestampMode::kSoftware) {
    const int enabled = 1;
    if (::setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPNS, &enabled, sizeof(enabled)) < 0) {
      RCLCPP_ERROR(logger(), "[%s] SO_TIMESTAMPNS failed: %s",
        ifname.c_str(), std::strerror(errno));
      ::close(fd_);
      fd_ = -1;
      return false;
    }
  } else if (rx_timestamp_mode_ == RxTimestampMode::kHardware) {
    // Request both timestamp types in a single SCM_TIMESTAMPING_NEW record. Do not combine
    // SO_TIMESTAMPNS with SO_TIMESTAMPING: the kernel documents a false software timestamp case.
    const int flags = SOF_TIMESTAMPING_RX_HARDWARE | SOF_TIMESTAMPING_RAW_HARDWARE |
      SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE;
    if (::setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPING_NEW, &flags, sizeof(flags)) < 0) {
      RCLCPP_ERROR(logger(), "[%s] SO_TIMESTAMPING_NEW failed: %s",
        ifname.c_str(), std::strerror(errno));
      ::close(fd_);
      fd_ = -1;
      return false;
    }
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

bool SocketCanBus::setReceiveEnabled(bool enabled)
{
  std::lock_guard<std::mutex> lk(io_mutex_);
  if (fd_ < 0) {
    return false;
  }
  const struct can_filter accept_all{0, 0};
  const void * filters = enabled ? static_cast<const void *>(&accept_all) : nullptr;
  const socklen_t length = enabled ? sizeof(accept_all) : 0;
  if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters, length) < 0) {
    RCLCPP_ERROR(logger(), "[%s] CAN_RAW_FILTER failed: %s", ifname_.c_str(),
      std::strerror(errno));
    return false;
  }
  return true;
}

bool SocketCanBus::setReceiveStandardIds(const std::vector<uint32_t> & ids)
{
  std::lock_guard<std::mutex> lk(io_mutex_);
  if (fd_ < 0 || ids.empty()) {
    return false;
  }
  std::vector<struct can_filter> filters;
  filters.reserve(ids.size());
  for (const uint32_t id : ids) {
    if (id > CAN_SFF_MASK) {
      return false;
    }
    // Match the ID exactly and exclude extended-ID and remote frames.
    filters.push_back({id, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG});
  }
  if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
    static_cast<socklen_t>(filters.size() * sizeof(filters[0]))) < 0)
  {
    RCLCPP_ERROR(logger(), "[%s] CAN_RAW_FILTER failed: %s", ifname_.c_str(),
      std::strerror(errno));
    return false;
  }
  return true;
}

bool SocketCanBus::send(const CanFrame & frame)
{
  std::lock_guard<std::mutex> lk(io_mutex_);
  if (fd_ < 0) {
    return false;
  }
  const bool length_ok = format_ == CanFrameFormat::kClassic ?
    frame.dlc <= kClassicCanMaxDataLength : isValidCanFdDataLength(frame.dlc);
  if (!length_ok) {
    static rclcpp::Clock clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(logger(), clock, 1000, "[%s] refusing %u-byte frame on a %s bus",
      ifname_.c_str(), static_cast<unsigned>(frame.dlc),
      format_ == CanFrameFormat::kClassic ? "classic" : "CAN FD");
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

  // FD frames up to 64 bytes are returned as-is. Callers that expect an 8-byte MIT reply check
  // both the ID and the length.
  using clock = std::chrono::steady_clock;
  const auto deadline = clock::now() + std::chrono::milliseconds(std::max(timeout_ms, 0));
  while (true) {
    // select() can keep reporting a busy bus as readable even after the deadline.
    // Keep the zero-timeout probe used by flushRx() nonblocking but still able to read a frame.
    if (timeout_ms > 0 && clock::now() >= deadline) {
      return false;
    }
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
    int64_t kernel_realtime_ns = 0;
    int64_t userspace_realtime_ns = 0;
    int64_t hardware_raw_ns = 0;
    ssize_t n;
    if (rx_timestamp_mode_ != RxTimestampMode::kOff) {
      struct iovec iov{&cf, sizeof(cf)};
      alignas(struct cmsghdr) char control[CMSG_SPACE(sizeof(struct scm_timestamping64))]{};
      struct msghdr msg{};
      msg.msg_iov = &iov;
      msg.msg_iovlen = 1;
      msg.msg_control = control;
      msg.msg_controllen = sizeof(control);
      n = ::recvmsg(fd_, &msg, 0);
      struct timespec received_at{};
      if (::clock_gettime(CLOCK_REALTIME, &received_at) == 0) {
        userspace_realtime_ns = static_cast<int64_t>(received_at.tv_sec) * 1000000000LL +
          received_at.tv_nsec;
      }
      if ((msg.msg_flags & MSG_CTRUNC) == 0) {
        for (struct cmsghdr * cmsg = CMSG_FIRSTHDR(&msg); cmsg != nullptr;
          cmsg = CMSG_NXTHDR(&msg, cmsg))
        {
          if (rx_timestamp_mode_ == RxTimestampMode::kSoftware &&
            cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SCM_TIMESTAMPNS &&
            cmsg->cmsg_len >= CMSG_LEN(sizeof(struct timespec)))
          {
            struct timespec stamped_at{};
            std::memcpy(&stamped_at, CMSG_DATA(cmsg), sizeof(stamped_at));
            kernel_realtime_ns = static_cast<int64_t>(stamped_at.tv_sec) * 1000000000LL +
              stamped_at.tv_nsec;
            break;
          } else if (rx_timestamp_mode_ == RxTimestampMode::kHardware &&
            cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMPING_NEW &&
            cmsg->cmsg_len >= CMSG_LEN(sizeof(struct scm_timestamping64)))
          {
            struct scm_timestamping64 stamps{};
            std::memcpy(&stamps, CMSG_DATA(cmsg), sizeof(stamps));
            if (stamps.ts[0].tv_sec != 0 || stamps.ts[0].tv_nsec != 0) {
              kernel_realtime_ns = static_cast<int64_t>(stamps.ts[0].tv_sec) * 1000000000LL +
                stamps.ts[0].tv_nsec;
            }
            if (stamps.ts[2].tv_sec != 0 || stamps.ts[2].tv_nsec != 0) {
              hardware_raw_ns = static_cast<int64_t>(stamps.ts[2].tv_sec) * 1000000000LL +
                stamps.ts[2].tv_nsec;
            }
            break;
          }
        }
      }
    } else {
      n = ::read(fd_, &cf, sizeof(cf));
    }
    if (n != static_cast<ssize_t>(CAN_MTU) && n != static_cast<ssize_t>(CANFD_MTU)) {
      return false;
    }
    if (cf.len > out.data.size()) {
      return false;  // Not produced by SocketCAN; guards the copy below.
    }
    out.extended = (cf.can_id & CAN_EFF_FLAG) != 0;
    out.id = cf.can_id & (out.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    out.dlc = cf.len;
    out.fd = n == static_cast<ssize_t>(CANFD_MTU);
    out.brs = out.fd && (cf.flags & CANFD_BRS) != 0;
    std::memset(out.data.data(), 0, out.data.size());
    std::memcpy(out.data.data(), cf.data, cf.len);
    out.rx_kernel_realtime_ns = kernel_realtime_ns;
    out.rx_userspace_realtime_ns = userspace_realtime_ns;
    out.rx_hardware_raw_ns = hardware_raw_ns;
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
