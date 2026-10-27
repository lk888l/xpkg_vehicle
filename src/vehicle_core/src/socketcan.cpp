// SPDX-License-Identifier: Apache-2.0
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <system_error>

#include "vehicle_core/transport.hpp"

namespace vehicle_core {
namespace {
[[noreturn]] void fail(const char* operation) {
  throw std::system_error(errno, std::generic_category(), operation);
}
}  // namespace
SocketCan::SocketCan(const std::string& interface, Address address) {
  const unsigned int index = if_nametoindex(interface.c_str());
  if (index == 0) fail("CAN interface lookup");
  fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
  if (fd_ < 0) fail("CAN socket");
  try {
    // CAN_ERR_FLAG must not appear in this mask: the kernel would register
    // it as an error subscription instead of a data-frame filter.
    const can_filter filter{address.base() | CAN_EFF_FLAG,
                            0x1FFFFF00U | CAN_EFF_FLAG | CAN_RTR_FLAG};
    if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0)
      fail("CAN filter");
    const can_err_mask_t errors = CAN_ERR_MASK;
    if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errors, sizeof(errors)) < 0)
      fail("CAN error filter");
    const int one = 1;
    if (::setsockopt(fd_, SOL_SOCKET, SO_RXQ_OVFL, &one, sizeof(one)) < 0)
      fail("CAN drop accounting");
    const int buffer_bytes = 256 * 1024;
    if (::setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &buffer_bytes, sizeof(buffer_bytes)) < 0)
      fail("CAN receive buffer");
    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = static_cast<int>(index);
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) fail("CAN bind");
  } catch (...) {
    ::close(fd_);
    fd_ = -1;
    throw;
  }
}
SocketCan::~SocketCan() {
  if (fd_ >= 0) ::close(fd_);
}
void SocketCan::send(const Frame& frame) {
  if (frame.size > 8 || !frame.extended || frame.remote || frame.error || frame.id > CAN_EFF_MASK)
    throw std::invalid_argument("only classic extended CAN data frames may be transmitted");
  can_frame raw{};
  raw.can_id = frame.id | CAN_EFF_FLAG;
  raw.len = frame.size;
  std::copy_n(frame.data.begin(), frame.size, raw.data);
  ssize_t count;
  do {
    count = ::write(fd_, &raw, CAN_MTU);
  } while (count < 0 && errno == EINTR);
  if (count < 0) fail("CAN send");
  if (count != CAN_MTU) throw std::runtime_error("short CAN write");
}
std::optional<Frame> SocketCan::receive() {
  can_frame raw{};
  alignas(cmsghdr) char control[CMSG_SPACE(sizeof(uint32_t))]{};
  iovec io{&raw, sizeof(raw)};
  msghdr msg{};
  msg.msg_iov = &io;
  msg.msg_iovlen = 1;
  msg.msg_control = control;
  msg.msg_controllen = sizeof(control);
  ssize_t count;
  do {
    count = ::recvmsg(fd_, &msg, MSG_DONTWAIT);
  } while (count < 0 && errno == EINTR);
  if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return std::nullopt;
  if (count < 0) fail("CAN receive");
  if (count != CAN_MTU || (msg.msg_flags & MSG_TRUNC) != 0 || raw.len > 8)
    throw std::runtime_error("invalid classic CAN datagram");
  for (auto* c = CMSG_FIRSTHDR(&msg); c != nullptr; c = CMSG_NXTHDR(&msg, c)) {
    if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SO_RXQ_OVFL &&
        c->cmsg_len >= CMSG_LEN(sizeof(dropped_))) {
      std::memcpy(&dropped_, CMSG_DATA(c), sizeof(dropped_));
    }
  }
  Frame frame{};
  frame.id = raw.can_id & CAN_EFF_MASK;
  frame.size = raw.len;
  frame.extended = (raw.can_id & CAN_EFF_FLAG) != 0;
  frame.remote = (raw.can_id & CAN_RTR_FLAG) != 0;
  frame.error = (raw.can_id & CAN_ERR_FLAG) != 0;
  std::copy_n(raw.data, raw.len, frame.data.begin());
  return frame;
}
}  // namespace vehicle_core
