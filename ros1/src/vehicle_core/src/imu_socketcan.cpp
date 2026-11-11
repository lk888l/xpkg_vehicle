// SPDX-License-Identifier: Apache-2.0
// can_dlc supports Ubuntu 20.04 Linux CAN headers; it aliases len on newer kernels.
#include "vehicle_core/imu.hpp"
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

namespace vehicle_core {
namespace {
[[noreturn]] void fail(const char* operation) {
  throw std::system_error(errno, std::generic_category(), operation);
}
}
ImuSocketCan::ImuSocketCan(const std::string& interface, uint8_t node) {
  if (node == 0 || node > 127 || node == 100 || node == 101)
    throw std::invalid_argument("IMU node must be 1..127, excluding 100 and 101");
  const unsigned int index = if_nametoindex(interface.c_str());
  if (!index) fail("IMU CAN interface lookup");
  fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
  if (fd_ < 0) fail("IMU CAN socket");
  try {
    const int one = 1;
    if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &one, sizeof(one)) < 0) fail("IMU CAN FD");
    const canid_t mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
    const can_filter filters[] = {{0x180U + node, mask}, {0x280U + node, mask},
                                  {0x380U + node, mask}, {0x480U + node, mask}, {0x700U + node, mask}};
    if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters, sizeof(filters)) < 0) fail("IMU CAN filter");
    const can_err_mask_t errors = CAN_ERR_MASK;
    if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errors, sizeof(errors)) < 0) fail("IMU CAN errors");
    if (::setsockopt(fd_, SOL_SOCKET, SO_RXQ_OVFL, &one, sizeof(one)) < 0) fail("IMU CAN drops");
    const int buffer = 256 * 1024;
    if (::setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &buffer, sizeof(buffer)) < 0) fail("IMU CAN buffer");
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = static_cast<int>(index);
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) fail("IMU CAN bind");
  } catch (...) { ::close(fd_); fd_ = -1; throw; }
}
ImuSocketCan::~ImuSocketCan() { if (fd_ >= 0) ::close(fd_); }
void ImuSocketCan::start(uint8_t node) {
  if (node == 0 || node > 127) throw std::invalid_argument("NMT Start requires a node ID");
  can_frame frame{};
  frame.can_dlc = 2;
  frame.data[0] = 1;
  frame.data[1] = node;
  ssize_t count;
  do { count = ::write(fd_, &frame, CAN_MTU); } while (count < 0 && errno == EINTR);
  if (count < 0) fail("IMU NMT Start");
  if (count != CAN_MTU) throw std::runtime_error("short IMU NMT write");
}
std::optional<ImuFrame> ImuSocketCan::receive() {
  canfd_frame raw{};
  alignas(cmsghdr) char control[CMSG_SPACE(sizeof(uint32_t))]{};
  iovec io{&raw, sizeof(raw)};
  msghdr msg{};
  msg.msg_iov = &io; msg.msg_iovlen = 1;
  msg.msg_control = control; msg.msg_controllen = sizeof(control);
  ssize_t count;
  do { count = ::recvmsg(fd_, &msg, MSG_DONTWAIT); } while (count < 0 && errno == EINTR);
  if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return std::nullopt;
  if (count < 0) fail("IMU CAN receive");
  if ((count != CAN_MTU && count != CANFD_MTU) || (msg.msg_flags & MSG_TRUNC) ||
      raw.len > (count == CAN_MTU ? 8 : 64)) throw std::runtime_error("invalid IMU CAN datagram");
  for (auto* c = CMSG_FIRSTHDR(&msg); c; c = CMSG_NXTHDR(&msg, c))
    if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SO_RXQ_OVFL && c->cmsg_len >= CMSG_LEN(sizeof(dropped_)))
      std::memcpy(&dropped_, CMSG_DATA(c), sizeof(dropped_));
  ImuFrame f;
  f.id = raw.can_id & CAN_EFF_MASK; f.size = raw.len;
  f.fd = count == CANFD_MTU; f.brs = f.fd && (raw.flags & CANFD_BRS);
  f.extended = raw.can_id & CAN_EFF_FLAG; f.remote = raw.can_id & CAN_RTR_FLAG; f.error = raw.can_id & CAN_ERR_FLAG;
  std::copy_n(raw.data, raw.len, f.data.begin());
  return f;
}
}  // namespace vehicle_core
