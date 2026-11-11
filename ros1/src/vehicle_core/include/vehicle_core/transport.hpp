// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <string>

#include "vehicle_core/protocol.hpp"

namespace vehicle_core {
// Dependency boundary for a real socket, replay source, or test double.
class CanTransport {
 public:
  virtual ~CanTransport() = default;
  virtual void send(const Frame& frame) = 0;
  virtual std::optional<Frame> receive() = 0;  // Nonblocking.
  virtual uint32_t dropped_frames() const noexcept = 0;
};
class SocketCan final : public CanTransport {
 public:
  explicit SocketCan(const std::string& interface, Address address = {});
  ~SocketCan() override;
  SocketCan(const SocketCan&) = delete;
  SocketCan& operator=(const SocketCan&) = delete;
  void send(const Frame& frame) override;
  std::optional<Frame> receive() override;
  uint32_t dropped_frames() const noexcept override { return dropped_; }

 private:
  int fd_{-1};
  uint32_t dropped_{};
};
}  // namespace vehicle_core
