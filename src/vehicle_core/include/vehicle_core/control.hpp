// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

#include "vehicle_core/protocol.hpp"

namespace vehicle_core {
// All times are monotonic seconds. At timeout, issue ONE stop then become idle.
class CommandBuffer {
 public:
  CommandBuffer(double timeout, double max_linear, double max_angular)
      : timeout_(timeout), max_linear_(max_linear), max_angular_(max_angular) {
    if (!std::isfinite(timeout) || timeout <= 0 || !std::isfinite(max_linear) || max_linear <= 0 ||
        max_linear > 32.767 || !std::isfinite(max_angular) || max_angular <= 0 ||
        max_angular > 32.767)
      throw std::invalid_argument("invalid command limits");
  }
  bool set(double vx, double yaw_rate, double now) {
    if (!std::isfinite(vx) || !std::isfinite(yaw_rate)) {
      stop();
      return false;
    }
    command_ = {std::clamp(vx, -max_linear_, max_linear_), 0,
                std::clamp(yaw_rate, -max_angular_, max_angular_)};
    received_ = now;
    active_ = true;
    stop_pending_ = false;
    return true;
  }
  void stop() {
    active_ = false;
    stop_pending_ = true;
  }
  std::optional<Motion> sample(double now) {
    if (active_ && now - received_ < timeout_ && now >= received_) return command_;
    if (active_ || stop_pending_) {
      active_ = false;
      stop_pending_ = false;
      return Motion{};
    }
    return std::nullopt;
  }
  bool active(double now) const {
    return active_ && now >= received_ && now - received_ < timeout_;
  }

 private:
  double timeout_, max_linear_, max_angular_, received_{};
  Motion command_{};
  bool active_{}, stop_pending_{};
};
}  // namespace vehicle_core
