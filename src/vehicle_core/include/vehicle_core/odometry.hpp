// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <optional>

#include "vehicle_core/protocol.hpp"

namespace vehicle_core {
struct Pose {
  double x{}, y{}, yaw{}, vx{}, yaw_rate{};
};
class Odometry {
 public:
  // track_width is an effective rolling width, not a guessed chassis dimension.
  explicit Odometry(double track_width = 0, double max_gap = 0.25, double max_wheel_speed = 10.0);
  bool update(const Motion& motion, double monotonic_time);
  bool update(const Mileage& mileage, double monotonic_time);
  void rebase();  // Preserve pose, discard counter/time baseline after reconnect or A5.
  void reset();
  const Pose& pose() const noexcept { return pose_; }
  uint64_t discontinuities() const noexcept { return discontinuities_; }

 private:
  void integrate(double distance, double angle, double dt);
  double track_width_, max_gap_, max_wheel_speed_;
  std::optional<double> last_time_;
  std::optional<Mileage> last_mileage_;
  Pose pose_{};
  uint64_t discontinuities_{};
};
}  // namespace vehicle_core
