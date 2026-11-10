// SPDX-License-Identifier: Apache-2.0
#include "vehicle_core/odometry.hpp"

#include <cmath>
#include <stdexcept>

namespace vehicle_core {
namespace {
double distance(int32_t current, int32_t previous) {
  const uint32_t delta = static_cast<uint32_t>(current) - static_cast<uint32_t>(previous);
  const int64_t signed_delta = delta >= 0x80000000U ? int64_t{delta} - 0x100000000LL : delta;
  return static_cast<double>(signed_delta) / 1000.0;
}
}  // namespace
Odometry::Odometry(double width, double gap, double max_speed)
    : track_width_(width), max_gap_(gap), max_wheel_speed_(max_speed) {
  if (!std::isfinite(width) || width < 0 || !std::isfinite(gap) || gap <= 0 ||
      !std::isfinite(max_speed) || max_speed <= 0)
    throw std::invalid_argument("invalid odometry configuration");
}
void Odometry::set_heading(double yaw, double time) {
  if (!std::isfinite(yaw) || !std::isfinite(time)) { invalidate_heading(); return; }
  if (!heading_fresh(time)) integrated_imu_yaw_.reset();
  imu_yaw_ = yaw;
  imu_time_ = time;
}
void Odometry::invalidate_heading() {
  imu_yaw_.reset(); imu_time_.reset(); integrated_imu_yaw_.reset();
}
double Odometry::heading_age(double time) const { return imu_time_ ? time - *imu_time_ : -1; }
bool Odometry::heading_fresh(double time) const {
  return imu_time_ && time >= *imu_time_ && time - *imu_time_ < max_gap_;
}
void Odometry::integrate(double ds, double da, double dt, double time) {
  if (heading_fresh(time)) {
    if (integrated_imu_yaw_) da = std::remainder(*imu_yaw_ - *integrated_imu_yaw_, 2.0 * std::acos(-1.0));
    integrated_imu_yaw_ = imu_yaw_;
  } else {
    integrated_imu_yaw_.reset();
  }
  const double half = da / 2.0;
  const double sinc = std::abs(half) < 1e-9 ? 1.0 : std::sin(half) / half;
  pose_.x += ds * sinc * std::cos(pose_.yaw + half);
  pose_.y += ds * sinc * std::sin(pose_.yaw + half);
  pose_.yaw = std::remainder(pose_.yaw + da, 2.0 * std::acos(-1.0));
  pose_.vx = ds / dt;
  pose_.yaw_rate = da / dt;
}
bool Odometry::update(const Motion& m, double time) {
  if (!std::isfinite(time) || !std::isfinite(m.vx) || !std::isfinite(m.yaw_rate)) return false;
  const auto previous = last_time_;
  last_time_ = time;
  if (!previous) {
    integrated_imu_yaw_ = heading_fresh(time) ? imu_yaw_ : std::nullopt;
    return false;
  }
  const double dt = time - *previous;
  if (dt <= 0 || dt > max_gap_) {
    integrated_imu_yaw_ = heading_fresh(time) ? imu_yaw_ : std::nullopt;
    ++discontinuities_;
    pose_.vx = pose_.yaw_rate = 0;
    return false;
  }
  integrate(m.vx * dt, m.yaw_rate * dt, dt, time);
  return true;
}
bool Odometry::update(const Mileage& m, double time) {
  if (m.secondary || !std::isfinite(time)) return false;
  if (track_width_ <= 0)
    throw std::logic_error("wheel odometry requires calibrated effective_track_width");
  const auto previous_time = last_time_;
  const auto previous = last_mileage_;
  last_time_ = time;
  last_mileage_ = m;
  if (!previous_time || !previous) {
    integrated_imu_yaw_ = heading_fresh(time) ? imu_yaw_ : std::nullopt;
    return false;
  }
  const double dt = time - *previous_time;
  const double dl = distance(m.left_mm, previous->left_mm);
  const double dr = distance(m.right_mm, previous->right_mm);
  if (dt <= 0 || dt > max_gap_ || std::abs(dl) > max_wheel_speed_ * dt + 0.01 ||
      std::abs(dr) > max_wheel_speed_ * dt + 0.01) {
    integrated_imu_yaw_ = heading_fresh(time) ? imu_yaw_ : std::nullopt;
    ++discontinuities_;
    pose_.vx = pose_.yaw_rate = 0;
    return false;
  }
  integrate((dl + dr) / 2.0, (dr - dl) / track_width_, dt, time);
  return true;
}
void Odometry::rebase() {
  last_time_.reset();
  last_mileage_.reset();
  integrated_imu_yaw_.reset();
  pose_.vx = pose_.yaw_rate = 0;
}
void Odometry::reset() {
  rebase();
  pose_ = {};
}
}  // namespace vehicle_core
