// SPDX-License-Identifier: Apache-2.0
#pragma once

namespace vehicle_driver {
// A value-based Strategy selected once at startup. Odometry and motion control
// remain sensor-agnostic; this policy owns their feedback prerequisites.
class HeadingPolicy final {
 public:
  explicit constexpr HeadingPolicy(bool use_imu) : use_imu_(use_imu) {}
  constexpr bool uses_imu() const { return use_imu_; }
  constexpr bool ready(bool odometry_fresh, bool imu_fresh) const {
    return odometry_fresh && (!use_imu_ || imu_fresh);
  }
  const char* name() const { return use_imu_ ? "imu" : "chassis"; }
  const char* requirements() const {
    return use_imu_ ? "需要已使能的 CAN 模式、新鲜里程计与已就绪的 IMU 航向；标定期间不能执行" :
                      "需要已使能的 CAN 模式和新鲜底盘里程计";
  }

 private:
  const bool use_imu_;
};
}  // namespace vehicle_driver
