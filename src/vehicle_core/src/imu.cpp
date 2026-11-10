// SPDX-License-Identifier: Apache-2.0
#include "vehicle_core/imu.hpp"
#include <cmath>

namespace vehicle_core {
namespace {
constexpr double pi = 3.14159265358979323846;
int16_t i16(const ImuFrame& f, size_t offset) {
  const auto v = uint32_t{f.data[offset]} | (uint32_t{f.data[offset + 1]} << 8U);
  return static_cast<int16_t>(v < 0x8000U ? static_cast<int32_t>(v) : static_cast<int32_t>(v) - 65536);
}
uint32_t u32(const ImuFrame& f, size_t offset) {
  return uint32_t{f.data[offset]} | (uint32_t{f.data[offset + 1]} << 8U) |
         (uint32_t{f.data[offset + 2]} << 16U) | (uint32_t{f.data[offset + 3]} << 24U);
}
bool mapping(const ImuFrame& f, uint32_t id) {
  return f.id == id && f.fd && !f.extended && !f.remote && !f.error && f.size == 32 &&
         f.data[29] == 0 && f.data[30] == 0 && f.data[31] == 0;
}
Vector3 vector(const ImuFrame& f, size_t offset, double scale) {
  return {i16(f, offset) * scale, i16(f, offset + 2) * scale, i16(f, offset + 4) * scale};
}
}  // namespace
std::optional<ImuMeasurement> decode_imu_measurement(const ImuFrame& f, uint8_t node) {
  if (!mapping(f, 0x180U + node)) return std::nullopt;
  return ImuMeasurement{vector(f, 0, 9.80665 / 1000), vector(f, 6, pi / 1800),
                        vector(f, 12, 1e-7), i16(f, 18) / 100.0, u32(f, 20), u32(f, 25), f.data[24]};
}
std::optional<ImuOrientation> decode_imu_orientation(const ImuFrame& f, uint8_t node) {
  if (!mapping(f, 0x280U + node)) return std::nullopt;
  ImuOrientation value;
  double norm = 0;
  for (size_t i = 0; i < 4; ++i) {
    value.quaternion[i] = i16(f, i * 2) / 10000.0;
    norm += value.quaternion[i] * value.quaternion[i];
  }
  norm = std::sqrt(norm);
  value.gravity = vector(f, 8, 9.80665 / 1000);
  value.gyro_bias = vector(f, 14, pi / 1800);
  value.counter = u32(f, 20);
  value.age_ms = u32(f, 24);
  value.status = f.data[28];
  value.valid = (value.status & 1U) != 0 && value.age_ms < 250 && norm > 0.98 && norm < 1.02;
  if (value.valid) for (auto& v : value.quaternion) v /= norm;
  return value;
}
Quaternion multiply(const Quaternion& a, const Quaternion& b) {
  const auto [w, x, y, z] = a;
  const auto [v, i, j, k] = b;
  return {w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j, w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v};
}
Quaternion conjugate(const Quaternion& q) { return {q[0], -q[1], -q[2], -q[3]}; }
Quaternion mounting_rotation(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll/2), sr = std::sin(roll/2);
  const double cp = std::cos(pitch/2), sp = std::sin(pitch/2);
  const double cy = std::cos(yaw/2), sy = std::sin(yaw/2);
  return {cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy};
}
Vector3 rotate(const Quaternion& q, const Vector3& v) {
  const auto result = multiply(multiply(q, {0, v[0], v[1], v[2]}), conjugate(q));
  return {result[1], result[2], result[3]};
}
double quaternion_yaw(const Quaternion& q) {
  return std::atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]));
}
bool ImuCounter::update(uint32_t counter, double time) {
  restarted_ = false;
  if (!std::isfinite(time) || (counter_ && *counter_ == counter)) return false;
  restarted_ = counter_ && static_cast<uint32_t>(counter - *counter_) > 0x80000000U;
  counter_ = counter;
  time_ = time;
  return true;
}
bool ImuCounter::fresh(double time, double timeout) const {
  return counter_ && time >= time_ && time - time_ < timeout;
}
void ImuCounter::reset() { counter_.reset(); restarted_ = false; }
}  // namespace vehicle_core
