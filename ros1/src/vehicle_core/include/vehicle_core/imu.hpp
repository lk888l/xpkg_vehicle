// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace vehicle_core {
// Kept separate from the chassis' classic, extended-ID protocol.
struct ImuFrame {
  uint32_t id{};
  uint8_t size{};
  std::array<uint8_t, 64> data{};
  bool fd{}, brs{}, extended{}, remote{}, error{};
};
class ImuSocketCan {
 public:
  ImuSocketCan(const std::string& interface, uint8_t node);
  ~ImuSocketCan();
  ImuSocketCan(const ImuSocketCan&) = delete;
  ImuSocketCan& operator=(const ImuSocketCan&) = delete;
  std::optional<ImuFrame> receive();
  void start(uint8_t node);  // Addressed Classic CAN NMT Start, never broadcast/reset.
  uint32_t dropped_frames() const { return dropped_; }
 private:
  int fd_{-1};
  uint32_t dropped_{};
};
using Vector3 = std::array<double, 3>;
using Quaternion = std::array<double, 4>;  // w, x, y, z
Quaternion multiply(const Quaternion& a, const Quaternion& b);
Quaternion conjugate(const Quaternion& q);
Quaternion mounting_rotation(double roll, double pitch, double yaw);
Vector3 rotate(const Quaternion& q, const Vector3& v);
double quaternion_yaw(const Quaternion& q);
struct ImuMeasurement {
  Vector3 acceleration{}, angular_velocity{}, magnetic_field{};  // SI
  double temperature{};  // Celsius
  uint32_t counter{}, faults{};
  uint8_t status{};
};
struct ImuOrientation {
  Quaternion quaternion{};
  Vector3 gravity{}, gyro_bias{};
  uint32_t counter{}, age_ms{};
  uint8_t status{};
  bool valid{};
};
std::optional<ImuMeasurement> decode_imu_measurement(const ImuFrame& frame, uint8_t node);
std::optional<ImuOrientation> decode_imu_orientation(const ImuFrame& frame, uint8_t node);
// Updated firmware: four standard-ID, little-endian int16 Classic CAN PDOs.
// No device counters, validity bits, fault word or device timestamps are supplied.
struct ImuClassicPdo {
  uint8_t pdo{};
  Vector3 vector{};
  Quaternion quaternion{};
  double temperature{};
  bool orientation_valid{};
};
std::optional<ImuClassicPdo> decode_imu_classic(const ImuFrame& frame, uint8_t node);
// A repeated PDO counter must not refresh data. A reset counter starts a new epoch.
class ImuCounter {
 public:
  bool update(uint32_t counter, double time);
  bool fresh(double time, double timeout) const;
  void reset();
  bool restarted() const { return restarted_; }
 private:
  std::optional<uint32_t> counter_;
  double time_{};
  bool restarted_{};
};
}  // namespace vehicle_core
