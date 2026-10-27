// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>

namespace vehicle_core {
struct Frame {
  uint32_t id{};  // Arbitration ID, without Linux flag bits.
  uint8_t size{};
  std::array<uint8_t, 8> data{};
  bool extended{true};
  bool remote{};
  bool error{};
};
struct Address {
  uint8_t type{0xE1};
  uint8_t node{0x01};
  uint32_t base() const noexcept {
    return 0x01000000U | (uint32_t{type} << 16U) | (uint32_t{node} << 8U);
  }
};
enum class Command : uint8_t {
  Reset = 0x01,
  Version = 0x02,
  Enable = 0x03,
  ClearFaults = 0x04,
  ZeroMotion = 0x05,
  Mode = 0x11,
  Motion = 0x12,
  RemoteFeedback = 0x15,
  SensorFeedback = 0x16,
  DriverMotionFeedback = 0x17,
  DriverStatusFeedback = 0x18
};
Frame reset(Address address);
Frame query_version(Address address);
Frame enable(Address address, bool enabled);
Frame clear_faults(Address address);
Frame zero_motion(Address address);
Frame mode(Address address, uint8_t mode, bool beep, bool brake, bool special);
Frame motion(Address address, double vx, double vy, double yaw_rate);
Frame feedback(Address address, Command command, bool enabled);

struct Version {
  std::string hardware;
  std::string software;
};
struct Ack {
  Command command;
};
struct Heartbeat {
  bool enabled;
};
struct Status {
  uint8_t work_state;
  uint8_t mode;
  double voltage;
  bool beep;
  uint8_t flags_raw;
};
struct Motion {
  double vx;
  double vy;
  double yaw_rate;
};
struct Mileage {
  int32_t left_mm;
  int32_t right_mm;
  bool secondary;
};
struct Remote {
  std::array<uint8_t, 4> switches;
  std::array<int8_t, 6> axes;
};
struct Sensors {
  uint8_t bumper_mask;
  std::array<uint8_t, 6> ultrasonic_raw;
};
struct DriverMotion {
  uint8_t channel;
  int16_t rpm;
  int32_t pulses;
};
struct DriverStatus {
  uint8_t channel;
  double voltage;
  double current;
  int8_t driver_temperature;
  int8_t motor_temperature;
  uint8_t faults_raw;
};
// BA and B6 layouts are inherited protocol hypotheses, not physically verified.
struct Faults {
  uint8_t motor;
  uint8_t driver;
  uint8_t driver_offline;
  uint8_t status_raw;
  uint8_t power;
};
using Event = std::variant<Version, Ack, Heartbeat, Status, Motion, Mileage, Remote, Sensors,
                           DriverMotion, DriverStatus, Faults>;
enum class DecodeStatus { Ok, Ignored, Malformed };
struct Decoded {
  DecodeStatus status{DecodeStatus::Ignored};
  std::optional<Event> event;
};
Decoded decode(const Frame& frame, Address address = {});
}  // namespace vehicle_core
