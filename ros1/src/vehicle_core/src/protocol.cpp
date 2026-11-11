// SPDX-License-Identifier: Apache-2.0
#include "vehicle_core/protocol.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace vehicle_core {
namespace {
Frame make(Address address, Command command, uint8_t size) {
  Frame frame;
  frame.id = address.base() | static_cast<uint8_t>(command);
  frame.size = size;
  return frame;
}
int16_t read_i16(const Frame& f, size_t pos) {
  const auto u = uint32_t{f.data[pos]} | (uint32_t{f.data[pos + 1]} << 8U);
  return static_cast<int16_t>(u >= 0x8000U ? static_cast<int32_t>(u) - 0x10000
                                           : static_cast<int32_t>(u));
}
int32_t read_i32(const Frame& f, size_t pos) {
  const auto u = uint32_t{f.data[pos]} | (uint32_t{f.data[pos + 1]} << 8U) |
                 (uint32_t{f.data[pos + 2]} << 16U) | (uint32_t{f.data[pos + 3]} << 24U);
  return static_cast<int32_t>(u >= 0x80000000U ? int64_t{u} - 0x100000000LL : u);
}
int8_t read_i8(uint8_t u) {
  return static_cast<int8_t>(u >= 0x80 ? static_cast<int>(u) - 0x100 : u);
}
void write_scaled(Frame& f, size_t pos, double value) {
  if (!std::isfinite(value) || value < -32.768 || value > 32.767) {
    throw std::invalid_argument("motion is non-finite or outside int16 milliscale range");
  }
  const auto u = static_cast<uint16_t>(static_cast<int16_t>(std::lround(value * 1000.0)));
  f.data[pos] = static_cast<uint8_t>(u & 0xFFU);
  f.data[pos + 1] = static_cast<uint8_t>(u >> 8U);
}
std::optional<std::string> ascii(const Frame& f, size_t start) {
  std::string value;
  for (size_t i = start; i < start + 4; ++i) {
    if (f.data[i] == 0) break;
    if (f.data[i] < 0x20 || f.data[i] > 0x7E) return std::nullopt;
    value.push_back(static_cast<char>(f.data[i]));
  }
  return value;
}
template <class T>
Decoded ok(T value) {
  return {DecodeStatus::Ok, Event{value}};
}
Decoded bad() { return {DecodeStatus::Malformed, std::nullopt}; }
}  // namespace
Frame reset(Address a) {
  auto f = make(a, Command::Reset, 3);
  f.data = {1, a.type, a.node};
  return f;
}
Frame query_version(Address a) { return make(a, Command::Version, 0); }
Frame enable(Address a, bool enabled) {
  auto f = make(a, Command::Enable, 5);
  f.data = {1, a.type, a.node, static_cast<uint8_t>(enabled), 1};
  return f;
}
Frame clear_faults(Address a) {
  auto f = make(a, Command::ClearFaults, 1);
  f.data[0] = 0xCC;
  return f;
}
Frame zero_motion(Address a) {
  auto f = make(a, Command::ZeroMotion, 1);
  f.data[0] = 0xCC;
  return f;
}
Frame mode(Address a, uint8_t value, bool beep, bool brake, bool special) {
  if (value > 3) throw std::invalid_argument("mode must be 0..3");
  auto f = make(a, Command::Mode, 4);
  f.data = {value, static_cast<uint8_t>(beep), static_cast<uint8_t>(brake),
            static_cast<uint8_t>(special)};
  return f;
}
Frame motion(Address a, double vx, double vy, double yaw_rate) {
  auto f = make(a, Command::Motion, 6);
  write_scaled(f, 0, vx);
  write_scaled(f, 2, vy);
  write_scaled(f, 4, yaw_rate);
  return f;
}
Frame feedback(Address a, Command command, bool enabled) {
  if (command < Command::RemoteFeedback || command > Command::DriverStatusFeedback) {
    throw std::invalid_argument("not a feedback switch command");
  }
  auto f = make(a, command, 1);
  f.data[0] = static_cast<uint8_t>(enabled);
  return f;
}
Decoded decode(const Frame& f, Address a) {
  if (!f.extended || f.remote || f.error || (f.id & 0xFFFFFF00U) != a.base()) return {};
  if (f.size > 8) return bad();
  switch (f.id & 0xFFU) {
    case 0xA2: {
      if (f.size != 8) return bad();
      auto hardware = ascii(f, 0);
      auto software = ascii(f, 4);
      if (!hardware || !software) return bad();
      return ok(Version{*hardware, *software});
    }
    case 0xA3:
    case 0xA4:
    case 0xA5:
      if (f.size != 0) return bad();
      return ok(Ack{static_cast<Command>((f.id & 0xFFU) - 0xA0U)});
    case 0xB0:
      if (f.size != 1 || f.data[0] > 1) return bad();
      return ok(Heartbeat{f.data[0] != 0});
    case 0xB1:
      if (f.size != 6 && f.size != 8) return bad();
      // No assumptions about D5; D6/D7 are deliberately never accessed.
      return ok(Status{f.data[0], f.data[1], read_i16(f, 2) / 10.0, f.data[4] != 0, f.data[5]});
    case 0xB2:
      if (f.size != 6) return bad();
      return ok(Motion{read_i16(f, 0) / 1000.0, read_i16(f, 2) / 1000.0, read_i16(f, 4) / 1000.0});
    case 0xB3:
    case 0xB4:
      if (f.size != 8) return bad();
      return ok(Mileage{read_i32(f, 0), read_i32(f, 4), (f.id & 0xFFU) == 0xB4});
    case 0xB5: {
      if (f.size != 7) return bad();
      Remote r{};
      for (size_t i = 0; i < 4; ++i) r.switches[i] = (f.data[0] >> (i * 2U)) & 3U;
      for (size_t i = 0; i < 6; ++i) r.axes[i] = read_i8(f.data[i + 1]);
      return ok(r);
    }
    case 0xB6: {
      if (f.size != 7) return bad();
      Sensors s{};
      s.bumper_mask = f.data[0];
      for (size_t i = 0; i < 6; ++i) s.ultrasonic_raw[i] = f.data[i + 1];
      return ok(s);
    }
    case 0xB7:
      if (f.size != 7 || f.data[0] > 3) return bad();
      return ok(DriverMotion{f.data[0], read_i16(f, 1), read_i32(f, 3)});
    case 0xB8:
      if (f.size != 8 || f.data[0] > 3) return bad();
      return ok(DriverStatus{f.data[0], read_i16(f, 1) / 10.0, read_i16(f, 3) / 10.0,
                             read_i8(f.data[5]), read_i8(f.data[6]), f.data[7]});
    case 0xBA:
      if (f.size < 5) return bad();
      return ok(Faults{f.data[0], f.data[1], f.data[2], f.data[3], f.data[4]});
    default:
      return {};
  }
}
}  // namespace vehicle_core
