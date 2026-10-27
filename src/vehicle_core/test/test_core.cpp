// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <vehicle_core/control.hpp>
#include <vehicle_core/odometry.hpp>
#include <vehicle_core/protocol.hpp>
#include <vehicle_core/transport.hpp>
using namespace vehicle_core;
namespace {
Frame frame(uint8_t function, std::initializer_list<uint8_t> payload) {
  Frame f;
  f.id = Address{}.base() | function;
  f.size = static_cast<uint8_t>(payload.size());
  std::copy(payload.begin(), payload.end(), f.data.begin());
  return f;
}
template <class T>
T event(const Frame& f) {
  auto result = decode(f);
  EXPECT_EQ(result.status, DecodeStatus::Ok);
  return std::get<T>(result.event.value());
}
}  // namespace
TEST(Protocol, CommandGoldenFrames) {
  EXPECT_EQ(query_version({}).id, 0x01E10102U);
  EXPECT_EQ(query_version({}).size, 0);
  auto f = enable({}, true);
  EXPECT_EQ(f.size, 5);
  EXPECT_EQ(f.data, (std::array<uint8_t, 8>{1, 0xE1, 1, 1, 1, 0, 0, 0}));
  EXPECT_EQ(reset({}).size, 3);
  EXPECT_EQ(reset({}).data[1], 0xE1);
  EXPECT_EQ(clear_faults({}).data[0], 0xCC);
  EXPECT_EQ(zero_motion({}).id, 0x01E10105U);
  EXPECT_EQ(mode({}, 2, true, false, true).data, (std::array<uint8_t, 8>{2, 1, 0, 1, 0, 0, 0, 0}));
  EXPECT_EQ(feedback({}, Command::DriverStatusFeedback, true).id, 0x01E10118U);
  EXPECT_THROW(mode({}, 4, false, false, false), std::invalid_argument);
  EXPECT_THROW(feedback({}, Command::Reset, true), std::invalid_argument);
}
TEST(Protocol, SignedLittleEndianMotionAndBounds) {
  auto f = motion({}, -0.1, 0.05, -0.2);
  EXPECT_EQ(f.data, (std::array<uint8_t, 8>{0x9C, 0xFF, 0x32, 0, 0x38, 0xFF, 0, 0}));
  f.id = Address{}.base() | 0xB2;
  auto m = event<Motion>(f);
  EXPECT_DOUBLE_EQ(m.vx, -0.1);
  EXPECT_DOUBLE_EQ(m.yaw_rate, -0.2);
  f = motion({}, -32.768, 32.767, 0);
  f.id = Address{}.base() | 0xB2;
  m = event<Motion>(f);
  EXPECT_DOUBLE_EQ(m.vx, -32.768);
  EXPECT_DOUBLE_EQ(m.vy, 32.767);
  EXPECT_THROW(motion({}, 32.768, 0, 0), std::invalid_argument);
  EXPECT_THROW(motion({}, std::numeric_limits<double>::quiet_NaN(), 0, 0), std::invalid_argument);
  EXPECT_THROW(motion({}, 0, 0, std::numeric_limits<double>::infinity()), std::invalid_argument);
}
TEST(Protocol, AsciiVersion) {
  auto v = event<Version>(frame(0xA2, {'3', '.', '0', 0, '4', '.', '0', 0}));
  EXPECT_EQ(v.hardware, "3.0");
  EXPECT_EQ(v.software, "4.0");
  v = event<Version>(frame(0xA2, {'3', '.', '1', '2', '4', '.', '1', '2'}));
  EXPECT_EQ(v.hardware, "3.12");
  EXPECT_EQ(decode(frame(0xA2, {1, 2, 3, 4, 5, 6, 7, 8})).status, DecodeStatus::Malformed);
}
TEST(Protocol, SixByteStatusNeverReadsTail) {
  auto f = frame(0xB1, {0, 2, 0x1A, 1, 0, 0x20});
  f.data[6] = f.data[7] = 0xFF;
  auto s = event<Status>(f);
  EXPECT_EQ(s.mode, 2);
  EXPECT_DOUBLE_EQ(s.voltage, 28.2);
  EXPECT_EQ(s.flags_raw, 0x20);
  EXPECT_FALSE(s.beep);
  f.size = 8;
  EXPECT_EQ(event<Status>(f).flags_raw, 0x20);
}
TEST(Protocol, ZeroLengthAcknowledgements) {
  for (uint8_t c = 3; c <= 5; ++c) {
    auto f = frame(static_cast<uint8_t>(0xA0 + c), {});
    EXPECT_EQ(static_cast<uint8_t>(event<Ack>(f).command), c);
    f.size = 1;
    EXPECT_EQ(decode(f).status, DecodeStatus::Malformed);
  }
}
TEST(Protocol, IdentityAndFrameTypeIsolation) {
  auto f = frame(0xB0, {1});
  for (const uint32_t id : {0x05E101B0U, 0x01E102B0U, 0x01E201B0U, 0x11E101B0U}) {
    f.id = id;
    EXPECT_EQ(decode(f).status, DecodeStatus::Ignored);
  }
  f = frame(0xB0, {1});
  f.extended = false;
  EXPECT_EQ(decode(f).status, DecodeStatus::Ignored);
  f.extended = true;
  f.remote = true;
  EXPECT_EQ(decode(f).status, DecodeStatus::Ignored);
  f.remote = false;
  f.error = true;
  EXPECT_EQ(decode(f).status, DecodeStatus::Ignored);
}
TEST(Protocol, RejectsEveryTruncatedKnownPayload) {
  for (const auto [code, size] : {std::pair{0xA2, 8},
                                  {0xB0, 1},
                                  {0xB1, 6},
                                  {0xB2, 6},
                                  {0xB3, 8},
                                  {0xB4, 8},
                                  {0xB5, 7},
                                  {0xB6, 7},
                                  {0xB7, 7},
                                  {0xB8, 8},
                                  {0xBA, 5}}) {
    for (int length = 0; length < size; ++length) {
      Frame f;
      f.id = Address{}.base() | static_cast<uint8_t>(code);
      f.size = static_cast<uint8_t>(length);
      EXPECT_EQ(decode(f).status, DecodeStatus::Malformed) << code << ":" << length;
    }
  }
}
TEST(Protocol, DriverChannelsAndSignedValues) {
  for (uint8_t c = 0; c < 4; ++c) {
    auto d = event<DriverMotion>(frame(0xB7, {c, 0xFF, 0xFF, 0, 0, 0, 0x80}));
    EXPECT_EQ(d.channel, c);
    EXPECT_EQ(d.rpm, -1);
    EXPECT_EQ(d.pulses, std::numeric_limits<int32_t>::min());
  }
  EXPECT_EQ(decode(frame(0xB7, {4, 0, 0, 0, 0, 0, 0})).status, DecodeStatus::Malformed);
  EXPECT_EQ(decode(frame(0xB8, {255, 0, 0, 0, 0, 0, 0, 0})).status, DecodeStatus::Malformed);
  auto d = event<DriverStatus>(frame(0xB8, {3, 0x1A, 1, 0xF6, 0xFF, 0xFE, 30, 0xA1}));
  EXPECT_DOUBLE_EQ(d.voltage, 28.2);
  EXPECT_DOUBLE_EQ(d.current, -1);
  EXPECT_EQ(d.driver_temperature, -2);
  EXPECT_EQ(d.faults_raw, 0xA1);
}
TEST(Protocol, SignedRemoteAxesAndRawLegacyFields) {
  auto r = event<Remote>(frame(0xB5, {0xE4, 0x9C, 100, 0xFF, 0, 0x80, 0x7F}));
  EXPECT_EQ(r.switches, (std::array<uint8_t, 4>{0, 1, 2, 3}));
  EXPECT_EQ(r.axes, (std::array<int8_t, 6>{-100, 100, -1, 0, -128, 127}));
  EXPECT_EQ(event<Sensors>(frame(0xB6, {0x80, 1, 2, 3, 4, 5, 6})).ultrasonic_raw[5], 6);
  EXPECT_EQ(event<Faults>(frame(0xBA, {1, 2, 3, 4, 5})).power, 5);
}
TEST(Protocol, ArbitraryFrameFuzzDoesNotReadOutsidePayload) {
  std::mt19937 random(0xE101);
  for (int i = 0; i < 50000; ++i) {
    Frame f;
    f.id = Address{}.base() | (random() & 0xFFU);
    f.size = random() % 256;
    for (auto& b : f.data) b = random() & 0xFFU;
    auto result = decode(f);
    EXPECT_EQ(result.event.has_value(), result.status == DecodeStatus::Ok);
  }
}
TEST(Control, LatestWinsClampsAndStopsOnce) {
  CommandBuffer buffer(0.3, 1.0, 2.0);
  EXPECT_FALSE(buffer.sample(1.0));
  ASSERT_TRUE(buffer.set(100, -100, 1.0));
  auto m = buffer.sample(1.1);
  ASSERT_TRUE(m);
  EXPECT_DOUBLE_EQ(m->vx, 1);
  EXPECT_DOUBLE_EQ(m->yaw_rate, -2);
  ASSERT_TRUE(buffer.set(-0.1, 0.2, 1.2));
  EXPECT_DOUBLE_EQ(buffer.sample(1.3)->vx, -0.1);
  EXPECT_DOUBLE_EQ(buffer.sample(1.51)->vx, 0);
  EXPECT_FALSE(buffer.sample(1.6));
}
TEST(Control, InvalidInputAndClockReversalStopInsteadOfReplaying) {
  CommandBuffer buffer(0.3, 1, 2);
  buffer.set(1, 1, 10);
  EXPECT_FALSE(buffer.set(NAN, 1, 10.1));
  EXPECT_DOUBLE_EQ(buffer.sample(10.1)->vx, 0);
  buffer.set(1, 1, 12);
  EXPECT_DOUBLE_EQ(buffer.sample(11)->vx, 0);
  EXPECT_FALSE(buffer.sample(12));
  EXPECT_THROW(CommandBuffer(0, 1, 2), std::invalid_argument);
}
TEST(Odometry, ExactCircularArcAndSmallMotionNotDeadbanded) {
  Odometry odom;
  odom.update(Motion{1, 0, 1}, 0);
  for (int i = 1; i <= 10; ++i) ASSERT_TRUE(odom.update(Motion{1, 0, 1}, i * 0.1));
  EXPECT_NEAR(odom.pose().x, std::sin(1.0), 1e-12);
  EXPECT_NEAR(odom.pose().y, 1 - std::cos(1.0), 1e-12);
  EXPECT_NEAR(odom.pose().yaw, 1.0, 1e-12);
  odom.reset();
  odom.update(Motion{0.001, 0, 0}, 0);
  odom.update(Motion{0.001, 0, 0}, 0.02);
  EXPECT_NEAR(odom.pose().x, 0.00002, 1e-12);
}
TEST(Odometry, MissingFeedbackDoesNotExtrapolate) {
  Odometry odom;
  odom.update(Motion{1, 0, 0}, 0);
  odom.update(Motion{1, 0, 0}, 0.1);
  EXPECT_FALSE(odom.update(Motion{1, 0, 0}, 2.0));
  EXPECT_DOUBLE_EQ(odom.pose().x, 0.1);
  odom.rebase();
  EXPECT_FALSE(odom.update(Motion{1, 0, 0}, 2.1));
  EXPECT_DOUBLE_EQ(odom.pose().x, 0.1);
}
TEST(Odometry, MileageRolloverAndExplicitRebase) {
  Odometry odom(0.5);
  odom.update(Mileage{2147483640, 2147483640, false}, 0);
  ASSERT_TRUE(odom.update(Mileage{-2147483646, -2147483646, false}, 0.02));
  EXPECT_NEAR(odom.pose().x, 0.01, 1e-12);
  odom.rebase();
  EXPECT_FALSE(odom.update(Mileage{0, 0, false}, 0.04));
  EXPECT_NEAR(odom.pose().x, 0.01, 1e-12);
}
TEST(Odometry, DifferentialTurnAndDiscontinuity) {
  Odometry odom(0.5);
  odom.update(Mileage{0, 0, false}, 0);
  ASSERT_TRUE(odom.update(Mileage{-25, 25, false}, 0.1));
  EXPECT_NEAR(odom.pose().yaw, 0.1, 1e-12);
  EXPECT_DOUBLE_EQ(odom.pose().x, 0);
  EXPECT_FALSE(odom.update(Mileage{2000000, 2000000, false}, 0.12));
  EXPECT_EQ(odom.discontinuities(), 1U);
  EXPECT_FALSE(odom.update(Mileage{0, 0, true}, 0.14));
  Odometry uncalibrated;
  EXPECT_THROW(uncalibrated.update(Mileage{}, 0), std::logic_error);
}
TEST(Transport, MissingInterfaceIsActionable) {
  EXPECT_THROW(SocketCan("absent_can_42"), std::system_error);
}
