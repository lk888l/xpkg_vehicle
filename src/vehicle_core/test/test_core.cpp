// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <vehicle_core/control.hpp>
#include <vehicle_core/motion_controller.hpp>
#include <vehicle_core/odometry.hpp>
#include <vehicle_core/protocol.hpp>
#include <vehicle_core/transport.hpp>
#include <vehicle_core/imu.hpp>
#include <vehicle_core/yaw_calibration.hpp>
using namespace vehicle_core;
TEST(MotionController, GeometryAndFeedbackAreBoundedAndDistinct) {
  MotionConfig config;
  config.max_angular_speed = 1;
  EXPECT_NEAR(config.feedforward_gain(), 1.308149183334789, 1e-9);
  MotionController c;
  c.start(MotionController::Mode::RelativeTurn, 0.1, config, {}, 0);
  const double initial = c.tick({}, .01, true)->yaw_rate;
  EXPECT_GT(initial, config.heading_gain * .1);
  Pose p; p.yaw_rate = .1;
  EXPECT_LT(c.tick(p, .02, true)->yaw_rate, initial);
  c.start(MotionController::Mode::RelativeTurn, 6, config, {}, 0);
  EXPECT_DOUBLE_EQ(c.tick({}, .01, true)->yaw_rate, 1);
  config.skid_compensation = 0;
  EXPECT_DOUBLE_EQ(config.feedforward_gain(), 1);
}

TEST(MotionController, RelativeTurnsKeepDirectionAcrossWrapAndMultipleRevolutions) {
  for (double target : {M_PI/2, -M_PI/2, 2*M_PI, -4*M_PI}) {
    MotionConfig config; config.max_angular_speed = .7;
    MotionController c;
    Pose p; p.yaw = 179 * M_PI / 180;
    double actual = p.yaw;
    c.start(MotionController::Mode::RelativeTurn, target, config, p, 0);
    for (int i = 1; i < 10000 && c.active(); ++i) {
      auto command = c.tick(p, i*.02, true);
      ASSERT_TRUE(command);
      ASSERT_EQ(command->vx, 0);
      ASSERT_LE(std::abs(command->yaw_rate), .7);
      // Plant has slip and a first-order rate response, independent of controller gain.
      p.yaw_rate += .2 * (command->yaw_rate / 1.4 - p.yaw_rate);
      actual += p.yaw_rate*.02;
      p.yaw = std::remainder(actual, 2*M_PI);
    }
    EXPECT_EQ(c.state(), MotionController::State::Succeeded);
    EXPECT_NEAR(actual - 179*M_PI/180, target, config.angle_tolerance);
    EXPECT_FALSE(c.tick(p, 201, true));
  }
}

TEST(MotionController, AbsoluteHeadingTakesShortestPathAndMustSettle) {
  MotionController c;
  Pose p; p.yaw = 170*M_PI/180;
  c.start(MotionController::Mode::Heading, -170*M_PI/180, {}, p, 0);
  EXPECT_NEAR(c.target(), 20*M_PI/180, 1e-9);
  EXPECT_GT(c.tick(p, .01, true)->yaw_rate, 0);
  p.yaw = -170*M_PI/180; p.yaw_rate = .2;
  EXPECT_EQ(c.tick(p, .1, true)->yaw_rate, 0);
  EXPECT_EQ(c.tick(p, .5, true)->yaw_rate, 0);
  EXPECT_TRUE(c.active());
  p.yaw_rate = 0;
  c.tick(p, .6, true); c.tick(p, .7, true);
  EXPECT_TRUE(c.active());
  c.tick(p, .81, true);
  EXPECT_EQ(c.state(), MotionController::State::Succeeded);
}

TEST(MotionController, DistanceUsesStartHeadingProjectionAndConvergesInBothDirections) {
  for (double target : {.5, -.5}) {
    MotionController c;
    Pose p; p.x = 2; p.y = 3; p.yaw = M_PI/3;
    c.start(MotionController::Mode::Distance, target, {}, p, 0);
    // Lateral displacement must not count as forward distance.
    p.x -= .1*std::sin(M_PI/3); p.y += .1*std::cos(M_PI/3);
    c.tick(p, .01, true);
    EXPECT_NEAR(c.progress(), 0, 1e-12);
    for (int i = 1; i < 2000 && c.active(); ++i) {
      auto command = c.tick(p, i*.02, true);
      ASSERT_TRUE(command);
      p.vx += .3 * (command->vx - p.vx);
      p.yaw_rate += .3 * (command->yaw_rate / 1.3 - p.yaw_rate);
      p.yaw += p.yaw_rate*.02;
      p.x += p.vx * std::cos(p.yaw)*.02; p.y += p.vx * std::sin(p.yaw)*.02;
    }
    EXPECT_EQ(c.state(), MotionController::State::Succeeded);
    EXPECT_NEAR(c.progress(), target, .01);
    EXPECT_EQ(c.command().vx, 0);
  }
}

TEST(MotionController, TimeoutStaleFeedbackAndLeaseExpiryStopWithoutResuming) {
  for (int failure = 0; failure < 3; ++failure) {
    MotionController c;
    MotionConfig config; config.timeout = 1;
    c.start(MotionController::Mode::Distance, 1, config, {}, 0, failure == 2 ? .2 : 0);
    ASSERT_GT(c.tick({}, .01, true)->vx, 0);
    c.keepalive(.1);
    const auto stopped = c.tick({}, failure == 0 ? 1.1 : .31, failure != 1);
    ASSERT_TRUE(stopped); EXPECT_EQ(stopped->vx, 0); EXPECT_EQ(stopped->yaw_rate, 0);
    EXPECT_EQ(c.state(), MotionController::State::Failed);
    c.keepalive(1.2);
    EXPECT_FALSE(c.tick({}, 1.3, true));
  }
  MotionController c;
  c.start(MotionController::Mode::RelativeTurn, 1, {}, {}, 0, .2);
  c.keepalive(.15);
  EXPECT_GT(c.tick({}, .3, true)->yaw_rate, 0);
  c.cancel("operator stop");
  EXPECT_EQ(c.tick({}, .31, true)->yaw_rate, 0);
  EXPECT_EQ(c.state(), MotionController::State::Canceled);
}

TEST(MotionController, RejectsInvalidGoalsWithoutReplacingRunningMotion) {
  MotionController c;
  c.start(MotionController::Mode::RelativeTurn, 1, {}, {}, 0);
  for (double value : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity()}) {
    MotionConfig config; config.track_width = value;
    EXPECT_THROW(c.start(MotionController::Mode::Distance, 1, config, {}, 1), std::invalid_argument);
  }
  EXPECT_THROW(c.start(MotionController::Mode::Distance, NAN, {}, {}, 0), std::invalid_argument);
  EXPECT_THROW(c.start(MotionController::Mode::Distance, 101, {}, {}, 0), std::invalid_argument);
  EXPECT_THROW(c.start(MotionController::Mode::Distance, 1, {}, {}, 0, -.1), std::invalid_argument);
  EXPECT_EQ(c.mode(), MotionController::Mode::RelativeTurn);
  EXPECT_TRUE(c.active());
}

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
namespace {
ImuFrame imu_frame(uint32_t id) {
  ImuFrame f; f.id = id; f.size = 32; f.fd = f.brs = true; return f;
}
void put16(ImuFrame& f, size_t at, int value) {
  const auto v = static_cast<uint16_t>(value);
  f.data[at] = static_cast<uint8_t>(v); f.data[at + 1] = static_cast<uint8_t>(v >> 8);
}
}
TEST(Imu, FirmwarePdoUnitsSignedFieldsAndMappingIsolation) {
  auto f = imu_frame(0x19F);
  put16(f, 0, -1000); put16(f, 4, 1000); put16(f, 6, 900); put16(f, 12, -123);
  put16(f, 18, 2567); f.data[20] = 42; f.data[24] = 3; f.data[25] = 2;
  const auto m = decode_imu_measurement(f, 31);
  ASSERT_TRUE(m);
  EXPECT_DOUBLE_EQ(m->acceleration[0], -9.80665);
  EXPECT_DOUBLE_EQ(m->acceleration[2], 9.80665);
  EXPECT_NEAR(m->angular_velocity[0], std::acos(-1.0)/2, 1e-12);
  EXPECT_NEAR(m->magnetic_field[0], -123e-7, 1e-15);
  EXPECT_DOUBLE_EQ(m->temperature, 25.67);
  EXPECT_EQ(m->counter, 42U); EXPECT_EQ(m->faults, 2U);
  EXPECT_FALSE(decode_imu_measurement(f, 30));
  f.extended = true; EXPECT_FALSE(decode_imu_measurement(f, 31)); f.extended = false;
  f.fd = false; EXPECT_FALSE(decode_imu_measurement(f, 31)); f.fd = true;
  f.data[31] = 1; EXPECT_FALSE(decode_imu_measurement(f, 31)); f.data[31] = 0;
  for (uint8_t n = 0; n < 32; ++n) { f.size = n; EXPECT_FALSE(decode_imu_measurement(f, 31)); }
}
TEST(Imu, QuaternionFreshnessNormalizationAndMounting) {
  auto f = imu_frame(0x29F);
  put16(f, 0, 7071); put16(f, 6, 7071); f.data[28] = 1;
  auto q = decode_imu_orientation(f, 31);
  ASSERT_TRUE(q && q->valid);
  EXPECT_NEAR(quaternion_yaw(q->quaternion), std::acos(-1.0)/2, 1e-12);
  auto mounting = mounting_rotation(0, 0, std::acos(-1.0)/2);
  EXPECT_NEAR(quaternion_yaw(multiply(q->quaternion, conjugate(mounting))), 0, 1e-12);
  const auto v = rotate(mounting, {1, 0, 0});
  EXPECT_NEAR(v[0], 0, 1e-12); EXPECT_NEAR(v[1], 1, 1e-12);
  f.data[24] = 250; EXPECT_FALSE(decode_imu_orientation(f, 31)->valid);
  f.data[24] = 0; f.data[28] = 0; EXPECT_FALSE(decode_imu_orientation(f, 31)->valid);
  f.data[28] = 1; put16(f, 0, 0); put16(f, 6, 0); EXPECT_FALSE(decode_imu_orientation(f, 31)->valid);
}
TEST(Imu, DuplicateCountersExpireWrapIsContinuousAndRebootsAreDetected) {
  ImuCounter counter;
  EXPECT_TRUE(counter.update(0xFFFFFFFFU, 1));
  EXPECT_TRUE(counter.update(0, 1.01)); EXPECT_FALSE(counter.restarted());
  EXPECT_FALSE(counter.update(0, 1.24)); EXPECT_FALSE(counter.fresh(1.27, 0.25));
  EXPECT_TRUE(counter.update(100, 1.3));
  EXPECT_TRUE(counter.update(1, 1.31)); EXPECT_TRUE(counter.restarted());
  EXPECT_FALSE(counter.fresh(1.2, 0.25));
  counter.reset(); EXPECT_FALSE(counter.fresh(1.32, 0.25));
}
TEST(Odometry, ImuReplacesWheelYawAndShapesTranslationArc) {
  Odometry odom;
  odom.set_heading(2.0, 0); odom.update(Motion{}, 0);
  odom.update(Motion{1, 0, 0}, 0.1);  // Align IMU reference to odom without a jump.
  odom.set_heading(2.0 + std::acos(-1.0)/2, 0.2);
  odom.update(Motion{1, 0, -1}, 0.2);
  EXPECT_NEAR(odom.pose().yaw, std::acos(-1.0)/2, 1e-12);
  EXPECT_NEAR(odom.pose().x, 0.1 + 0.2/std::acos(-1.0), 1e-12);
  EXPECT_NEAR(odom.pose().y, 0.2/std::acos(-1.0), 1e-12);
}
TEST(Odometry, ImuWrapFallbackRecoveryAndResetRemainContinuous) {
  Odometry odom(0.5);
  odom.set_heading(3.13, 0); odom.update(Mileage{}, 0); odom.update(Mileage{}, 0.1);
  odom.set_heading(-3.13, 0.2); odom.update(Mileage{}, 0.2);
  EXPECT_NEAR(odom.pose().yaw, 2*std::acos(-1.0)-6.26, 1e-12);
  const double yaw = odom.pose().yaw;
  odom.invalidate_heading();
  odom.update(Mileage{-25, 25, false}, 0.3);
  EXPECT_NEAR(odom.pose().yaw, yaw + 0.1, 1e-12);
  odom.set_heading(1.2, 0.4); odom.update(Mileage{-25, 25, false}, 0.4);
  EXPECT_NEAR(odom.pose().yaw, yaw + 0.1, 1e-12);
  odom.reset(); odom.update(Mileage{-25, 25, false}, 0.5);
  odom.set_heading(1.2, 0.6); odom.update(Mileage{-25, 25, false}, 0.6);
  EXPECT_DOUBLE_EQ(odom.pose().yaw, 0); EXPECT_DOUBLE_EQ(odom.pose().x, 0);
  odom.update(Mileage{-50, 50, false}, 0.8); // Last IMU sample still fresh.
  odom.update(Mileage{-75, 75, false}, 0.9); // Stale: resume wheel yaw.
  EXPECT_NEAR(odom.pose().yaw, 0.1, 1e-12);
}
TEST(YawCalibration, StartupIdentifiesBothSignsAcrossWrapAndReturnsNearStart) {
  for (int sign : {-1, 1}) {
    YawCalibration c;
    c.request();
    double yaw = 3.12, command = 0;
    bool saw_positive = false, saw_negative = false;
    for (int i = 0; i < 600 && c.pending(); ++i) {
      const double time = i * 0.02;
      yaw += sign * command * 0.02;
      c.observe(std::remainder(yaw, 2 * std::acos(-1.0)), time);
      command = c.tick(time, true).value_or(0);
      saw_positive |= command > 0;
      saw_negative |= command < 0;
    }
    EXPECT_TRUE(saw_positive); EXPECT_TRUE(saw_negative);
    EXPECT_EQ(c.state(), YawCalibration::State::Succeeded);
    EXPECT_EQ(c.sign(), sign);
    EXPECT_LT(c.positive_delta() * c.negative_delta(), 0);
    EXPECT_NEAR(yaw, 3.12, 0.012);
    EXPECT_DOUBLE_EQ(command, 0);
    EXPECT_FALSE(c.tick(20, true));  // A completed calibration never restarts itself.
  }
}
TEST(YawCalibration, MissingDataWaitsAndStalledMotionFailsWithStop) {
  YawCalibration c;
  c.request();
  EXPECT_FALSE(c.tick(1, true));
  c.observe(0, 1);
  EXPECT_FALSE(c.tick(1, false));
  for (int i = 0; i < 300 && c.pending(); ++i) {
    const double t = 2 + i * .02;
    c.observe(0, t);  // New samples, but no physical rotation.
    const auto command = c.tick(t, true);
    ASSERT_TRUE(command);
    if (!c.pending()) EXPECT_DOUBLE_EQ(*command, 0);
  }
  EXPECT_EQ(c.state(), YawCalibration::State::Failed);
  EXPECT_FALSE(c.tick(20, true));
}
TEST(YawCalibration, ImuGapInvalidationAndManualCancelProduceOneStop) {
  for (int failure = 0; failure < 3; ++failure) {
    YawCalibration c;
    c.request(); c.observe(0, 1); c.tick(1, true);
    c.observe(0, 1.4);  // The data gap itself aborts an active run.
    EXPECT_EQ(c.state(), YawCalibration::State::Failed);
    EXPECT_DOUBLE_EQ(*c.tick(1.4, true), 0);
    EXPECT_FALSE(c.tick(1.41, true));
    c.request(); c.observe(0, 2); c.tick(2, true);
    c.observe(0, 2.2); c.tick(2.2, true);
    c.observe(0, 2.31); ASSERT_GT(*c.tick(2.31, true), 0);
    if (failure == 0) c.invalidate();
    if (failure == 1) c.cancel("manual takeover");
    if (failure == 2) c.fail("CAN disconnected");
    EXPECT_DOUBLE_EQ(*c.tick(2.32, true), 0);
    EXPECT_FALSE(c.tick(2.33, true));
    EXPECT_FALSE(c.pending());
  }
}
TEST(YawCalibration, RejectsSameDirectionInBothLegsAndInvalidConfiguration) {
  YawCalibration c;
  c.request();
  double yaw = 0, command = 0;
  for (int i = 0; i < 600 && c.pending(); ++i) {
    yaw += std::abs(command) * .02;  // A broken direction response, not a valid sign flip.
    c.observe(yaw, i*.02);
    command = c.tick(i*.02, true).value_or(0);
  }
  EXPECT_EQ(c.state(), YawCalibration::State::Failed);
  EXPECT_DOUBLE_EQ(command, 0);
  YawCalibrationConfig config;
  config.angle = .01;
  EXPECT_THROW(YawCalibration{config}, std::invalid_argument);
  config.angle = .2; config.speed = NAN;
  EXPECT_THROW(YawCalibration{config}, std::invalid_argument);
}
