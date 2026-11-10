// SPDX-License-Identifier: Apache-2.0
#include "vehicle_core/motion_controller.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vehicle_core {
namespace {
constexpr double pi = 3.14159265358979323846;
double wrap(double value) { return std::remainder(value, 2 * pi); }
bool valid_pose(const Pose& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw) &&
         std::isfinite(p.vx) && std::isfinite(p.yaw_rate);
}
}
void MotionConfig::validate() const {
  const auto within = [](double v, double lo, double hi) {
    return std::isfinite(v) && v >= lo && v <= hi;
  };
  if (!within(track_width, 0.05, 5) || !within(wheelbase, 0.05, 5) ||
      !within(skid_compensation, 0, 5) || feedforward_gain() > 10 ||
      !within(max_linear_speed, 0.01, 5) || !within(max_angular_speed, 0.01, 5) ||
      !within(heading_gain, 0.01, 20) || !within(yaw_rate_gain, 0, 5) ||
      !within(angle_tolerance, 0.001, 0.35) || !within(distance_gain, 0.01, 10) ||
      !within(distance_tolerance, 0.001, 0.2) || !within(timeout, 0.1, 3600))
    throw std::invalid_argument("运动参数无效：检查尺寸、增益、速度、误差与超时范围");
}
double MotionConfig::feedforward_gain() const {
  return 1 + skid_compensation * std::pow(wheelbase / track_width, 2);
}
void MotionController::start(Mode mode, double target, const MotionConfig& config,
                             const Pose& pose, double now, double client_timeout) {
  config.validate();
  if ((mode != Mode::RelativeTurn && mode != Mode::Heading && mode != Mode::Distance) ||
      !std::isfinite(target) || !valid_pose(pose) || !std::isfinite(now) ||
      !std::isfinite(client_timeout) || client_timeout < 0 || client_timeout > 5 ||
      (client_timeout > 0 && client_timeout < 0.1) ||
      std::abs(target) > (mode == Mode::Distance ? 100.0 : 20*pi))
    throw std::invalid_argument("运动目标或续期时间无效");
  config_ = config;
  mode_ = mode;
  start_ = pose;
  target_ = mode == Mode::Heading ? wrap(target - pose.yaw) : target;
  target_yaw_ = pose.yaw + (mode == Mode::Distance ? 0 : target_);
  heading_error_ = target_yaw_ - pose.yaw;
  progress_ = 0;
  previous_yaw_ = yaw_ = pose.yaw;
  started_ = last_keepalive_ = now;
  client_timeout_ = client_timeout;
  settled_since_.reset();
  command_ = {};
  stop_pending_ = false;
  state_ = State::Running;
  message_ = mode == Mode::Distance ? "定距行驶中" : "定角转向中";
}
void MotionController::finish(State state, const std::string& reason) {
  if (!active()) return;
  state_ = state;
  message_ = reason;
  command_ = {};
  stop_pending_ = true;
}
void MotionController::cancel(const std::string& reason) { finish(State::Canceled, reason); }
void MotionController::fail(const std::string& reason) { finish(State::Failed, reason); }
void MotionController::keepalive(double now) {
  if (active() && std::isfinite(now) && now >= last_keepalive_) last_keepalive_ = now;
}
std::optional<Motion> MotionController::tick(const Pose& pose, double now, bool feedback_ready) {
  if (active()) {
    if (!feedback_ready || !valid_pose(pose)) fail("底盘或 IMU / 里程计反馈无效，已停止");
    else if (!std::isfinite(now) || now < started_ || now - started_ >= config_.timeout)
      fail("运动执行超时，已停止");
    else if (client_timeout_ > 0 && now - last_keepalive_ >= client_timeout_)
      fail("控制端续期超时，已停止");
  }
  if (!active()) {
    if (stop_pending_) { stop_pending_ = false; return Motion{}; }
    return std::nullopt;
  }
  yaw_ += wrap(pose.yaw - previous_yaw_);
  previous_yaw_ = pose.yaw;
  const bool distance = mode_ == Mode::Distance;
  progress_ = distance ? (pose.x-start_.x)*std::cos(start_.yaw) +
                         (pose.y-start_.y)*std::sin(start_.yaw) : yaw_ - start_.yaw;
  heading_error_ = distance ? wrap(target_yaw_ - yaw_) : target_yaw_ - yaw_;
  const bool angle_reached = std::abs(heading_error_) <= config_.angle_tolerance;
  const bool distance_reached = !distance || std::abs(remaining()) <= config_.distance_tolerance;
  // Heading P generates a reference rate, then geometry/slip feedforward and
  // measured yaw-rate feedback produce the bounded CAN body-rate request.
  const double reference = std::clamp(config_.heading_gain * heading_error_,
                                      -config_.max_angular_speed, config_.max_angular_speed);
  command_.yaw_rate = angle_reached ? 0 : std::clamp(
      config_.feedforward_gain()*reference + config_.yaw_rate_gain*(reference-pose.yaw_rate),
      -config_.max_angular_speed, config_.max_angular_speed);
  command_.vx = distance && !distance_reached ? std::clamp(config_.distance_gain * remaining(),
      -config_.max_linear_speed, config_.max_linear_speed) * std::max(0.0, std::cos(heading_error_)) : 0;
  if (angle_reached && distance_reached && std::abs(pose.vx) < 0.03 && std::abs(pose.yaw_rate) < 0.05) {
    if (!settled_since_) settled_since_ = now;
    if (now - *settled_since_ >= 0.2) {
      finish(State::Succeeded, distance ? "定距行驶完成" : "转向完成");
      stop_pending_ = false;
    }
  } else settled_since_.reset();
  return command_;
}
}  // namespace vehicle_core
