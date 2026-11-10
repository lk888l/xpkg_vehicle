// SPDX-License-Identifier: Apache-2.0
#include "vehicle_core/yaw_calibration.hpp"
#include <cmath>
#include <stdexcept>

namespace vehicle_core {
YawCalibration::YawCalibration(YawCalibrationConfig config) : config_(config) {
  for (double value : {config.speed, config.angle, config.min_angle, config.settle_time, config.phase_timeout})
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("invalid yaw calibration configuration");
  if (config.angle < config.min_angle)
    throw std::invalid_argument("calibration angle must be >= min_angle");
}
void YawCalibration::request() {
  stop_pending_ = false;
  state_ = State::Waiting;
  positive_delta_ = negative_delta_ = 0;
  message_ = "Waiting for enabled CAN mode and fresh IMU yaw";
}
bool YawCalibration::fresh(double time) const {
  return sample_time_ && time >= *sample_time_ && time - *sample_time_ < 0.25;
}
void YawCalibration::observe(double yaw, double time) {
  if (!std::isfinite(yaw) || !std::isfinite(time)) { invalidate(); return; }
  if (sample_time_ && time <= *sample_time_) return;
  if (!fresh(time)) {
    if (active()) fail("IMU data gap during direction calibration");
    yaw_.reset();
  }
  unwrapped_ = yaw_ ? unwrapped_ + std::remainder(yaw - *yaw_, 2 * std::acos(-1.0)) : yaw;
  yaw_ = yaw;
  sample_time_ = time;
}
void YawCalibration::invalidate() {
  yaw_.reset(); sample_time_.reset();
  if (active()) fail("IMU orientation invalidated during direction calibration");
}
void YawCalibration::cancel(const std::string& reason) {
  if (!pending()) return;
  stop_pending_ = active();
  state_ = State::Canceled; message_ = reason;
}
void YawCalibration::fail(const std::string& reason) {
  if (!pending()) return;
  stop_pending_ = active();
  state_ = State::Failed; message_ = reason;
}
void YawCalibration::enter(State state, double time, const std::string& message) {
  state_ = state; phase_time_ = time; message_ = message;
}
double YawCalibration::command() const {
  return state_ == State::Positive ? config_.speed : state_ == State::Negative ? -config_.speed : 0;
}
std::optional<double> YawCalibration::tick(double time, bool chassis_ready) {
  if (stop_pending_) { stop_pending_ = false; return 0; }
  if (state_ == State::Waiting) {
    if (!chassis_ready || !fresh(time)) return std::nullopt;
    enter(State::Settling, time, "Settling before positive yaw test");
    return 0;
  }
  if (!active()) return std::nullopt;
  if (!chassis_ready || !fresh(time) || time < phase_time_) {
    fail("Chassis feedback or IMU yaw unavailable during calibration");
    stop_pending_ = false;
    return 0;
  }
  const double elapsed = time - phase_time_;
  switch (state_) {
    case State::Settling:
      if (elapsed >= config_.settle_time) {
        positive_start_ = unwrapped_;
        enter(State::Positive, time, "Measuring positive chassis yaw");
      }
      break;
    case State::Positive:
      positive_delta_ = unwrapped_ - positive_start_;
      if (std::abs(positive_delta_) >= config_.angle || elapsed >= config_.phase_timeout) {
        if (std::abs(positive_delta_) < config_.min_angle) fail("Positive yaw response too small");
        else enter(State::Pause, time, "Settling before reverse yaw test");
      }
      break;
    case State::Pause:
      positive_delta_ = unwrapped_ - positive_start_;
      if (elapsed >= config_.settle_time) {
        if (std::abs(positive_delta_) < config_.min_angle) fail("Positive yaw response too small after settling");
        else {
          negative_start_ = unwrapped_;
          enter(State::Negative, time, "Measuring negative chassis yaw and returning");
        }
      }
      break;
    case State::Negative:
      negative_delta_ = unwrapped_ - negative_start_;
      if (std::abs(negative_delta_) >= std::abs(positive_delta_) || elapsed >= config_.phase_timeout) {
        if (std::abs(negative_delta_) < config_.min_angle) fail("Negative yaw response too small");
        else enter(State::Verifying, time, "Settling and checking both yaw directions");
      }
      break;
    case State::Verifying:
      negative_delta_ = unwrapped_ - negative_start_;
      if (elapsed >= config_.settle_time) {
        if (std::abs(negative_delta_) < config_.min_angle || positive_delta_ * negative_delta_ >= 0) {
          fail("Positive and negative yaw responses are not opposite");
        } else {
          sign_ = positive_delta_ > 0 ? 1 : -1;
          enter(State::Succeeded, time, sign_ > 0 ? "IMU yaw direction confirmed: +1" : "IMU yaw direction corrected: -1");
        }
      }
      break;
    default: break;
  }
  stop_pending_ = false;  // This tick already returns the terminal stop when it fails.
  return command();
}
}  // namespace vehicle_core
