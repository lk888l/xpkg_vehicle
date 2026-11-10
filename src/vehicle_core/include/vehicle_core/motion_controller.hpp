// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include "vehicle_core/odometry.hpp"

namespace vehicle_core {
struct MotionConfig {
  double track_width{0.535}, wheelbase{0.420}, skid_compensation{0.5};
  double max_linear_speed{0.1}, max_angular_speed{0.2};
  double heading_gain{1.8}, yaw_rate_gain{0.2}, angle_tolerance{0.017453292519943295};
  double distance_gain{0.8}, distance_tolerance{0.01}, timeout{120.0};
  void validate() const;
  // Symmetric, linear slip-resistance approximation; requires surface calibration.
  // k is lateral / longitudinal resistance. It is NOT determined by geometry.
  double feedforward_gain() const;
};

class MotionController {
 public:
  enum class Mode : uint8_t { Cancel = 0, RelativeTurn = 1, Heading = 2, Distance = 3 };
  enum class State : uint8_t { Idle = 0, Running = 1, Succeeded = 2, Canceled = 3, Failed = 4 };
  void start(Mode mode, double target, const MotionConfig& config, const Pose& pose,
             double now, double client_timeout = 0);
  void cancel(const std::string& reason);
  void fail(const std::string& reason);
  void keepalive(double now);
  std::optional<Motion> tick(const Pose& pose, double now, bool feedback_ready);
  bool active() const { return state_ == State::Running; }
  State state() const { return state_; }
  Mode mode() const { return mode_; }
  const std::string& message() const { return message_; }
  const MotionConfig& config() const { return config_; }
  const Motion& command() const { return command_; }
  double target() const { return target_; }
  double progress() const { return progress_; }
  double remaining() const { return target_ - progress_; }
  double target_yaw() const { return target_yaw_; }
  double heading_error() const { return heading_error_; }

 private:
  void finish(State state, const std::string& reason);
  MotionConfig config_;
  Mode mode_{Mode::Cancel};
  State state_{State::Idle};
  std::string message_{"等待运动命令"};
  Pose start_;
  Motion command_;
  double target_{}, progress_{}, target_yaw_{}, heading_error_{}, yaw_{}, previous_yaw_{};
  double started_{}, last_keepalive_{}, client_timeout_{};
  std::optional<double> settled_since_;
  bool stop_pending_{};
};
}  // namespace vehicle_core
