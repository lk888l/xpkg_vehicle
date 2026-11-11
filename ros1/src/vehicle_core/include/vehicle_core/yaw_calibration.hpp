// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cstdint>
#include <optional>
#include <string>

namespace vehicle_core {
struct YawCalibrationConfig {
  double speed{0.30};
  double angle{0.17453292519943295};  // 10 degrees, each direction.
  double min_angle{0.05235987755982989};  // 3 degrees to distinguish motion from noise.
  double settle_time{0.30};
  double phase_timeout{3.0};
};
// A one-shot, nonblocking sign identification against commanded chassis yaw.
// Uses raw mounted IMU yaw, never the already corrected odometry heading.
class YawCalibration {
 public:
  enum class State : uint8_t {
    Disabled, Waiting, Settling, Positive, Pause, Negative, Verifying, Succeeded, Failed, Canceled
  };
  explicit YawCalibration(YawCalibrationConfig config = {});
  void request();
  void observe(double yaw, double sample_time);
  void invalidate();
  void cancel(const std::string& reason);
  void fail(const std::string& reason);
  // nullopt: calibration does not own motion. Zero: explicitly send stop.
  std::optional<double> tick(double time, bool chassis_ready);
  bool pending() const { return state_ == State::Waiting || active(); }
  bool active() const { return state_ >= State::Settling && state_ <= State::Verifying; }
  bool fresh(double time) const;
  State state() const { return state_; }
  int sign() const { return sign_; }
  double positive_delta() const { return positive_delta_; }
  double negative_delta() const { return negative_delta_; }
  double command() const;
  const std::string& message() const { return message_; }

 private:
  void enter(State state, double time, const std::string& message);
  YawCalibrationConfig config_;
  State state_{State::Disabled};
  std::optional<double> yaw_, sample_time_;
  double unwrapped_{}, phase_time_{}, positive_start_{}, negative_start_{};
  double positive_delta_{}, negative_delta_{};
  int sign_{1};
  bool stop_pending_{};
  std::string message_{"Automatic IMU direction calibration disabled"};
};
}  // namespace vehicle_core
