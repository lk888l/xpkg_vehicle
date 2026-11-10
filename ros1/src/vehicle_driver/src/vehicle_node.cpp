// SPDX-License-Identifier: Apache-2.0
#include <linux/can/error.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <cmath>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <optional>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/UInt64.h>
#include <stdexcept>
#include <string>
#include <vehicle_core/control.hpp>
#include <vehicle_core/motion_controller.hpp>
#include <vehicle_core/odometry.hpp>
#include <vehicle_core/transport.hpp>
#include <vehicle_core/yaw_calibration.hpp>
#include <vehicle_interfaces/Acknowledgement.h>
#include <vehicle_interfaces/CanFrame.h>
#include <vehicle_interfaces/DriverMotion.h>
#include <vehicle_interfaces/DriverStatus.h>
#include <vehicle_interfaces/Faults.h>
#include <vehicle_interfaces/Heading.h>
#include <vehicle_interfaces/ImuCalibration.h>
#include <vehicle_interfaces/MotionStatus.h>
#include <vehicle_interfaces/Remote.h>
#include <vehicle_interfaces/Sensors.h>
#include <vehicle_interfaces/State.h>
#include <vehicle_interfaces/Version.h>
#include <vehicle_interfaces/WheelMileage.h>
#include <vehicle_interfaces/Command.h>
#include <vehicle_interfaces/ExecuteMotion.h>

namespace vehicle_driver {
namespace core = vehicle_core;
namespace msg = vehicle_interfaces;
using CommandService = vehicle_interfaces::Command;
using MotionService = vehicle_interfaces::ExecuteMotion;
using Clock = std::chrono::steady_clock;
static double monotonic() {
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}
// CAN, timers and subscriptions run on the main queue. Command service waits on a
// separate queue and releases mutex_, so ACK waits never block CAN or the watchdog.
class VehicleNode final {
 public:
  VehicleNode() {
    receive_only_ = parameter<bool>("receive_only", true);
    interface_ = parameter<std::string>("can_interface", "can0");
    const int type = parameter<int>("device_type", 0xE1);
    const int node = parameter<int>("device_node", 1);
    if (type < 0 || type > 255 || node < 0 || node > 255)
      throw std::invalid_argument("CAN device_type and device_node must be bytes");
    address_ = {static_cast<uint8_t>(type), static_cast<uint8_t>(node)};
    initialize_ = parameter<bool>("auto_initialize", false) && !receive_only_;
    disable_on_shutdown_ = parameter<bool>("disable_on_shutdown", true);
    const bool stamped = parameter<bool>("use_stamped_cmd_vel", true);
    const double rate = positive("command_rate", 50.0);
    if (rate > 500.0) throw std::invalid_argument("command_rate must be <= 500 Hz");
    command_timeout_ = positive("command_timeout", 0.30);
    max_linear_speed_ = positive("max_linear_speed", 1.0);
    max_angular_speed_ = positive("max_angular_speed", 2.0);
    commands_ = std::make_unique<core::CommandBuffer>(
        command_timeout_, max_linear_speed_, max_angular_speed_);
    motion_config_.track_width = positive("motion_track_width", 0.535);
    motion_config_.wheelbase = positive("motion_wheelbase", 0.420);
    motion_config_.skid_compensation = parameter<double>("motion_skid_compensation", 0.5);
    motion_config_.max_linear_speed = positive("motion_linear_speed", 0.1);
    motion_config_.max_angular_speed = positive("motion_angular_speed", 0.2);
    motion_config_.heading_gain = positive("motion_heading_gain", 1.8);
    motion_config_.yaw_rate_gain = parameter<double>("motion_yaw_rate_gain", 0.2);
    motion_config_.angle_tolerance = positive("motion_angle_tolerance", motion_config_.angle_tolerance);
    motion_config_.distance_gain = positive("motion_distance_gain", 0.8);
    motion_config_.distance_tolerance = positive("motion_distance_tolerance", 0.01);
    motion_config_.timeout = positive("motion_timeout", 120.0);
    motion_config_.validate();
    service_timeout_ = positive("service_timeout", 1.2);
    heartbeat_timeout_ = positive("heartbeat_timeout", 1.5);
    remote_feedback_ = parameter<bool>("remote_feedback", false);
    sensor_feedback_ = parameter<bool>("sensor_feedback", false);
    driver_motion_feedback_ = parameter<bool>("driver_motion_feedback", true);
    driver_status_feedback_ = parameter<bool>("driver_status_feedback", true);
    const auto odom_source = parameter<std::string>("odom_source", "velocity");
    if (odom_source != "velocity" && odom_source != "wheel_mileage")
      throw std::invalid_argument("odom_source must be velocity or wheel_mileage");
    wheel_odometry_ = odom_source == "wheel_mileage";
    const double width = parameter<double>("effective_track_width", 0.0);
    if (wheel_odometry_ && (!std::isfinite(width) || width <= 0))
      throw std::invalid_argument("wheel_mileage requires calibrated effective_track_width > 0");
    odometry_ = std::make_unique<core::Odometry>(width);
    use_imu_heading_ = parameter<bool>("use_imu_heading", true);
    imu_yaw_sign_ = parameter<int>("imu_yaw_sign", 1);
    if (imu_yaw_sign_ != 1 && imu_yaw_sign_ != -1)
      throw std::invalid_argument("imu_yaw_sign must be +1 or -1");
    core::YawCalibrationConfig calibration_config;
    calibration_config.speed = std::min(positive("imu_calibration_speed", 0.30),
                                        max_angular_speed_);
    calibration_config.angle = positive("imu_calibration_angle", calibration_config.angle);
    calibration_config.min_angle = positive("imu_calibration_min_angle", calibration_config.min_angle);
    calibration_config.settle_time = positive("imu_calibration_settle_time", 0.30);
    calibration_config.phase_timeout = positive("imu_calibration_phase_timeout", 3.0);
    calibration_ = std::make_unique<core::YawCalibration>(calibration_config);
    auto_imu_calibration_ = parameter<bool>("auto_imu_calibration", false) && !receive_only_;
    if (auto_imu_calibration_ && use_imu_heading_) calibration_->request();
    odom_frame_ = parameter<std::string>("odom_frame", "odom");
    base_frame_ = parameter<std::string>("base_frame", "base_link");
    if (odom_frame_.empty() || base_frame_.empty() || odom_frame_ == base_frame_)
      throw std::invalid_argument("odom_frame and base_frame must be nonempty and distinct");
    pose_xy_ = positive("pose_variance_xy", 0.05);
    pose_yaw_ = positive("pose_variance_yaw", 0.10);
    twist_linear_ = positive("twist_variance_linear", 0.02);
    twist_angular_ = positive("twist_variance_angular", 0.05);
    if (parameter<bool>("publish_tf", true))
      tf_ = std::make_unique<tf2_ros::TransformBroadcaster>();
    const uint32_t telemetry_qos = 5;
    raw_pub_ = private_.advertise<msg::CanFrame>("can_rx", 128);
    state_pub_ = private_.advertise<msg::State>("state", 1, true);
    version_pub_ = private_.advertise<msg::Version>("version", 1, true);
    ack_pub_ = private_.advertise<msg::Acknowledgement>("acknowledgements", 10);
    velocity_pub_ = private_.advertise<geometry_msgs::TwistStamped>("velocity", telemetry_qos);
    mileage_pub_ = private_.advertise<msg::WheelMileage>("wheel_mileage", telemetry_qos);
    driver_motion_pub_ = private_.advertise<msg::DriverMotion>("driver_motion", telemetry_qos);
    driver_status_pub_ = private_.advertise<msg::DriverStatus>("driver_status", telemetry_qos);
    remote_pub_ = private_.advertise<msg::Remote>("remote", telemetry_qos);
    sensors_pub_ = private_.advertise<msg::Sensors>("sensors", telemetry_qos);
    faults_pub_ = private_.advertise<msg::Faults>("faults", 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
    heading_pub_ = private_.advertise<msg::Heading>("heading", 10);
    calibration_pub_ = private_.advertise<msg::ImuCalibration>("imu_calibration", 1, true);
    motion_pub_ = private_.advertise<msg::MotionStatus>("motion_status", 1, true);
    motion_service_ = private_.advertiseService("execute_motion", &VehicleNode::motion_request, this);
    keepalive_sub_ = nh_.subscribe<std_msgs::UInt64>(private_.resolveName("motion_keepalive"), 1,
        [this](std_msgs::UInt64::ConstPtr m) {
          std::lock_guard<std::mutex> guard(mutex_);
          if (m->data == motion_id_) motion_.keepalive(monotonic());
        });
    if (use_imu_heading_) {
      imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(
          "imu/data", 10, [this](sensor_msgs::Imu::ConstPtr m) {
          std::lock_guard<std::mutex> guard(mutex_);
            receive_imu(*m);
          });
    }
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);
    if (stamped) {
      stamped_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(
          "cmd_vel", 1, [this](geometry_msgs::TwistStamped::ConstPtr m) {
          std::lock_guard<std::mutex> guard(mutex_);
            calibration_->cancel("Canceled by cmd_vel input");
            motion_.cancel("速度命令已接管");
            try {
              if (m->header.stamp.sec != 0 || m->header.stamp.nsec != 0) {
                const double age =
                    (ros::Time::now() - m->header.stamp)
                        .toSec();
                if (age > command_timeout_ || age < -0.1) {
                  ++rejected_commands_;
                  commands_->stop();
                  return;
                }
              }
              accept_command(m->twist);
            } catch (const std::exception&) {
              ++rejected_commands_;
              commands_->stop();
            }
          });
    } else {
      twist_sub_ = nh_.subscribe<geometry_msgs::Twist>(
          "cmd_vel", 1,
          [this](geometry_msgs::Twist::ConstPtr m) {
          std::lock_guard<std::mutex> guard(mutex_); accept_command(*m); });
    }
    command_nh_.setCallbackQueue(&command_queue_);
    service_ = command_nh_.advertiseService("command", &VehicleNode::command_request, this);
    reset_odom_ = private_.advertiseService("reset_odometry", &VehicleNode::reset_odometry, this);
    calibrate_imu_ = private_.advertiseService("calibrate_imu", &VehicleNode::calibrate_imu, this);
    connect();
    poll_timer_ = nh_.createWallTimer(ros::WallDuration(0.002), [this](const ros::WallTimerEvent&) {
      std::lock_guard<std::mutex> guard(mutex_); poll_can(); });
    control_timer_ =
        nh_.createWallTimer(ros::WallDuration(1.0 / rate), [this](const ros::WallTimerEvent&) {
          std::lock_guard<std::mutex> guard(mutex_); control_tick(); });
    health_timer_ = nh_.createWallTimer(ros::WallDuration(0.5), [this](const ros::WallTimerEvent&) {
      std::lock_guard<std::mutex> guard(mutex_); health_tick(); });
    ROS_INFO("Chassis receive_only=%s", receive_only_ ? "true" : "false");
    publish_calibration();
    publish_motion_status(true);
    ROS_INFO( "SocketCAN %s, vehicle ID 0x%08X, odometry=%s, cmd_vel=%s",
                interface_.c_str(), address_.base(), odom_source.c_str(),
                stamped ? "TwistStamped" : "Twist");
    command_spinner_.start();
  }
  ~VehicleNode() {
    {
      std::lock_guard<std::mutex> guard(mutex_);
      shutting_down_ = true;
      pending_changed_.notify_all();
    }
    command_spinner_.stop();
    if (transport_ && !receive_only_) {
      try {
        transport_->send(core::motion(address_, 0, 0, 0));
      } catch (...) {
      }
      if (disable_on_shutdown_) {
        try {
          transport_->send(core::enable(address_, false));
        } catch (...) {
        }
      }
    }
  }

 private:
  template <class T>
  T parameter(const std::string& name, const T& value) {
    T result;
    private_.param<T>(name, result, value);
    return result;
  }

  double positive(const std::string& name, double value) {
    const auto result = parameter<double>(name, value);
    if (!std::isfinite(result) || result <= 0)
      throw std::invalid_argument(name + " must be positive and finite");
    return result;
  }
  template <class T>
  T message(const ros::Time& stamp) const {
    T m;
    m.header.stamp = stamp;
    m.header.frame_id = base_frame_;
    return m;
  }
  void accept_command(const geometry_msgs::Twist& twist) {
    if (receive_only_) { ++rejected_commands_; return; }
    calibration_->cancel("Canceled by cmd_vel input");
    motion_.cancel("速度命令已接管");
    if (!std::isfinite(twist.linear.y) || !std::isfinite(twist.linear.z) ||
        !std::isfinite(twist.angular.x) || !std::isfinite(twist.angular.y) ||
        !commands_->set(twist.linear.x, twist.angular.z, monotonic())) {
      ++rejected_commands_;
      commands_->stop();
      return;
    }
    if (twist.linear.y != 0 || twist.linear.z != 0 || twist.angular.x != 0 || twist.angular.y != 0)
      ROS_WARN_THROTTLE(5.0,
          "Differential chassis uses linear.x and angular.z; other axes are ignored");
  }
  void connect() {
    next_connect_ = monotonic() + 1.0;
    try {
      transport_ = std::make_unique<core::SocketCan>(interface_, address_);
      io_error_.clear();
      odometry_->rebase();
      ROS_INFO( "Opened %s", interface_.c_str());
    } catch (const std::exception& error) {
      io_error_ = error.what();
      ROS_WARN_THROTTLE(5.0, "%s", io_error_.c_str());
    }
  }
  void disconnect(const std::string& error) {
    io_error_ = error;
    transport_.reset();
    next_connect_ = monotonic() + 1.0;
    commands_->stop();
    motion_.fail("底盘 CAN 断开，运动已停止");
    if (calibration_->active()) calibration_->fail("Chassis CAN transport disconnected");
    odometry_->rebase();
    // Never replay a command cached before a transport failure.
    ROS_ERROR_THROTTLE(2.0, "%s", error.c_str());
  }
  bool transmit(const core::Frame& frame) {
    if (receive_only_) { ++blocked_transmissions_; return false; }
    if (!transport_) return false;
    try {
      transport_->send(frame);
      ++tx_frames_;
      return true;
    } catch (const std::exception& error) {
      ++tx_errors_;
      disconnect(error.what());
      return false;
    }
  }
  void poll_can() {
    if (!transport_) {
      if (monotonic() >= next_connect_) connect();
      check_pending();
      return;
    }
    try {
      // Bound each callback so traffic cannot starve ROS callbacks or the watchdog.
      for (size_t i = 0; i < 256 && transport_; ++i) {
        auto frame = transport_->receive();
        if (!frame) break;
        const auto stamp = ros::Time::now();
        const double tick = monotonic();
        ++rx_frames_;
        auto raw = message<msg::CanFrame>(stamp);
        raw.id = frame->id;
        raw.length = frame->size;
        std::copy(frame->data.begin(), frame->data.end(), raw.data.begin());
        raw.extended = frame->extended;
        raw.remote = frame->remote;
        raw.error = frame->error;
        raw_pub_.publish(raw);
        if (frame->error) {
          ++error_frames_;
          if ((frame->id & CAN_ERR_BUSOFF) != 0) disconnect("SocketCAN reported bus-off");
          continue;
        }
        auto decoded = core::decode(*frame, address_);
        if (decoded.status == core::DecodeStatus::Malformed) {
          ++malformed_frames_;
          continue;
        }
        if (decoded.event)
          std::visit([this, &stamp, tick](const auto& event) { receive(event, stamp, tick); },
                     *decoded.event);
      }
    } catch (const std::exception& error) {
      ++rx_errors_;
      disconnect(error.what());
    }
    check_pending();
  }
  void control_tick() {
    const double tick = monotonic();
    if (heartbeat_time_ && tick - *heartbeat_time_ > heartbeat_timeout_) commands_->stop();
    const bool ready = transport_ && heartbeat_time_ && tick - *heartbeat_time_ < heartbeat_timeout_ &&
        state_.enabled && status_time_ && tick - *status_time_ < 0.5 && state_.mode == 2 && state_.work_state == 0;
    const auto previous = calibration_->state();
    const auto calibration_command = calibration_->tick(tick, ready);
    if (previous == core::YawCalibration::State::Waiting && calibration_->active()) {
      commands_->stop();
      odometry_->invalidate_heading();
    }
    if (previous != core::YawCalibration::State::Succeeded &&
        calibration_->state() == core::YawCalibration::State::Succeeded) {
      imu_yaw_sign_ = calibration_->sign();
      imu_yaw_calibrated_ = true;
      odometry_->invalidate_heading();
      if (raw_imu_yaw_ && raw_imu_time_ && calibration_->fresh(tick))
        odometry_->set_heading(imu_yaw_sign_ * *raw_imu_yaw_, *raw_imu_time_);
    }
    if (calibration_command) {
      transmit(core::motion(address_, 0, 0, *calibration_command));
    } else if (auto automatic = motion_.tick(odometry_->pose(), tick, motion_ready(tick))) {
      transmit(core::motion(address_, automatic->vx, 0, automatic->yaw_rate));
    } else if (auto command = commands_->sample(tick)) {
      transmit(core::motion(address_, command->vx, 0, command->yaw_rate));
    }
    publish_calibration();
    publish_motion_status();
    if (startup_wait_ && tick > startup_deadline_) {
      startup_wait_ = false;
      initialize_ = false;
      ROS_WARN(
                  "Startup enable not observed; inspect state and use command service");
    }
  }
  bool motion_ready(double tick) const {
    return transport_ && heartbeat_time_ && tick - *heartbeat_time_ < heartbeat_timeout_ &&
        state_.enabled && status_time_ && tick - *status_time_ < 0.5 && state_.mode == 2 &&
        state_.work_state == 0 && odom_time_ && tick - *odom_time_ < 0.25 &&
        odometry_->heading_fresh(tick) && !calibration_->pending();
  }
  void execute_motion(const MotionService::Request& request, MotionService::Response& response) {
    response.goal_id = motion_id_;
    if (receive_only_ && request.mode != MotionService::Request::CANCEL) {
      response.message = "receive_only is enabled; chassis motion is prohibited";
      return;
    }
    if (request.mode == MotionService::Request::CANCEL) {
      motion_.cancel("运动命令已取消");
      calibration_->cancel("Canceled by motion stop");
      commands_->stop();
      transmit(core::motion(address_, 0, 0, 0));
      response.accepted = true;
      response.message = "已请求停止";
      publish_motion_status(true);
      return;
    }
    if (!motion_ready(monotonic())) {
      response.message = "需要已使能的 CAN 模式、新鲜里程计与已就绪的 IMU 航向；标定期间不能执行";
      return;
    }
    try {
      auto config = motion_config_;
      if (request.use_custom_config) {
        config.track_width = request.track_width;
        config.wheelbase = request.wheelbase;
        config.skid_compensation = request.skid_compensation;
        config.max_linear_speed = request.max_linear_speed;
        config.max_angular_speed = request.max_angular_speed;
        config.heading_gain = request.heading_gain;
        config.yaw_rate_gain = request.yaw_rate_gain;
        config.angle_tolerance = request.angle_tolerance;
        config.distance_gain = request.distance_gain;
        config.distance_tolerance = request.distance_tolerance;
        config.timeout = request.timeout;
      }
      config.validate();
      config.max_linear_speed = std::min(config.max_linear_speed, max_linear_speed_);
      config.max_angular_speed = std::min(config.max_angular_speed, max_angular_speed_);
      motion_.start(static_cast<core::MotionController::Mode>(request.mode), request.target,
                    config, odometry_->pose(), monotonic(), request.client_timeout);
      ++motion_id_;
      commands_->stop();
      // Consume the old manual stop now so it cannot replay after this goal.
      commands_->sample(monotonic());
      if (!transmit(core::motion(address_, 0, 0, 0))) {
        motion_.fail("底盘停止指令发送失败");
        response.message = motion_.message();
      } else {
        response.accepted = true;
        response.message = "运动请求已受理，完成结果见 motion_status";
      }
      response.goal_id = motion_id_;
      publish_motion_status(true);
    } catch (const std::exception& error) {
      response.message = error.what();
    }
  }
  void publish_motion_status(bool force = false) {
    const double tick = monotonic();
    if (!force && last_motion_state_ == motion_.state() && tick - last_motion_publish_ < 0.1) return;
    last_motion_publish_ = tick;
    last_motion_state_ = motion_.state();
    auto m = message<msg::MotionStatus>(ros::Time::now());
    m.header.frame_id = odom_frame_;
    m.goal_id = motion_id_;
    m.state = static_cast<uint8_t>(motion_.state());
    m.mode = static_cast<uint8_t>(motion_.mode());
    m.active = motion_.active();
    m.target = motion_.target(); m.progress = motion_.progress(); m.remaining = motion_.remaining();
    m.target_yaw = motion_.target_yaw(); m.heading_error = motion_.heading_error();
    m.command_linear_speed = motion_.command().vx; m.command_yaw_rate = motion_.command().yaw_rate;
    m.feedforward_gain = motion_.config().feedforward_gain();
    m.message = motion_.message();
    motion_pub_.publish(m);
  }
  void receive_imu(const sensor_msgs::Imu& m) {
    const auto& q = m.orientation;
    const double norm = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    const double age = (ros::Time::now() - m.header.stamp).toSec();
    if (m.header.frame_id != base_frame_ || m.orientation_covariance[0] < 0 ||
        !std::isfinite(norm) || norm < 0.98 || norm > 1.02 || age < -0.1 || age >= 0.25) {
      calibration_->invalidate();
      motion_.fail("IMU 姿态无效，运动已停止");
      raw_imu_yaw_.reset(); raw_imu_time_.reset();
      odometry_->invalidate_heading();
      return;
    }
    const double time = monotonic() - std::max(0.0, age);
    if (raw_imu_time_ && time <= *raw_imu_time_) return;
    raw_imu_yaw_ = std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
    raw_imu_time_ = time;
    calibration_->observe(*raw_imu_yaw_, time);
    if (!calibration_->active() && (!auto_imu_calibration_ || imu_yaw_calibrated_))
      odometry_->set_heading(imu_yaw_sign_ * *raw_imu_yaw_, time);
  }
  void publish_calibration() {
    const double tick = monotonic();
    const bool changed = !last_calibration_state_ || *last_calibration_state_ != calibration_->state();
    if (!changed && tick - last_calibration_publish_ < 0.1) return;
    last_calibration_publish_ = tick;
    auto m = message<msg::ImuCalibration>(ros::Time::now());
    m.state = static_cast<uint8_t>(calibration_->state());
    m.active = calibration_->active(); m.calibrated = imu_yaw_calibrated_;
    m.yaw_sign = static_cast<int8_t>(imu_yaw_sign_);
    m.positive_delta = calibration_->positive_delta(); m.negative_delta = calibration_->negative_delta();
    m.command_yaw_rate = calibration_->command(); m.message = calibration_->message();
    calibration_pub_.publish(m);
    if (changed) ROS_INFO( "IMU calibration: %s", m.message.c_str());
    last_calibration_state_ = calibration_->state();
  }
  void initialize_after_heartbeat(bool enabled) {
    if (!initialize_) return;
    if (!startup_wait_) {
      startup_wait_ = true;
      startup_deadline_ = monotonic() + service_timeout_;
      transmit(core::query_version(address_));
      if (!enabled) {
        transmit(core::enable(address_, true));
        return;
      }
    }
    if (!enabled) return;
    initialize_ = false;
    startup_wait_ = false;
    transmit(core::mode(address_, 2, false, false, false));
    transmit(core::feedback(address_, core::Command::RemoteFeedback, remote_feedback_));
    transmit(core::feedback(address_, core::Command::SensorFeedback, sensor_feedback_));
    transmit(
        core::feedback(address_, core::Command::DriverMotionFeedback, driver_motion_feedback_));
    transmit(
        core::feedback(address_, core::Command::DriverStatusFeedback, driver_status_feedback_));
    ROS_INFO(
                "Startup configuration sent once; no automatic mode locking or counter reset");
  }
  void publish_state(const ros::Time& stamp) {
    state_.header.stamp = stamp;
    state_.header.frame_id = base_frame_;
    state_.online =
        transport_ && heartbeat_time_ && monotonic() - *heartbeat_time_ < heartbeat_timeout_;
    state_pub_.publish(state_);
  }
  void receive(const core::Heartbeat& event, const ros::Time& stamp, double tick) {
    if (!event.enabled) motion_.fail("底盘失能，运动已停止");
    if (state_.heartbeat_received && state_.enabled && !event.enabled) commands_->stop();
    heartbeat_time_ = tick;
    state_.heartbeat_received = true;
    state_.heartbeat_stamp = stamp;
    state_.enabled = event.enabled;
    if (pending_ && pending_->request->command == CommandService::Request::ENABLE &&
        pending_->request->enabled == event.enabled)
      pending_->response.state_confirmed = true;
    initialize_after_heartbeat(event.enabled);
    publish_state(stamp);
  }
  void receive(const core::Status& event, const ros::Time& stamp, double tick) {
    if (event.mode != 2 || event.work_state != 0) motion_.fail("底盘模式或工作状态改变，运动已停止");
    if (state_.status_received && state_.mode != event.mode) commands_->stop();
    status_time_ = tick;
    state_.status_received = true;
    state_.status_stamp = stamp;
    state_.work_state = event.work_state;
    state_.mode = event.mode;
    state_.battery_voltage = event.voltage;
    state_.beep = event.beep;
    state_.flags_raw = event.flags_raw;
    if (pending_ && pending_->request->command == CommandService::Request::SET_MODE &&
        event.mode == pending_->request->mode && event.beep == pending_->request->beep)
      pending_->response.state_confirmed = true;
    publish_state(stamp);
  }
  void receive(const core::Version& event, const ros::Time& stamp, double) {
    auto m = message<msg::Version>(stamp);
    m.hardware = event.hardware;
    m.software = event.software;
    version_pub_.publish(m);
    if (pending_ && pending_->request->command == CommandService::Request::QUERY_VERSION) {
      pending_->response.acknowledged = true;
      pending_->response.state_confirmed = true;
      pending_->response.hardware_version = event.hardware;
      pending_->response.software_version = event.software;
    }
  }
  void receive(const core::Ack& event, const ros::Time& stamp, double) {
    auto m = message<msg::Acknowledgement>(stamp);
    m.command = static_cast<uint8_t>(event.command);
    ack_pub_.publish(m);
    if (event.command == core::Command::ZeroMotion) odometry_->rebase();
    if (pending_ && pending_->request->command == m.command) pending_->response.acknowledged = true;
  }
  void receive(const core::Motion& event, const ros::Time& stamp, double tick) {
    motion_time_ = tick;
    auto m = message<geometry_msgs::TwistStamped>(stamp);
    m.twist.linear.x = event.vx;
    m.twist.linear.y = event.vy;
    m.twist.angular.z = event.yaw_rate;
    velocity_pub_.publish(m);
    if (!wheel_odometry_ && odometry_->update(event, tick)) publish_odometry(stamp);
  }
  void receive(const core::Mileage& event, const ros::Time& stamp, double tick) {
    auto m = message<msg::WheelMileage>(stamp);
    m.left_mm = event.left_mm;
    m.right_mm = event.right_mm;
    m.secondary = event.secondary;
    mileage_pub_.publish(m);
    if (pending_ && pending_->request->command == CommandService::Request::ZERO_MOTION) return;
    if (wheel_odometry_ && odometry_->update(event, tick)) publish_odometry(stamp);
  }
  void receive(const core::Remote& event, const ros::Time& stamp, double) {
    auto m = message<msg::Remote>(stamp);
    std::copy(event.switches.begin(), event.switches.end(), m.switches.begin());
    std::copy(event.axes.begin(), event.axes.end(), m.axes.begin());
    remote_pub_.publish(m);
  }
  void receive(const core::Sensors& event, const ros::Time& stamp, double) {
    auto m = message<msg::Sensors>(stamp);
    m.bumper_mask = event.bumper_mask;
    std::copy(event.ultrasonic_raw.begin(), event.ultrasonic_raw.end(), m.ultrasonic_raw.begin());
    sensors_pub_.publish(m);
  }
  void receive(const core::DriverMotion& event, const ros::Time& stamp, double) {
    auto m = message<msg::DriverMotion>(stamp);
    m.channel = event.channel;
    m.driver_id = event.channel + 1;
    m.rpm = event.rpm;
    m.pulses = event.pulses;
    driver_motion_pub_.publish(m);
  }
  void receive(const core::DriverStatus& event, const ros::Time& stamp, double) {
    auto m = message<msg::DriverStatus>(stamp);
    m.channel = event.channel;
    m.driver_id = event.channel + 1;
    m.voltage = event.voltage;
    m.current = event.current;
    m.driver_temperature = event.driver_temperature;
    m.motor_temperature = event.motor_temperature;
    m.faults_raw = event.faults_raw;
    driver_status_pub_.publish(m);
  }
  void receive(const core::Faults& event, const ros::Time& stamp, double) {
    auto m = message<msg::Faults>(stamp);
    m.motor = event.motor;
    m.driver = event.driver;
    m.driver_offline = event.driver_offline;
    m.status_raw = event.status_raw;
    m.power = event.power;
    faults_pub_.publish(m);
  }
  void publish_odometry(const ros::Time& stamp) {
    odom_time_ = monotonic();
    const auto& pose = odometry_->pose();
    auto heading = message<msg::Heading>(stamp);
    heading.header.frame_id = odom_frame_;
    heading.yaw = pose.yaw; heading.yaw_rate = pose.yaw_rate;
    heading.imu_active = odometry_->heading_fresh(monotonic());
    heading.imu_age = odometry_->heading_age(monotonic());
    heading_pub_.publish(heading);
    auto m = message<nav_msgs::Odometry>(stamp);
    m.header.frame_id = odom_frame_;
    m.child_frame_id = base_frame_;
    m.pose.pose.position.x = pose.x;
    m.pose.pose.position.y = pose.y;
    m.pose.pose.orientation.z = std::sin(pose.yaw / 2);
    m.pose.pose.orientation.w = std::cos(pose.yaw / 2);
    m.twist.twist.linear.x = pose.vx;
    m.twist.twist.angular.z = pose.yaw_rate;
    m.pose.covariance[0] = m.pose.covariance[7] = pose_xy_;
    m.pose.covariance[14] = m.pose.covariance[21] = m.pose.covariance[28] = 1e6;
    m.pose.covariance[35] = pose_yaw_;
    m.twist.covariance[0] = twist_linear_;
    m.twist.covariance[35] = twist_angular_;
    m.twist.covariance[7] = m.twist.covariance[14] = m.twist.covariance[21] =
        m.twist.covariance[28] = 1e6;
    odom_pub_.publish(m);
    if (tf_) {
      auto transform = message<geometry_msgs::TransformStamped>(stamp);
      transform.header.frame_id = odom_frame_;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = pose.x;
      transform.transform.translation.y = pose.y;
      transform.transform.rotation = m.pose.pose.orientation;
      tf_->sendTransform(transform);
    }
  }
  bool motion_request(MotionService::Request& request, MotionService::Response& response) {
    std::lock_guard<std::mutex> guard(mutex_);
    execute_motion(request, response);
    return true;
  }
  bool reset_odometry(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    std::lock_guard<std::mutex> guard(mutex_);
    calibration_->cancel("Canceled by odometry reset");
    motion_.cancel("里程计重置，运动已取消");
    commands_->stop();
    if (!receive_only_) transmit(core::motion(address_, 0, 0, 0));
    odometry_->reset();
    response.success = true;
    response.message = "Local odometry reset; device counters unchanged";
    return true;
  }
  bool calibrate_imu(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (receive_only_ || !use_imu_heading_ || calibration_->pending()) {
      response.success = false;
      response.message = receive_only_ ? "receive_only is enabled; rotation calibration is prohibited" :
          !use_imu_heading_ ? "use_imu_heading is disabled" : "Calibration already pending or active";
      return true;
    }
    commands_->stop();
    transmit(core::motion(address_, 0, 0, 0));
    calibration_->request();
    motion_.cancel("IMU 标定已接管");
    response.success = true;
    response.message = "Calibration requested; observe imu_calibration for the measured result";
    publish_calibration();
    return true;
  }
  struct Pending {
    std::shared_ptr<CommandService::Request> request;
    CommandService::Response response;
    double deadline{};
    bool complete{};
  };
  bool command_request(CommandService::Request& input, CommandService::Response& response) {
    std::unique_lock<std::mutex> guard(mutex_);
    if (receive_only_) {
      response.message = "receive_only is enabled; all chassis CAN transmissions are prohibited";
      return true;
    }
    if (pending_) {
      response.message = "Another command awaits feedback; CAN ACKs have no transaction IDs";
      return true;
    }
    auto request = std::make_shared<CommandService::Request>(input);
    // An explicit user command supersedes startup intent. Drain previously queued ACKs first.
    initialize_ = false;
    startup_wait_ = false;
    poll_can();
    core::Frame frame;
    const auto command = static_cast<core::Command>(request->command);
    try {
      switch (command) {
        case core::Command::Reset:
          frame = core::reset(address_);
          break;
        case core::Command::Version:
          frame = core::query_version(address_);
          break;
        case core::Command::Enable:
          frame = core::enable(address_, request->enabled);
          break;
        case core::Command::ClearFaults:
          frame = core::clear_faults(address_);
          break;
        case core::Command::ZeroMotion:
          frame = core::zero_motion(address_);
          break;
        case core::Command::Mode:
          frame =
              core::mode(address_, request->mode, request->beep, request->brake, request->special);
          break;
        case core::Command::RemoteFeedback:
        case core::Command::SensorFeedback:
        case core::Command::DriverMotionFeedback:
        case core::Command::DriverStatusFeedback:
          frame = core::feedback(address_, command, request->enabled);
          break;
        default:
          throw std::invalid_argument("unsupported command; use cmd_vel for motion");
      }
      if (command == core::Command::Reset || command == core::Command::Mode ||
          command == core::Command::ZeroMotion ||
          (command == core::Command::Enable && !request->enabled)) {
        calibration_->cancel("Canceled by explicit chassis command");
        motion_.cancel("底盘操作已取消运动");
        commands_->stop();
        transmit(core::motion(address_, 0, 0, 0));
      }
      if (command == core::Command::ZeroMotion || command == core::Command::Reset)
        odometry_->rebase();
      response.sent = transmit(frame);
      if (!response.sent) {
        response.message = "SocketCAN send failed: " + io_error_;
        return true;
      }
      if (command == core::Command::Reset || command >= core::Command::RemoteFeedback) {
        response.message =
            command == core::Command::Reset
                ? "Reset frame sent once; reset/recovery behavior is unverified; no retry or "
                  "interface reconfiguration"
                : "Feedback switch sent; protocol has no ACK for this command; inspect telemetry";
        return true;
      }
      auto transaction = std::make_shared<Pending>();
      transaction->request = request;
      transaction->response = response;
      transaction->deadline = monotonic() + service_timeout_;
      pending_ = transaction;
      // wait_for unlocks the session mutex: the main queue keeps receiving CAN,
      // handling cmd_vel and issuing the 300 ms watchdog stop during this wait.
      pending_changed_.wait_for(guard, std::chrono::duration<double>(service_timeout_ + 0.1),
          [&] { return transaction->complete || shutting_down_ || !ros::ok(); });
      if (!transaction->complete) {
        transaction->response.message = "Command wait ended on timeout or node shutdown";
        if (pending_ == transaction) pending_.reset();
      }
      response = transaction->response;
    } catch (const std::exception& error) {
      response.message = error.what();

    }
    return true;
  }
  void check_pending() {
    if (!pending_) return;
    const auto command = pending_->request->command;
    const auto& r = pending_->response;
    const bool complete = command == CommandService::Request::ENABLE
                              ? r.acknowledged && r.state_confirmed
                          : command == CommandService::Request::SET_MODE ? r.state_confirmed
                                                                         : r.acknowledged;
    if (!complete && transport_ && monotonic() < pending_->deadline) return;
    auto done = pending_;
    pending_.reset();
    if (command == CommandService::Request::ZERO_MOTION) odometry_->rebase();
    if (!complete)
      done->response.message =
          "Feedback timeout or transport loss; inspect acknowledged/state_confirmed separately";
    else if (command == CommandService::Request::ENABLE)
      done->response.message = "A3 received and new B0 enabled state matched";
    else if (command == CommandService::Request::SET_MODE)
      done->response.message =
          "New B1 mode/beep matched; brake and special-function effects remain unverified";
    else if (command == CommandService::Request::QUERY_VERSION)
      done->response.message = "ASCII A2 version received";
    else
      done->response.message = "Command receipt acknowledged; physical effect is not proven by ACK";
    done->complete = true;
    pending_changed_.notify_all();
  }
  void health_tick() {
    const auto stamp = ros::Time::now();
    const double tick = monotonic();
    publish_state(stamp);
    diagnostic_msgs::DiagnosticArray diagnostics;
    diagnostics.header.stamp = stamp;
    diagnostic_msgs::DiagnosticStatus status;
    status.name = std::string(ros::this_node::getName()) + "/socketcan";
    status.hardware_id = interface_ + ":" + std::to_string(address_.base());
    status.level = diagnostic_msgs::DiagnosticStatus::OK;
    status.message = "Receiving vehicle feedback";
    if (!transport_ || !state_.online) {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      status.message = io_error_.empty() ? "Vehicle heartbeat missing or stale" : io_error_;
    } else if (state_.enabled && (!status_time_ || tick - *status_time_ > 0.5 || !motion_time_ ||
                                  tick - *motion_time_ > 0.2)) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "Enabled vehicle has missing/stale B1 or B2";
    } else if (state_.status_received && state_.work_state != 0) {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      status.message = "Vehicle reports a nonzero work_state; inspect raw status/fault telemetry";
    } else if (!state_.enabled) {
      status.message = "Online, disabled";
    }
    if (receive_only_) status.message += " [receive_only: chassis TX blocked]";
    auto value = [&status](const std::string& key, const auto& data) {
      diagnostic_msgs::KeyValue item;
      item.key = key;
      item.value = std::to_string(data);
      status.values.push_back(item);
    };
    value("heartbeat_age_s", heartbeat_time_ ? tick - *heartbeat_time_ : -1.0);
    value("status_age_s", status_time_ ? tick - *status_time_ : -1.0);
    value("velocity_age_s", motion_time_ ? tick - *motion_time_ : -1.0);
    value("rx_frames", rx_frames_);
    value("tx_frames", tx_frames_);
    value("receive_only", receive_only_);
    value("blocked_transmissions", blocked_transmissions_);
    value("malformed_frames", malformed_frames_);
    value("can_error_frames", error_frames_);
    value("rx_errors", rx_errors_);
    value("tx_errors", tx_errors_);
    value("socket_dropped_frames", transport_ ? transport_->dropped_frames() : 0);
    value("rejected_commands", rejected_commands_);
    value("command_active", commands_->active(tick));
    value("odometry_discontinuities", odometry_->discontinuities());
    value("imu_heading_active", odometry_->heading_fresh(tick));
    value("imu_heading_age_s", odometry_->heading_age(tick));
    value("imu_yaw_sign", imu_yaw_sign_);
    value("imu_yaw_calibrated", imu_yaw_calibrated_);
    value("imu_calibration_state", static_cast<int>(calibration_->state()));
    diagnostics.status.push_back(status);
    diagnostics_pub_.publish(diagnostics);
  }
  ros::NodeHandle nh_, private_{"~"}, command_nh_{"~"};
  ros::CallbackQueue command_queue_;
  ros::AsyncSpinner command_spinner_{2, &command_queue_};
  std::mutex mutex_;
  std::condition_variable pending_changed_;
  bool receive_only_{true}, shutting_down_{};
  double max_linear_speed_{}, max_angular_speed_{};
  uint64_t blocked_transmissions_{};
  core::Address address_;
  std::string interface_, odom_frame_, base_frame_, io_error_;
  bool initialize_{}, startup_wait_{}, disable_on_shutdown_{}, wheel_odometry_{};
  bool remote_feedback_{}, sensor_feedback_{}, driver_motion_feedback_{}, driver_status_feedback_{};
  double command_timeout_{}, service_timeout_{}, heartbeat_timeout_{}, next_connect_{},
      startup_deadline_{};
  double pose_xy_{}, pose_yaw_{}, twist_linear_{}, twist_angular_{};
  uint64_t rx_frames_{}, tx_frames_{}, malformed_frames_{}, error_frames_{}, rx_errors_{},
      tx_errors_{}, rejected_commands_{};
  std::unique_ptr<core::CanTransport> transport_;
  std::unique_ptr<core::CommandBuffer> commands_;
  core::MotionConfig motion_config_;
  core::MotionController motion_;
  uint64_t motion_id_{};
  core::MotionController::State last_motion_state_{core::MotionController::State::Idle};
  double last_motion_publish_{};
  std::optional<double> odom_time_;
  std::unique_ptr<core::Odometry> odometry_;
  std::unique_ptr<core::YawCalibration> calibration_;
  bool use_imu_heading_{}, imu_yaw_calibrated_{}, auto_imu_calibration_{};
  int imu_yaw_sign_{1};
  std::optional<double> raw_imu_yaw_, raw_imu_time_;
  std::optional<core::YawCalibration::State> last_calibration_state_;
  double last_calibration_publish_{};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::optional<double> heartbeat_time_, status_time_, motion_time_;
  std::shared_ptr<Pending> pending_;
  msg::State state_;
  ros::Publisher raw_pub_;
  ros::Publisher state_pub_;
  ros::Publisher version_pub_;
  ros::Publisher ack_pub_;
  ros::Publisher velocity_pub_;
  ros::Publisher mileage_pub_;
  ros::Publisher driver_motion_pub_;
  ros::Publisher driver_status_pub_;
  ros::Publisher remote_pub_;
  ros::Publisher sensors_pub_;
  ros::Publisher faults_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher heading_pub_;
  ros::Publisher calibration_pub_;
  ros::Publisher motion_pub_;
  ros::Publisher diagnostics_pub_;
  ros::Subscriber stamped_sub_;
  ros::Subscriber twist_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber keepalive_sub_;
  ros::ServiceServer motion_service_;
  ros::ServiceServer service_;
  ros::ServiceServer reset_odom_;
  ros::ServiceServer calibrate_imu_;
  ros::WallTimer poll_timer_, control_timer_, health_timer_;
};
}  // namespace vehicle_driver
int main(int argc, char** argv) {
  ros::init(argc, argv, "vehicle");
  try {
    vehicle_driver::VehicleNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Vehicle initialization failed: %s", error.what());
    return 1;
  }
  return 0;
}
