// SPDX-License-Identifier: Apache-2.0
#include <linux/can/error.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>
#include <string>
#include <vehicle_core/control.hpp>
#include <vehicle_core/odometry.hpp>
#include <vehicle_core/transport.hpp>
#include <vehicle_interfaces/msg/acknowledgement.hpp>
#include <vehicle_interfaces/msg/can_frame.hpp>
#include <vehicle_interfaces/msg/driver_motion.hpp>
#include <vehicle_interfaces/msg/driver_status.hpp>
#include <vehicle_interfaces/msg/faults.hpp>
#include <vehicle_interfaces/msg/remote.hpp>
#include <vehicle_interfaces/msg/sensors.hpp>
#include <vehicle_interfaces/msg/state.hpp>
#include <vehicle_interfaces/msg/version.hpp>
#include <vehicle_interfaces/msg/wheel_mileage.hpp>
#include <vehicle_interfaces/srv/command.hpp>

namespace vehicle_driver {
namespace core = vehicle_core;
namespace msg = vehicle_interfaces::msg;
using CommandService = vehicle_interfaces::srv::Command;
using Service = rclcpp::Service<CommandService>;
using Clock = std::chrono::steady_clock;
static double monotonic() {
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}
// All callbacks share the default mutually-exclusive group, including in a
// multithreaded component container. No global state, worker threads or blocking ACK waits.
class VehicleNode final : public rclcpp::Node {
 public:
  explicit VehicleNode(const rclcpp::NodeOptions& options) : Node("vehicle", options) {
    interface_ = parameter<std::string>("can_interface", "can0");
    const int type = parameter<int>("device_type", 0xE1);
    const int node = parameter<int>("device_node", 1);
    if (type < 0 || type > 255 || node < 0 || node > 255)
      throw std::invalid_argument("CAN device_type and device_node must be bytes");
    address_ = {static_cast<uint8_t>(type), static_cast<uint8_t>(node)};
    initialize_ = parameter<bool>("auto_initialize", true);
    disable_on_shutdown_ = parameter<bool>("disable_on_shutdown", true);
    const bool stamped = parameter<bool>("use_stamped_cmd_vel", true);
    const double rate = positive("command_rate", 50.0);
    if (rate > 500.0) throw std::invalid_argument("command_rate must be <= 500 Hz");
    command_timeout_ = positive("command_timeout", 0.30);
    commands_ = std::make_unique<core::CommandBuffer>(
        command_timeout_, positive("max_linear_speed", 1.0), positive("max_angular_speed", 2.0));
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
    odom_frame_ = parameter<std::string>("odom_frame", "odom");
    base_frame_ = parameter<std::string>("base_frame", "base_link");
    if (odom_frame_.empty() || base_frame_.empty() || odom_frame_ == base_frame_)
      throw std::invalid_argument("odom_frame and base_frame must be nonempty and distinct");
    pose_xy_ = positive("pose_variance_xy", 0.05);
    pose_yaw_ = positive("pose_variance_yaw", 0.10);
    twist_linear_ = positive("twist_variance_linear", 0.02);
    twist_angular_ = positive("twist_variance_angular", 0.05);
    if (parameter<bool>("publish_tf", true))
      tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    const auto telemetry_qos = rclcpp::SensorDataQoS();
    raw_pub_ = create_publisher<msg::CanFrame>("~/can_rx", rclcpp::SensorDataQoS().keep_last(128));
    state_pub_ = create_publisher<msg::State>("~/state", rclcpp::QoS(1).transient_local());
    version_pub_ = create_publisher<msg::Version>("~/version", rclcpp::QoS(1).transient_local());
    ack_pub_ = create_publisher<msg::Acknowledgement>("~/acknowledgements", rclcpp::QoS(10));
    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("~/velocity", telemetry_qos);
    mileage_pub_ = create_publisher<msg::WheelMileage>("~/wheel_mileage", telemetry_qos);
    driver_motion_pub_ = create_publisher<msg::DriverMotion>("~/driver_motion", telemetry_qos);
    driver_status_pub_ = create_publisher<msg::DriverStatus>("~/driver_status", telemetry_qos);
    remote_pub_ = create_publisher<msg::Remote>("~/remote", telemetry_qos);
    sensors_pub_ = create_publisher<msg::Sensors>("~/sensors", telemetry_qos);
    faults_pub_ = create_publisher<msg::Faults>("~/faults", rclcpp::QoS(10));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
    if (stamped) {
      stamped_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
          "cmd_vel", rclcpp::QoS(1), [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
            try {
              if (m->header.stamp.sec != 0 || m->header.stamp.nanosec != 0) {
                const double age =
                    (now() - rclcpp::Time(m->header.stamp, get_clock()->get_clock_type()))
                        .seconds();
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
      twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
          "cmd_vel", rclcpp::QoS(1),
          [this](geometry_msgs::msg::Twist::ConstSharedPtr m) { accept_command(*m); });
    }
    service_ = create_service<CommandService>(
        "~/command", [this](Service::SharedPtr service, std::shared_ptr<rmw_request_id_t> id,
                            CommandService::Request::SharedPtr request) {
          command_request(service, id, request);
        });
    reset_odom_ = create_service<std_srvs::srv::Trigger>(
        "~/reset_odometry", [this](std_srvs::srv::Trigger::Request::SharedPtr,
                                   std_srvs::srv::Trigger::Response::SharedPtr response) {
          odometry_->reset();
          response->success = true;
          response->message = "Local odometry reset; device counters unchanged";
        });
    connect();
    poll_timer_ = create_wall_timer(std::chrono::milliseconds(2), [this] { poll_can(); });
    control_timer_ =
        create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] { control_tick(); });
    health_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { health_tick(); });
    RCLCPP_INFO(get_logger(), "SocketCAN %s, vehicle ID 0x%08X, odometry=%s, cmd_vel=%s",
                interface_.c_str(), address_.base(), odom_source.c_str(),
                stamped ? "TwistStamped" : "Twist");
  }
  ~VehicleNode() override {
    if (transport_) {
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
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;  // Configuration changes take effect by relaunching.
    return declare_parameter<T>(name, value, descriptor);
  }
  double positive(const std::string& name, double value) {
    const auto result = parameter<double>(name, value);
    if (!std::isfinite(result) || result <= 0)
      throw std::invalid_argument(name + " must be positive and finite");
    return result;
  }
  template <class T>
  T message(const rclcpp::Time& stamp) const {
    T m;
    m.header.stamp = stamp;
    m.header.frame_id = base_frame_;
    return m;
  }
  void accept_command(const geometry_msgs::msg::Twist& twist) {
    if (!std::isfinite(twist.linear.y) || !std::isfinite(twist.linear.z) ||
        !std::isfinite(twist.angular.x) || !std::isfinite(twist.angular.y) ||
        !commands_->set(twist.linear.x, twist.angular.z, monotonic())) {
      ++rejected_commands_;
      commands_->stop();
      return;
    }
    if (twist.linear.y != 0 || twist.linear.z != 0 || twist.angular.x != 0 || twist.angular.y != 0)
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Differential chassis uses linear.x and angular.z; other axes are ignored");
  }
  void connect() {
    next_connect_ = monotonic() + 1.0;
    try {
      transport_ = std::make_unique<core::SocketCan>(interface_, address_);
      io_error_.clear();
      odometry_->rebase();
      RCLCPP_INFO(get_logger(), "Opened %s", interface_.c_str());
    } catch (const std::exception& error) {
      io_error_ = error.what();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", io_error_.c_str());
    }
  }
  void disconnect(const std::string& error) {
    io_error_ = error;
    transport_.reset();
    next_connect_ = monotonic() + 1.0;
    commands_->stop();
    odometry_->rebase();
    // Never replay a command cached before a transport failure.
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.c_str());
  }
  bool transmit(const core::Frame& frame) {
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
      // Bound each callback so traffic cannot starve DDS callbacks or the watchdog.
      for (size_t i = 0; i < 256 && transport_; ++i) {
        auto frame = transport_->receive();
        if (!frame) break;
        const auto stamp = now();
        const double tick = monotonic();
        ++rx_frames_;
        auto raw = message<msg::CanFrame>(stamp);
        raw.id = frame->id;
        raw.length = frame->size;
        raw.data = frame->data;
        raw.extended = frame->extended;
        raw.remote = frame->remote;
        raw.error = frame->error;
        raw_pub_->publish(raw);
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
    if (auto command = commands_->sample(tick))
      transmit(core::motion(address_, command->vx, 0, command->yaw_rate));
    if (startup_wait_ && tick > startup_deadline_) {
      startup_wait_ = false;
      initialize_ = false;
      RCLCPP_WARN(get_logger(),
                  "Startup enable not observed; inspect state and use command service");
    }
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
    RCLCPP_INFO(get_logger(),
                "Startup configuration sent once; no automatic mode locking or counter reset");
  }
  void publish_state(const rclcpp::Time& stamp) {
    state_.header.stamp = stamp;
    state_.header.frame_id = base_frame_;
    state_.online =
        transport_ && heartbeat_time_ && monotonic() - *heartbeat_time_ < heartbeat_timeout_;
    state_pub_->publish(state_);
  }
  void receive(const core::Heartbeat& event, const rclcpp::Time& stamp, double tick) {
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
  void receive(const core::Status& event, const rclcpp::Time& stamp, double tick) {
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
  void receive(const core::Version& event, const rclcpp::Time& stamp, double) {
    auto m = message<msg::Version>(stamp);
    m.hardware = event.hardware;
    m.software = event.software;
    version_pub_->publish(m);
    if (pending_ && pending_->request->command == CommandService::Request::QUERY_VERSION) {
      pending_->response.acknowledged = true;
      pending_->response.state_confirmed = true;
      pending_->response.hardware_version = event.hardware;
      pending_->response.software_version = event.software;
    }
  }
  void receive(const core::Ack& event, const rclcpp::Time& stamp, double) {
    auto m = message<msg::Acknowledgement>(stamp);
    m.command = static_cast<uint8_t>(event.command);
    ack_pub_->publish(m);
    if (event.command == core::Command::ZeroMotion) odometry_->rebase();
    if (pending_ && pending_->request->command == m.command) pending_->response.acknowledged = true;
  }
  void receive(const core::Motion& event, const rclcpp::Time& stamp, double tick) {
    motion_time_ = tick;
    auto m = message<geometry_msgs::msg::TwistStamped>(stamp);
    m.twist.linear.x = event.vx;
    m.twist.linear.y = event.vy;
    m.twist.angular.z = event.yaw_rate;
    velocity_pub_->publish(m);
    if (!wheel_odometry_ && odometry_->update(event, tick)) publish_odometry(stamp);
  }
  void receive(const core::Mileage& event, const rclcpp::Time& stamp, double tick) {
    auto m = message<msg::WheelMileage>(stamp);
    m.left_mm = event.left_mm;
    m.right_mm = event.right_mm;
    m.secondary = event.secondary;
    mileage_pub_->publish(m);
    if (pending_ && pending_->request->command == CommandService::Request::ZERO_MOTION) return;
    if (wheel_odometry_ && odometry_->update(event, tick)) publish_odometry(stamp);
  }
  void receive(const core::Remote& event, const rclcpp::Time& stamp, double) {
    auto m = message<msg::Remote>(stamp);
    m.switches = event.switches;
    m.axes = event.axes;
    remote_pub_->publish(m);
  }
  void receive(const core::Sensors& event, const rclcpp::Time& stamp, double) {
    auto m = message<msg::Sensors>(stamp);
    m.bumper_mask = event.bumper_mask;
    m.ultrasonic_raw = event.ultrasonic_raw;
    sensors_pub_->publish(m);
  }
  void receive(const core::DriverMotion& event, const rclcpp::Time& stamp, double) {
    auto m = message<msg::DriverMotion>(stamp);
    m.channel = event.channel;
    m.driver_id = event.channel + 1;
    m.rpm = event.rpm;
    m.pulses = event.pulses;
    driver_motion_pub_->publish(m);
  }
  void receive(const core::DriverStatus& event, const rclcpp::Time& stamp, double) {
    auto m = message<msg::DriverStatus>(stamp);
    m.channel = event.channel;
    m.driver_id = event.channel + 1;
    m.voltage = event.voltage;
    m.current = event.current;
    m.driver_temperature = event.driver_temperature;
    m.motor_temperature = event.motor_temperature;
    m.faults_raw = event.faults_raw;
    driver_status_pub_->publish(m);
  }
  void receive(const core::Faults& event, const rclcpp::Time& stamp, double) {
    auto m = message<msg::Faults>(stamp);
    m.motor = event.motor;
    m.driver = event.driver;
    m.driver_offline = event.driver_offline;
    m.status_raw = event.status_raw;
    m.power = event.power;
    faults_pub_->publish(m);
  }
  void publish_odometry(const rclcpp::Time& stamp) {
    const auto& pose = odometry_->pose();
    auto m = message<nav_msgs::msg::Odometry>(stamp);
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
    odom_pub_->publish(m);
    if (tf_) {
      auto transform = message<geometry_msgs::msg::TransformStamped>(stamp);
      transform.header.frame_id = odom_frame_;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = pose.x;
      transform.transform.translation.y = pose.y;
      transform.transform.rotation = m.pose.pose.orientation;
      tf_->sendTransform(transform);
    }
  }
  struct Pending {
    Service::SharedPtr service;
    std::shared_ptr<rmw_request_id_t> id;
    CommandService::Request::SharedPtr request;
    CommandService::Response response;
    double deadline;
  };
  void command_request(Service::SharedPtr service, std::shared_ptr<rmw_request_id_t> id,
                       CommandService::Request::SharedPtr request) {
    CommandService::Response response;
    if (pending_) {
      response.message = "Another command awaits feedback; CAN ACKs have no transaction IDs";
      service->send_response(*id, response);
      return;
    }
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
        commands_->stop();
        transmit(core::motion(address_, 0, 0, 0));
      }
      if (command == core::Command::ZeroMotion || command == core::Command::Reset)
        odometry_->rebase();
      response.sent = transmit(frame);
      if (!response.sent) {
        response.message = "SocketCAN send failed: " + io_error_;
        service->send_response(*id, response);
        return;
      }
      if (command == core::Command::Reset || command >= core::Command::RemoteFeedback) {
        response.message =
            command == core::Command::Reset
                ? "Reset frame sent once; reset/recovery behavior is unverified; no retry or "
                  "interface reconfiguration"
                : "Feedback switch sent; protocol has no ACK for this command; inspect telemetry";
        service->send_response(*id, response);
        return;
      }
      pending_ = Pending{service, id, request, response, monotonic() + service_timeout_};
    } catch (const std::exception& error) {
      response.message = error.what();
      service->send_response(*id, response);
    }
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
    auto done = std::move(*pending_);
    pending_.reset();
    if (command == CommandService::Request::ZERO_MOTION) odometry_->rebase();
    if (!complete)
      done.response.message =
          "Feedback timeout or transport loss; inspect acknowledged/state_confirmed separately";
    else if (command == CommandService::Request::ENABLE)
      done.response.message = "A3 received and new B0 enabled state matched";
    else if (command == CommandService::Request::SET_MODE)
      done.response.message =
          "New B1 mode/beep matched; brake and special-function effects remain unverified";
    else if (command == CommandService::Request::QUERY_VERSION)
      done.response.message = "ASCII A2 version received";
    else
      done.response.message = "Command receipt acknowledged; physical effect is not proven by ACK";
    done.service->send_response(*done.id, done.response);
  }
  void health_tick() {
    const auto stamp = now();
    const double tick = monotonic();
    publish_state(stamp);
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = std::string(get_fully_qualified_name()) + "/socketcan";
    status.hardware_id = interface_ + ":" + std::to_string(address_.base());
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Receiving vehicle feedback";
    if (!transport_ || !state_.online) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = io_error_.empty() ? "Vehicle heartbeat missing or stale" : io_error_;
    } else if (state_.enabled && (!status_time_ || tick - *status_time_ > 0.5 || !motion_time_ ||
                                  tick - *motion_time_ > 0.2)) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "Enabled vehicle has missing/stale B1 or B2";
    } else if (state_.status_received && state_.work_state != 0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "Vehicle reports a nonzero work_state; inspect raw status/fault telemetry";
    } else if (!state_.enabled) {
      status.message = "Online, disabled";
    }
    auto value = [&status](const std::string& key, const auto& data) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = std::to_string(data);
      status.values.push_back(item);
    };
    value("heartbeat_age_s", heartbeat_time_ ? tick - *heartbeat_time_ : -1.0);
    value("status_age_s", status_time_ ? tick - *status_time_ : -1.0);
    value("velocity_age_s", motion_time_ ? tick - *motion_time_ : -1.0);
    value("rx_frames", rx_frames_);
    value("tx_frames", tx_frames_);
    value("malformed_frames", malformed_frames_);
    value("can_error_frames", error_frames_);
    value("rx_errors", rx_errors_);
    value("tx_errors", tx_errors_);
    value("socket_dropped_frames", transport_ ? transport_->dropped_frames() : 0);
    value("rejected_commands", rejected_commands_);
    value("command_active", commands_->active(tick));
    value("odometry_discontinuities", odometry_->discontinuities());
    diagnostics.status.push_back(status);
    diagnostics_pub_->publish(diagnostics);
  }
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
  std::unique_ptr<core::Odometry> odometry_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::optional<double> heartbeat_time_, status_time_, motion_time_;
  std::optional<Pending> pending_;
  msg::State state_;
  rclcpp::Publisher<msg::CanFrame>::SharedPtr raw_pub_;
  rclcpp::Publisher<msg::State>::SharedPtr state_pub_;
  rclcpp::Publisher<msg::Version>::SharedPtr version_pub_;
  rclcpp::Publisher<msg::Acknowledgement>::SharedPtr ack_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<msg::WheelMileage>::SharedPtr mileage_pub_;
  rclcpp::Publisher<msg::DriverMotion>::SharedPtr driver_motion_pub_;
  rclcpp::Publisher<msg::DriverStatus>::SharedPtr driver_status_pub_;
  rclcpp::Publisher<msg::Remote>::SharedPtr remote_pub_;
  rclcpp::Publisher<msg::Sensors>::SharedPtr sensors_pub_;
  rclcpp::Publisher<msg::Faults>::SharedPtr faults_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  Service::SharedPtr service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_odom_;
  rclcpp::TimerBase::SharedPtr poll_timer_, control_timer_, health_timer_;
};
}  // namespace vehicle_driver
RCLCPP_COMPONENTS_REGISTER_NODE(vehicle_driver::VehicleNode)
