// SPDX-License-Identifier: Apache-2.0
#include <linux/can/error.h>
#include <algorithm>
#include <ros/ros.h>
#include <chrono>
#include <cmath>
#include <memory>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Trigger.h>
#include <vehicle_core/imu.hpp>
#include <vehicle_interfaces/ImuStatus.h>

namespace vehicle_driver {
namespace core = vehicle_core;
static double imu_clock() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
class ImuNode final {
 public:
  ImuNode() {
    interface_ = parameter<std::string>("imu_can_interface", "can0");
    protocol_ = parameter<std::string>("imu_protocol", "auto");
    if (protocol_ != "auto" && protocol_ != "classic" && protocol_ != "legacy_fd")
      throw std::invalid_argument("imu_protocol must be auto, classic or legacy_fd");
    const int node = parameter<int>("imu_node_id", 31);
    if (node < 1 || node > 127 || node == 100 || node == 101)
      throw std::invalid_argument("imu_node_id must be 1..127 excluding 100,101");
    node_ = static_cast<uint8_t>(node);
    auto_start_ = parameter<bool>("imu_auto_start", true);
    frame_ = parameter<std::string>("base_frame", "base_link");
    const auto rpy = parameter<std::vector<double>>("imu_mount_rpy", {0.0, 0.0, 0.0});
    if (rpy.size() != 3 || !std::all_of(rpy.begin(), rpy.end(), [](double v) { return std::isfinite(v); }))
      throw std::invalid_argument("imu_mount_rpy requires three finite radians");
    mounting_ = core::mounting_rotation(rpy[0], rpy[1], rpy[2]);
    orientation_variance_ = positive("imu_orientation_variance", 0.01);
    gyro_variance_ = positive("imu_gyro_variance", 0.0004);
    accel_variance_ = positive("imu_accel_variance", 0.04);
    data_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 10);
    raw_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/raw", 5);
    mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("imu/magnetic_field", 5);
    temp_pub_ = nh_.advertise<sensor_msgs::Temperature>("imu/temperature", 5);
    status_pub_ = nh_.advertise<vehicle_interfaces::ImuStatus>("imu/status", 10);
    zero_orientation_ = private_.advertiseService("zero_orientation", &ImuNode::zero_orientation, this);
    connect();
    poll_ = nh_.createWallTimer(ros::WallDuration(0.002), [this](const ros::WallTimerEvent&) { poll(); });
    health_ = nh_.createWallTimer(ros::WallDuration(0.1), [this](const ros::WallTimerEvent&) { status(); });
    ROS_INFO("HPM IMU %s node %u, protocol %s", interface_.c_str(), node_, protocol_.c_str());
  }
 private:
  template <class T> T parameter(const std::string& name, const T& value) {
    T result;
    private_.param<T>(name, result, value);
    return result;
  }
  bool zero_orientation(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    if (!orientation_valid(imu_clock())) {
      response.success = false;
      response.message = "No fresh valid IMU orientation; zero reference unchanged";
      return true;
    }
    zero_reference_ = mounted_orientation();
    // TCPROS preserves sample order: invalidate the downstream yaw baseline first.
    publish_invalid_orientation();
    publish_orientation();
    response.success = true;
    response.message = "IMU roll, pitch and yaw zeroed at the current orientation";
    return true;
  }

  double positive(const std::string& name, double value) {
    const double result = parameter<double>(name, value);
    if (!std::isfinite(result) || result <= 0) throw std::invalid_argument(name + " must be positive");
    return result;
  }
  template <class T> T message() {
    T m; m.header.stamp = ros::Time::now(); m.header.frame_id = frame_; return m;
  }
  static void vector(geometry_msgs::Vector3& dst, const core::Vector3& src) {
    dst.x = src[0]; dst.y = src[1]; dst.z = src[2];
  }
  static bool fresh(double stamp, double time) {
    return stamp > 0 && time >= stamp && time - stamp < 0.25;
  }
  bool use_protocol(const std::string& protocol) {
    if (active_protocol_.empty()) {
      active_protocol_ = protocol;
      ROS_INFO("Detected IMU protocol: %s", protocol.c_str());
    }
    return active_protocol_ == protocol;
  }
  bool measurement_valid(double time) const {
    if (active_protocol_ == "classic")
      return measurement_ && fresh(measurement_time_, time) && fresh(gyro_time_, time);
    return measurement_ && !(measurement_->faults & 1U) && measurement_counter_.fresh(time, 0.25);
  }
  bool orientation_valid(double time) const {
    if (active_protocol_ == "classic")
      return orientation_ && orientation_->valid && fresh(orientation_time_, time);
    return orientation_ && orientation_->valid && orientation_counter_.fresh(time, 0.25) &&
           time - orientation_time_ + orientation_->age_ms / 1000.0 < 0.25;
  }
  core::Quaternion mounted_orientation() const {
    return core::multiply(orientation_->quaternion, core::conjugate(mounting_));
  }
  void publish_invalid_orientation() {
    auto m = message<sensor_msgs::Imu>();
    m.orientation_covariance[0] = m.angular_velocity_covariance[0] = m.linear_acceleration_covariance[0] = -1;
    data_pub_.publish(m);
  }
  void publish_orientation() {
    auto m = message<sensor_msgs::Imu>();
    // Zeroing must not make the stored sample appear newer than it is.
    m.header.stamp = orientation_stamp_;
    const auto q = core::multiply(core::conjugate(zero_reference_), mounted_orientation());
    m.orientation.w = q[0]; m.orientation.x = q[1]; m.orientation.y = q[2]; m.orientation.z = q[3];
    for (size_t axis : {0U, 4U, 8U}) m.orientation_covariance[axis] = orientation_variance_;
    fill_measurement(m, imu_clock());
    data_pub_.publish(m);
  }
  void invalidate() {
    measurement_.reset(); orientation_.reset();
    measurement_time_ = orientation_time_ = gyro_time_ = mag_time_ = last_data_time_ = 0;
    measurement_counter_.reset(); orientation_counter_.reset();
    zero_reference_ = {1, 0, 0, 0};
    publish_invalid_orientation();
  }
  void connect() {
    next_connect_ = imu_clock() + 1;
    try {
      socket_ = std::make_unique<core::ImuSocketCan>(interface_, node_);
      error_.clear(); heartbeat_.reset(); nmt_ = 0; next_start_ = 0;
      active_protocol_ = protocol_ == "auto" ? "" : protocol_; invalidate();
    } catch (const std::exception& e) {
      error_ = e.what();
      ROS_WARN_THROTTLE(5.0, "%s", error_.c_str());
    }
  }
  void failure(const std::string& error) {
    error_ = error; socket_.reset(); heartbeat_.reset(); next_connect_ = imu_clock() + 1;
    invalidate();
    ROS_WARN_THROTTLE(5.0, "%s", error_.c_str());
  }
  void fill_measurement(sensor_msgs::Imu& m, double time) {
    if (active_protocol_ == "classic") {
      const bool accel = measurement_ && fresh(measurement_time_, time);
      const bool gyro = measurement_ && fresh(gyro_time_, time);
      if (accel) vector(m.linear_acceleration, core::rotate(mounting_, measurement_->acceleration));
      if (gyro) vector(m.angular_velocity, core::rotate(mounting_, measurement_->angular_velocity));
      for (size_t i : {0U, 4U, 8U}) {
        if (accel) m.linear_acceleration_covariance[i] = accel_variance_;
        if (gyro) m.angular_velocity_covariance[i] = gyro_variance_;
      }
      if (!accel) m.linear_acceleration_covariance[0] = -1;
      if (!gyro) m.angular_velocity_covariance[0] = -1;
      return;
    }
    if (!measurement_valid(time)) {
      m.angular_velocity_covariance[0] = m.linear_acceleration_covariance[0] = -1; return;
    }
    vector(m.angular_velocity, core::rotate(mounting_, measurement_->angular_velocity));
    vector(m.linear_acceleration, core::rotate(mounting_, measurement_->acceleration));
    for (size_t i : {0U, 4U, 8U}) {
      m.angular_velocity_covariance[i] = gyro_variance_;
      m.linear_acceleration_covariance[i] = accel_variance_;
    }
  }
  void receive_classic(const core::ImuClassicPdo& pdo, double time) {
    last_data_time_ = time;
    if (pdo.pdo == 2) {
      ++received_orientations_;
      core::ImuOrientation orientation;
      orientation.quaternion = pdo.quaternion;
      orientation.valid = pdo.orientation_valid;
      orientation_ = orientation;
      orientation_time_ = time;
      orientation_stamp_ = ros::Time::now();
      if (orientation.valid) publish_orientation();
      else publish_invalid_orientation();
      return;
    }
    if (!measurement_) measurement_ = core::ImuMeasurement{};
    if (pdo.pdo == 4) {
      measurement_->magnetic_field = pdo.vector;
      mag_time_ = time;
      auto mag = message<sensor_msgs::MagneticField>();
      vector(mag.magnetic_field, core::rotate(mounting_, pdo.vector));
      mag_pub_.publish(mag);
      return;
    }
    ++received_measurements_;
    if (pdo.pdo == 1) {
      measurement_->acceleration = pdo.vector;
      measurement_->temperature = pdo.temperature;
      measurement_time_ = time;
      auto temp = message<sensor_msgs::Temperature>();
      temp.temperature = pdo.temperature;
      temp_pub_.publish(temp);
    } else {
      measurement_->angular_velocity = pdo.vector;
      gyro_time_ = time;
    }
    auto raw = message<sensor_msgs::Imu>();
    raw.orientation_covariance[0] = -1;
    fill_measurement(raw, time);
    raw_pub_.publish(raw);
  }
  void poll() {
    if (!socket_) { if (imu_clock() >= next_connect_) connect(); return; }
    try {
      for (size_t i = 0; i < 256; ++i) {
        const auto f = socket_->receive();
        if (!f) break;
        ++rx_;
        const double time = imu_clock();
        if (f->error) {
          ++errors_;
          if (f->id & CAN_ERR_BUSOFF) { failure("IMU CAN bus-off"); return; }
          continue;
        }
        if (f->id == 0x700U + node_ && !f->fd && !f->extended && !f->remote && f->size == 1) {
          nmt_ = f->data[0]; heartbeat_ = time;
          if (nmt_ != 5) invalidate();
          if (auto_start_ && (nmt_ == 0 || nmt_ == 127) && time >= next_start_) {
            socket_->start(node_); next_start_ = time + 1;
          }
        } else if (auto pdo = core::decode_imu_classic(*f, node_); pdo && use_protocol("classic")) {
          receive_classic(*pdo, time);
        } else if (auto value = core::decode_imu_measurement(*f, node_); value && use_protocol("legacy_fd")) {
          last_data_time_ = time; ++received_measurements_;
          // Retain faults even if a stalled sensor keeps repeating its sample count.
          measurement_ = value;
          if (!measurement_counter_.update(value->counter, time)) continue;
          measurement_time_ = time;
          auto m = message<sensor_msgs::Imu>();
          m.orientation_covariance[0] = -1; fill_measurement(m, time); raw_pub_.publish(m);
          if ((value->status & 2U) && !(value->faults & 2U)) {
            mag_time_ = time;
            auto mag = message<sensor_msgs::MagneticField>();
            vector(mag.magnetic_field, core::rotate(mounting_, value->magnetic_field)); mag_pub_.publish(mag);
          }
          if (measurement_valid(time)) {
            auto temp = message<sensor_msgs::Temperature>();
            temp.temperature = value->temperature; temp_pub_.publish(temp);
          }
        } else if (auto orientation = core::decode_imu_orientation(*f, node_); orientation && use_protocol("legacy_fd")) {
          last_data_time_ = time; ++received_orientations_;
          orientation_ = orientation;
          if (!orientation->valid) {
            auto m = message<sensor_msgs::Imu>();
            m.orientation_covariance[0] = -1; fill_measurement(m, time); data_pub_.publish(m);
            continue;
          }
          if (!orientation_counter_.update(orientation->counter, time)) continue;
          if (orientation_counter_.restarted()) {
            zero_reference_ = {1, 0, 0, 0};
            publish_invalid_orientation();
            // Reliable ordered delivery rebases the receiver before the next valid sample.
            orientation_->valid = false; continue;
          }
          orientation_time_ = time;
          orientation_stamp_ = ros::Time::now() - ros::Duration(orientation->age_ms / 1000.0);
          publish_orientation();
        } else { ++malformed_; }
      }
    } catch (const std::exception& e) { failure(e.what()); }
  }
  void status() {
    const double time = imu_clock();
    auto m = message<vehicle_interfaces::ImuStatus>();
    m.protocol = active_protocol_.empty() ? "waiting" : active_protocol_;
    m.device_metadata_available = active_protocol_ == "legacy_fd";
    m.heartbeat_valid = heartbeat_ && time - *heartbeat_ < 1.5;
    m.online = socket_ && (m.heartbeat_valid || fresh(last_data_time_, time));
    m.received_measurements = received_measurements_; m.received_orientations = received_orientations_;
    m.acceleration_valid = active_protocol_ == "classic" ? fresh(measurement_time_, time) : measurement_valid(time);
    m.angular_velocity_valid = active_protocol_ == "classic" ? fresh(gyro_time_, time) : measurement_valid(time);
    m.magnetic_field_valid = fresh(mag_time_, time) &&
        (active_protocol_ == "classic" || (measurement_ && !(measurement_->faults & 2U)));
    m.measurement_valid = measurement_valid(time); m.orientation_valid = orientation_valid(time);
    m.nmt_state = nmt_; m.rx_frames = rx_; m.malformed_frames = malformed_; m.error_frames = errors_;
    m.dropped_frames = socket_ ? socket_->dropped_frames() : 0;
    m.measurement_age = measurement_time_ > 0 ? time - measurement_time_ : -1;
    if (active_protocol_ == "classic")
      m.measurement_age = measurement_time_ > 0 && gyro_time_ > 0 ?
          time - std::min(measurement_time_, gyro_time_) : -1;
    m.orientation_age = orientation_ && orientation_time_ > 0 ?
        time - orientation_time_ + orientation_->age_ms / 1000.0 : -1;
    if (measurement_) {
      m.measurement_status = measurement_->status; m.sample_count = measurement_->counter; m.faults = measurement_->faults;
    }
    if (orientation_) {
      m.orientation_status = orientation_->status; m.quaternion_count = orientation_->counter;
      m.orientation_device_age_ms = orientation_->age_ms;
    }
    m.message = !error_.empty() ? error_ : m.orientation_valid ? "IMU relative heading available" : "Waiting for fresh IMU orientation";
    if (active_protocol_ == "classic")
      m.message += "; Classic CAN: host reception freshness; device counters/faults/age unavailable";
    status_pub_.publish(m);
  }
  ros::NodeHandle nh_, private_{"~"};
  std::string interface_, frame_, error_, protocol_, active_protocol_;
  uint8_t node_{}, nmt_{};
  bool auto_start_{};
  core::Quaternion mounting_{};
  core::Quaternion zero_reference_{1, 0, 0, 0};
  ros::Time orientation_stamp_{0};
  double next_connect_{}, next_start_{}, measurement_time_{}, orientation_time_{};
  double gyro_time_{}, mag_time_{}, last_data_time_{};
  double orientation_variance_{}, gyro_variance_{}, accel_variance_{};
  uint64_t rx_{}, malformed_{}, errors_{}, received_measurements_{}, received_orientations_{};
  std::optional<double> heartbeat_;
  std::optional<core::ImuMeasurement> measurement_;
  std::optional<core::ImuOrientation> orientation_;
  core::ImuCounter measurement_counter_, orientation_counter_;
  std::unique_ptr<core::ImuSocketCan> socket_;
  ros::Publisher data_pub_, raw_pub_;
  ros::Publisher mag_pub_;
  ros::Publisher temp_pub_;
  ros::Publisher status_pub_;
  ros::ServiceServer zero_orientation_;
  ros::WallTimer poll_, health_;
};
}  // namespace vehicle_driver
int main(int argc, char** argv) {
  ros::init(argc, argv, "imu");
  try {
    vehicle_driver::ImuNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("IMU initialization failed: %s", error.what());
    return 1;
  }
  return 0;
}
