# SPDX-License-Identifier: Apache-2.0
"""Native rospy transport; Qt and service waits remain in separate threads."""
from concurrent.futures import Future
import math
from queue import Empty, Full, Queue
from threading import Thread
import time

from PyQt5.QtCore import QThread, pyqtSignal
import rospy
from rosgraph.names import resolve_name
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import UInt64
from vehicle_interfaces.msg import (Acknowledgement, CanFrame, DriverMotion, DriverStatus,
    Faults, Heading, ImuCalibration, ImuStatus, MotionStatus, State, Version, WheelMileage)
from vehicle_interfaces.srv import Command, CommandRequest, ExecuteMotion, ExecuteMotionRequest


class ServiceClient:
    def __init__(self, name, service_type):
        self.name = name
        self.proxy = rospy.ServiceProxy(name, service_type)

    def service_is_ready(self):
        try:
            rospy.wait_for_service(self.name, timeout=0.05)
            return True
        except rospy.ROSException:
            return False

    def call_async(self, payload):
        future = Future()
        def call():
            if not future.set_running_or_notify_cancel():
                return
            try:
                future.set_result(self.proxy(payload))
            except Exception as error:
                future.set_exception(error)
        Thread(target=call, daemon=True).start()
        return future


class VehicleRosNode:
    def __init__(self, store, ros_args=None):
        self.store = store
        self.odom_after_reset = self.heading_after_reset = None
        mappings = rospy.names.load_mappings(ros_args or [])
        namespace = mappings.get("__ns", rospy.get_namespace())
        self.node_name = resolve_name(mappings.get("__name", rospy.get_name()), namespace.rstrip("/") + "/")
        self.remappings = {resolve_name(k, self.node_name): resolve_name(v, self.node_name)
                          for k, v in mappings.items() if not k.startswith("_")}
        driver = rospy.get_param(self.node_name + "/driver_name", "vehicle").rstrip("/")
        self.stamped = rospy.get_param(self.node_name + "/use_stamped_cmd_vel", True)
        self.topic = lambda name: f"{driver}/{name}"
        self.subscriptions = []
        self.motion_publisher = rospy.Publisher(self.resolve_topic_name("cmd_vel"),
            TwistStamped if self.stamped else Twist, queue_size=1, tcp_nodelay=True)
        self.keepalive_publisher = rospy.Publisher(self.resolve_topic_name(self.topic("motion_keepalive")),
            UInt64, queue_size=1, tcp_nodelay=True)
        self.subscribe(State, self.topic("state"), self.on_state, 1)
        self.subscribe(Version, self.topic("version"),
            lambda m: store.update("version", {"hardware": m.hardware, "software": m.software}), 1)
        self.subscribe(TwistStamped, self.topic("velocity"),
            lambda m: store.update("velocity", {"vx": m.twist.linear.x, "wz": m.twist.angular.z}), 5)
        self.subscribe(WheelMileage, self.topic("wheel_mileage"), self.on_mileage, 5)
        self.subscribe(DriverMotion, self.topic("driver_motion"), self.on_driver_motion, 5)
        self.subscribe(DriverStatus, self.topic("driver_status"), self.on_driver_status, 5)
        self.subscribe(Odometry, "odom", self.on_odom)
        self.subscribe(Heading, self.topic("heading"), self.on_heading)
        self.subscribe(ImuCalibration, self.topic("imu_calibration"), self.on_calibration, 1)
        self.subscribe(MotionStatus, self.topic("motion_status"), self.on_motion_status, 1)
        self.subscribe(Imu, "imu/data", self.on_imu)
        self.subscribe(ImuStatus, "imu/status", self.on_imu_status)
        self.subscribe(DiagnosticArray, "diagnostics", self.on_diagnostics)
        self.subscribe(Acknowledgement, self.topic("acknowledgements"),
            lambda m: store.event(f"设备 ACK · 0x{m.command:02X}"))
        self.subscribe(CanFrame, self.topic("can_rx"), self.on_frame, 128)
        self.subscribe(Faults, self.topic("faults"), self.on_faults)
        self.command_client = ServiceClient(self.resolve_topic_name(self.topic("command")), Command)
        self.reset_client = ServiceClient(self.resolve_topic_name(self.topic("reset_odometry")), Trigger)
        self.calibration_client = ServiceClient(self.resolve_topic_name(self.topic("calibrate_imu")), Trigger)
        self.zero_imu_client = ServiceClient(self.resolve_topic_name("imu/zero_orientation"), Trigger)
        self.execute_client = ServiceClient(self.resolve_topic_name(self.topic("execute_motion")), ExecuteMotion)

    def resolve_topic_name(self, name):
        full = resolve_name(name, self.node_name)
        return self.remappings.get(full, full)

    def subscribe(self, message_type, name, callback, depth=10):
        self.subscriptions.append(rospy.Subscriber(self.resolve_topic_name(name), message_type,
            callback, queue_size=depth, tcp_nodelay=True))

    def close(self):
        for subscription in self.subscriptions:
            subscription.unregister()
        self.motion_publisher.unregister()
        self.keepalive_publisher.unregister()

    def on_motion_status(self, message):
        previous = self.store.latest("motion_status")
        self.store.update("motion_status", {key: getattr(message, key) for key in
            ("goal_id", "state", "mode", "active", "target", "progress", "remaining", "target_yaw",
             "heading_error", "command_linear_speed", "command_yaw_rate", "feedforward_gain", "message")})
        if message.active or (previous and previous.value["active"]):
            self.store.update("command", {"vx": message.command_linear_speed, "wz": message.command_yaw_rate})
        if message.goal_id and (previous is None or previous.value["state"] != message.state or
                               previous.value["goal_id"] != message.goal_id):
            self.store.event("自动运动 · " + message.message,
                             "warning" if message.state == MotionStatus.FAILED else "info")

    def on_state(self, message):
        now = rospy.Time.now().to_sec()
        def age(stamp, valid):
            return max(0.0, now - stamp.to_sec()) if valid else float("inf")
        self.store.update("state", {
            "online": message.online, "enabled": message.enabled,
            "heartbeat_received": message.heartbeat_received, "status_received": message.status_received,
            "heartbeat_age": age(message.heartbeat_stamp, message.heartbeat_received),
            "status_age": age(message.status_stamp, message.status_received),
            "mode": message.mode, "voltage": message.battery_voltage,
            "work_state": message.work_state, "beep": message.beep, "flags_raw": message.flags_raw,
        })

    def on_mileage(self, message):
        self.store.update("secondary_mileage" if message.secondary else "mileage",
                          {"left": message.left_mm, "right": message.right_mm})

    def on_driver_motion(self, message):
        if message.channel < 4:
            self.store.update(f"driver_motion_{message.channel}",
                              {"rpm": message.rpm, "pulses": message.pulses})

    def on_driver_status(self, message):
        if message.channel < 4:
            self.store.update(f"driver_status_{message.channel}", {
                "voltage": message.voltage, "current": message.current,
                "driver_temperature": message.driver_temperature,
                "motor_temperature": message.motor_temperature, "faults_raw": message.faults_raw,
            })

    def on_odom(self, message):
        stamp = message.header.stamp.to_nsec()
        if self.odom_after_reset is not None:
            if stamp <= self.odom_after_reset:
                return
            self.odom_after_reset = None
        p, q = message.pose.pose.position, message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.store.update("odom", {"x": p.x, "y": p.y, "yaw": yaw,
                                  "ros_stamp_ns": stamp, "vx": message.twist.twist.linear.x,
                                  "wz": message.twist.twist.angular.z,
                                  "frame": message.header.frame_id, "child": message.child_frame_id})

    def on_heading(self, message):
        stamp = message.header.stamp.to_nsec()
        if self.heading_after_reset is not None:
            if stamp <= self.heading_after_reset:
                return
            self.heading_after_reset = None
        self.store.update("heading", {"yaw": message.yaw, "wz": message.yaw_rate,
                                      "ros_stamp_ns": stamp,
                                      "imu_active": message.imu_active, "imu_age": message.imu_age})

    def on_imu(self, message):
        if message.orientation_covariance[0] < 0:
            self.store.update("imu", {"valid": False})
            return
        q = message.orientation
        self.store.update("imu", {
            "valid": True,
            "roll": math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)),
            "pitch": math.asin(max(-1.0, min(1.0, 2*(q.w*q.y-q.z*q.x)))),
            "yaw": math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
        })

    def on_calibration(self, message):
        previous = self.store.latest("imu_calibration")
        self.store.update("imu_calibration", {key: getattr(message, key) for key in
            ("state", "active", "calibrated", "yaw_sign", "positive_delta", "negative_delta",
             "command_yaw_rate", "message")})
        if message.active or (previous and previous.value["active"]):
            self.store.update("command", {"vx": 0.0, "wz": message.command_yaw_rate})
        if previous is None or previous.value["state"] != message.state:
            self.store.event("IMU 标定 · " + message.message,
                             "warning" if message.state == ImuCalibration.FAILED else "info")

    def on_imu_status(self, message):
        self.store.update("imu_status", {key: getattr(message, key) for key in
            ("online", "measurement_valid", "orientation_valid", "nmt_state", "faults",
             "sample_count", "quaternion_count", "measurement_age", "orientation_age",
             "rx_frames", "malformed_frames", "error_frames", "dropped_frames", "message")})

    def clear_odometry_display(self):
        # Queued pre-reset TCPROS samples must not reconnect the old path to the origin.
        self.odom_after_reset = rospy.Time.now().to_nsec()
        self.heading_after_reset = self.odom_after_reset
        self.store.clear_trajectory(clear_pose=True)

    def on_diagnostics(self, message):
        expected = self.resolve_topic_name(self.topic("socketcan"))
        for status in message.status:
            if status.name != expected:
                continue
            level = status.level if isinstance(status.level, int) else int.from_bytes(status.level, "little")
            self.store.update("diagnostics", {"level": level, "message": status.message,
                                               **{item.key: item.value for item in status.values}})

    def on_frame(self, message):
        self.store.frame(message.id, {"length": message.length,
                                     "data": bytes(message.data[:message.length]).hex(" ").upper(),
                                     "error": message.error})

    def on_faults(self, message):
        self.store.update("faults", {key: getattr(message, key) for key in
                                    ("motor", "driver", "driver_offline", "status_raw", "power")})

    def publish_motion(self, values):
        vx, wz = values
        message = TwistStamped() if self.stamped else Twist()
        if self.stamped:
            message.header.stamp = rospy.Time.now()
        twist = message.twist if self.stamped else message
        twist.linear.x, twist.angular.z = vx, wz
        self.motion_publisher.publish(message)
        self.store.update("command", {"vx": vx, "wz": wz})


class RosWorker(QThread):
    ready = pyqtSignal(str)
    outcome = pyqtSignal(object)
    failed = pyqtSignal(str)

    def __init__(self, store, lease, ros_args=None, parent=None):
        super().__init__(parent)
        self.store, self.lease, self.ros_args = store, lease, ros_args
        self.requests = Queue(maxsize=8)

    def submit(self, kind, command=0, **fields):
        try:
            self.requests.put_nowait({"kind": kind, "command": command, "fields": fields})
            return True
        except Full:
            return False

    def run(self):
        node = None
        pending = None
        goal_lease = None
        try:
            if not rospy.core.is_initialized():
                rospy.init_node("vehicle_gui", argv=["vehicle_gui"] + list(self.ros_args or []),
                                disable_signals=True)
            node = VehicleRosNode(self.store, self.ros_args)
            self.ready.emit(node.resolve_topic_name(node.topic("state")))
            next_send = time.monotonic()
            while not rospy.is_shutdown() and not self.isInterruptionRequested():
                time.sleep(0.01)
                now = time.monotonic()
                if now >= next_send:
                    next_send = now + 0.05
                    values = self.lease.sample(now)
                    if goal_lease is not None:
                        goal_id, revision = goal_lease
                        status = self.store.latest("motion_status")
                        if not self.lease.is_current(revision) or (status and
                                status.value["goal_id"] == goal_id and not status.value["active"]):
                            goal_lease = None
                        elif self.lease.fresh(revision, now):
                            node.keepalive_publisher.publish(UInt64(data=goal_id))
                        else:
                            # One explicit stop also covers a worker that resumes after a freeze.
                            values = values if values is not None else (0.0, 0.0)
                            goal_lease = None
                    if pending is not None and pending[1]["kind"] in ("calibrate_imu", "execute_motion") and \
                            not self.lease.is_current(pending[1]["fields"]["motion_revision"]):
                        # Keep a stop effective even if the service reaches the driver
                        # after the operator's first stop. Fresh manual input takes over.
                        values = values if values is not None else (0.0, 0.0)
                    if values is not None:
                        node.publish_motion(values)
                if pending is not None:
                    future, request, deadline = pending
                    if future.done() or now >= deadline:
                        result = dict(request, sent=False, acknowledged=False, state_confirmed=False)
                        try:
                            if not future.done():
                                future.cancel()
                                raise TimeoutError("等待服务响应超时，请查看设备状态")
                            response = future.result()
                            if request["kind"] in ("reset_odometry", "zero_imu"):
                                if request["kind"] == "reset_odometry" and response.success:
                                    node.clear_odometry_display()
                                result.update(sent=True, state_confirmed=response.success,
                                              message=response.message)
                            elif request["kind"] == "calibrate_imu":
                                result.update(sent=True, request_accepted=response.success,
                                              message=response.message)
                            elif request["kind"] == "execute_motion":
                                result.update(sent=True, request_accepted=response.accepted,
                                              goal_id=response.goal_id, message=response.message)
                            else:
                                result.update({key: getattr(response, key) for key in
                                               ("sent", "acknowledged", "state_confirmed", "message",
                                                "hardware_version", "software_version")})
                        except Exception as error:
                            result["message"] = str(error)
                        if request["kind"] in ("calibrate_imu", "execute_motion"):
                            revision = request["fields"]["motion_revision"]
                            result["superseded"] = not self.lease.is_current(revision)
                            if not result.get("request_accepted"):
                                self.lease.stop(revision)
                            if result["superseded"] or not result.get("request_accepted"):
                                node.publish_motion(self.lease.sample(now) or (0.0, 0.0))
                            elif request["kind"] == "execute_motion":
                                goal_lease = (result["goal_id"], revision)
                        self.outcome.emit(result)
                        pending = None
                try:
                    request = self.requests.get_nowait()
                except Empty:
                    continue
                if pending is not None:
                    self.outcome.emit(dict(request, sent=False, message="上一条命令仍在等待设备反馈"))
                    continue
                calibrating = request["kind"] in ("calibrate_imu", "execute_motion")
                if calibrating and not self.lease.is_current(request["fields"]["motion_revision"]):
                    self.outcome.emit(dict(request, sent=False, superseded=True,
                                           message="运动请求已被停止或手动控制取消，未发送"))
                    continue
                client = {"reset_odometry": node.reset_client, "calibrate_imu": node.calibration_client,
                          "zero_imu": node.zero_imu_client, "execute_motion": node.execute_client}.get(
                    request["kind"], node.command_client)
                if not client.service_is_ready():
                    if calibrating:
                        self.lease.stop(request["fields"]["motion_revision"])
                    message = "IMU 服务未连接，请先启动 IMU 节点" if request["kind"] == "zero_imu" else \
                              "底盘服务未连接，请先启动驱动"
                    self.outcome.emit(dict(request, sent=False, message=message))
                    continue
                if request["kind"] == "execute_motion":
                    payload = ExecuteMotionRequest(**{key: value for key, value in request["fields"].items()
                                                       if key != "motion_revision"})
                elif request["kind"] == "command":
                    payload = CommandRequest(command=request["command"], **request["fields"])
                else:
                    payload = TriggerRequest()
                pending = (client.call_async(payload), request, now + 3.0)
        except Exception as error:
            self.failed.emit(str(error))
        finally:
            self.lease.stop()
            if node is not None and not rospy.is_shutdown():
                try:
                    node.publish_motion((0.0, 0.0))
                    until = time.monotonic() + 0.08
                    while time.monotonic() < until:
                        time.sleep(0.005)
                except Exception:
                    pass
            if node is not None:
                node.close()
