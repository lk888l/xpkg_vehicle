# SPDX-License-Identifier: Apache-2.0
"""The ROS executor and all service futures live in this one worker thread."""
import math
from queue import Empty, Full, Queue
import time

from PyQt5.QtCore import QThread, pyqtSignal
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import Trigger
from vehicle_interfaces.msg import (Acknowledgement, CanFrame, DriverMotion, DriverStatus,
                                    Faults, State, Version, WheelMileage)
from vehicle_interfaces.srv import Command


class VehicleRosNode(Node):
    def __init__(self, store, context):
        super().__init__("vehicle_gui", context=context)
        self.store = store
        self.odom_after_reset = None
        driver = self.declare_parameter("driver_name", "vehicle").value.rstrip("/")
        self.stamped = self.declare_parameter("use_stamped_cmd_vel", True).value
        self.topic = lambda name: f"{driver}/{name}"
        self.motion_publisher = self.create_publisher(TwistStamped if self.stamped else Twist, "cmd_vel", 1)
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(State, self.topic("state"), self.on_state, durable)
        self.create_subscription(Version, self.topic("version"),
                                 lambda m: store.update("version", {"hardware": m.hardware, "software": m.software}), durable)
        self.create_subscription(TwistStamped, self.topic("velocity"),
                                 lambda m: store.update("velocity", {"vx": m.twist.linear.x, "wz": m.twist.angular.z}), qos_profile_sensor_data)
        self.create_subscription(WheelMileage, self.topic("wheel_mileage"), self.on_mileage, qos_profile_sensor_data)
        self.create_subscription(DriverMotion, self.topic("driver_motion"), self.on_driver_motion, qos_profile_sensor_data)
        self.create_subscription(DriverStatus, self.topic("driver_status"), self.on_driver_status, qos_profile_sensor_data)
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.create_subscription(DiagnosticArray, "diagnostics", self.on_diagnostics, 10)
        self.create_subscription(Acknowledgement, self.topic("acknowledgements"),
                                 lambda m: store.event(f"设备 ACK · 0x{m.command:02X}"), 10)
        self.create_subscription(CanFrame, self.topic("can_rx"), self.on_frame, qos_profile_sensor_data)
        self.create_subscription(Faults, self.topic("faults"), self.on_faults, 10)
        self.command_client = self.create_client(Command, self.topic("command"))
        self.reset_client = self.create_client(Trigger, self.topic("reset_odometry"))

    def on_state(self, message):
        now = self.get_clock().now().nanoseconds / 1e9
        def age(stamp, valid):
            return max(0.0, now - stamp.sec - stamp.nanosec / 1e9) if valid else float("inf")
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
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if self.odom_after_reset is not None:
            if stamp <= self.odom_after_reset:
                return
            self.odom_after_reset = None
        p, q = message.pose.pose.position, message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.store.update("odom", {"x": p.x, "y": p.y, "yaw": yaw,
                                  "frame": message.header.frame_id, "child": message.child_frame_id})

    def clear_odometry_display(self):
        # Queued pre-reset DDS samples must not reconnect the old path to the origin.
        self.odom_after_reset = self.get_clock().now().nanoseconds
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
            message.header.stamp = self.get_clock().now().to_msg()
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
        context = Context()
        executor = None
        node = None
        pending = None
        try:
            rclpy.init(args=self.ros_args, context=context)
            node = VehicleRosNode(self.store, context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            self.ready.emit(node.resolve_topic_name(node.topic("state")))
            next_send = time.monotonic()
            while context.ok() and not self.isInterruptionRequested():
                executor.spin_once(timeout_sec=0.01)
                now = time.monotonic()
                if now >= next_send:
                    next_send = now + 0.05
                    values = self.lease.sample(now)
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
                            if request["kind"] == "reset_odometry":
                                if response.success:
                                    node.clear_odometry_display()
                                result.update(sent=True, state_confirmed=response.success,
                                              message=response.message)
                            else:
                                result.update({key: getattr(response, key) for key in
                                               ("sent", "acknowledged", "state_confirmed", "message",
                                                "hardware_version", "software_version")})
                        except Exception as error:
                            result["message"] = str(error)
                        self.outcome.emit(result)
                        pending = None
                try:
                    request = self.requests.get_nowait()
                except Empty:
                    continue
                if pending is not None:
                    self.outcome.emit(dict(request, sent=False, message="上一条命令仍在等待设备反馈"))
                    continue
                client = node.reset_client if request["kind"] == "reset_odometry" else node.command_client
                if not client.service_is_ready():
                    self.outcome.emit(dict(request, sent=False, message="底盘服务未连接，请先启动驱动"))
                    continue
                payload = Trigger.Request() if request["kind"] == "reset_odometry" else Command.Request(
                    command=request["command"], **request["fields"])
                pending = (client.call_async(payload), request, now + 3.0)
        except Exception as error:
            self.failed.emit(str(error))
        finally:
            self.lease.stop()
            if node is not None and context.ok():
                try:
                    node.publish_motion((0.0, 0.0))
                    until = time.monotonic() + 0.08
                    while time.monotonic() < until:
                        executor.spin_once(timeout_sec=0.005)
                except Exception:
                    pass
            if executor is not None:
                executor.shutdown(timeout_sec=0.5)
            if node is not None:
                node.destroy_node()
            if context.ok():
                context.shutdown()
