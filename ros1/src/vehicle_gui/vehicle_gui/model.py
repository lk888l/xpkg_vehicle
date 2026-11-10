# SPDX-License-Identifier: Apache-2.0
"""Thread-safe state and short-lived operator intent, independent of Qt and ROS."""
from __future__ import annotations
from collections import deque
import csv
from dataclasses import dataclass
import math
from threading import Lock
import time
from typing import Any


@dataclass(frozen=True)
class Sample:
    value: dict[str, Any]
    received: float

    def age(self, now=None):
        return max(0.0, (time.monotonic() if now is None else now) - self.received)

    def fresh(self, limit, now=None):
        return self.age(now) <= limit


class TelemetryStore:
    PATH_COLUMNS = ("monotonic_s", "ros_stamp_ns", "segment", "x", "y", "yaw", "vx", "wz",
                    "imu_active", "imu_age", "imu_yaw", "imu_received", "heading_received",
                    "heading_ros_stamp_ns", "target_yaw", "control_mode",
                    "imu_yaw_sign", "imu_calibration_state",
                    "command_vx", "command_wz", "frame", "child")

    def __init__(self, capacity=3000, path_capacity=60000):
        self._lock = Lock()
        self._latest = {}
        self._history = {key: deque(maxlen=capacity) for key in ("velocity", "command", "odom")}
        self._history["odom"] = deque(maxlen=path_capacity)
        self._frames = {}
        self._events = deque(maxlen=200)
        self._revision = 0
        self._recording = None
        self._writer = None
        self._record_count = 0
        self._record_error = ""
        self._last_flush = 0.0
        self._segment = 0

    def update(self, name, value, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            if name == "odom":
                value = dict(value, monotonic_s=now, segment=self._segment)
                heading = self._latest.get("heading")
                value["imu_active"] = bool(heading and heading.fresh(0.25, now) and
                                           heading.value["imu_active"] and
                                           heading.value["imu_age"] + heading.age(now) < 0.25)
                value["imu_age"] = (heading.value["imu_age"] + heading.age(now)) if \
                    heading and heading.value["imu_age"] >= 0 else -1
                if heading:
                    value["heading_received"] = heading.received
                    value["heading_ros_stamp_ns"] = heading.value.get("ros_stamp_ns", "")
                for source, fields in (("imu", {"yaw": "imu_yaw"}),
                                       ("imu_calibration", {"yaw_sign": "imu_yaw_sign", "state": "imu_calibration_state"}),
                                       ("steering", {"target": "target_yaw", "mode": "control_mode"}),
                                       ("command", {"vx": "command_vx", "wz": "command_wz"})):
                    sample = self._latest.get(source)
                    if sample:
                        value.update({dest: sample.value.get(key) for key, dest in fields.items()})
                        if source == "imu":
                            value["imu_received"] = sample.received
                if self._writer:
                    try:
                        self._writer.writerow({key: value.get(key, "") for key in self.PATH_COLUMNS})
                        self._record_count += 1
                        if now - self._last_flush >= 1:
                            self._recording.flush()
                            self._last_flush = now
                    except OSError as error:
                        self._record_error = str(error)
                        try:
                            self._recording.close()
                        except OSError:
                            pass
                        self._recording = self._writer = None
            self._latest[name] = Sample(dict(value), now)
            if name in self._history:
                # Keep a long display path; streaming CSV still receives every odometry sample.
                if name == "odom" and self._history[name] and now - self._history[name][-1][0] < 0.10:
                    return
                self._history[name].append((now, dict(value)))

    def latest(self, name):
        with self._lock:
            return self._latest.get(name)

    def start_recording(self, path):
        with self._lock:
            if self._recording:
                raise RuntimeError("路径记录已在运行")
            stream = open(path, "w", newline="", encoding="utf-8")
            try:
                writer = csv.DictWriter(stream, fieldnames=self.PATH_COLUMNS)
                writer.writeheader()
            except Exception:
                stream.close()
                raise
            self._recording, self._writer = stream, writer
            self._record_count, self._record_error = 0, ""

    def stop_recording(self):
        with self._lock:
            stream = self._recording
            self._recording = self._writer = None
            if stream:
                try:
                    stream.close()
                except OSError as error:
                    self._record_error = str(error)

    def recording_status(self):
        with self._lock:
            return bool(self._recording), self._record_count, self._record_error

    def export_path(self, path):
        with self._lock:
            rows = list(self._history["odom"])
        with open(path, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=self.PATH_COLUMNS, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(dict(value, monotonic_s=stamp) for stamp, value in rows)

    def frame(self, identifier, value):
        with self._lock:
            old = self._frames.get(identifier)
            if old is None and len(self._frames) >= 64:
                self._frames.pop(next(iter(self._frames)))
            self._frames[identifier] = Sample(
                dict(value, count=1 if old is None else old.value["count"] + 1), time.monotonic())

    def event(self, text, level="info"):
        with self._lock:
            self._revision += 1
            self._events.append((time.strftime("%H:%M:%S"), level, text))

    def snapshot(self):
        with self._lock:
            return (dict(self._latest), {k: list(v) for k, v in self._history.items()},
                    dict(self._frames), list(self._events), self._revision)

    def clear_history(self):
        with self._lock:
            for values in self._history.values():
                values.clear()

    def clear_trajectory(self, *, clear_pose=False):
        with self._lock:
            self._history["odom"].clear()
            if clear_pose:
                self._latest.pop("odom", None)
                self._latest.pop("heading", None)
                self._segment += 1


def connection_state(sample, now=None):
    if sample is None:
        return "等待设备", False
    now = time.monotonic() if now is None else now
    value = sample.value
    age = value.get("heartbeat_age", float("inf")) + sample.age(now)
    online = sample.fresh(1.5, now) and value.get("online", False) and age < 1.5
    return ("设备在线" if online else "反馈已过期"), online


class MotionLease:
    """The ROS worker never keeps transmitting a frozen UI's last command."""
    def __init__(self, timeout=0.20):
        self.timeout = timeout
        self._lock = Lock()
        self._value = (0.0, 0.0)
        self._received = 0.0
        self._active = False
        self._stop_pending = False
        self._revision = 0

    def renew(self, vx, wz, now=None):
        now = time.monotonic() if now is None else now
        if not all(math.isfinite(x) for x in (vx, wz, now)):
            self.stop()
            return False
        with self._lock:
            self._revision += 1
            self._value = (vx, wz)
            self._received = now
            self._active = True
            self._stop_pending = False
        return True

    def stop(self, revision=None):
        with self._lock:
            if revision is not None and revision != self._revision:
                return False
            self._revision += 1
            self._active = False
            self._stop_pending = True
            return True

    def handoff(self):
        """Release GUI motion; the receiving service itself stops the chassis.

        Do not enqueue a zero cmd_vel here: it would cancel the new calibration.
        Any subsequent operator input invalidates the returned request revision.
        """
        with self._lock:
            self._revision += 1
            self._active = self._stop_pending = False
            self._received = time.monotonic()
            return self._revision

    def keepalive(self, revision, now=None):
        with self._lock:
            if revision != self._revision:
                return False
            self._received = time.monotonic() if now is None else now
            return True

    def fresh(self, revision, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            return revision == self._revision and 0 <= now - self._received < self.timeout

    def is_current(self, revision):
        with self._lock:
            return revision == self._revision

    def sample(self, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            if self._active and 0 <= now - self._received < self.timeout:
                return self._value
            if self._active or self._stop_pending:
                self._active = self._stop_pending = False
                return (0.0, 0.0)
            return None


COMMAND_NAMES = {2: "查询版本", 3: "设置使能", 4: "清故障", 5: "设备计数清零",
                 17: "设置模式 / 蜂鸣器", 21: "遥控反馈", 22: "传感反馈",
                 23: "驱动运动反馈", 24: "驱动状态反馈"}


def describe_result(result):
    if result.get("kind") == "execute_motion":
        if result.get("superseded"):
            return "info", "运动请求已被停止或手动控制取消"
        if result.get("request_accepted"):
            return "info", "运动请求已受理；到位与停止原因见自动运动状态"
        return "warning", "运动请求未受理 · " + result.get("message", "请检查底盘与 IMU")
    if result.get("kind") == "calibrate_imu" and result.get("superseded"):
        return "info", "IMU 标定请求已被停止或手动控制取消"
    if not result.get("sent"):
        return "error", result.get("message", "命令未发送")
    if result.get("kind") == "calibrate_imu":
        if result.get("request_accepted"):
            return "info", "IMU 标定请求已受理；进度与最终结果见上方标定状态"
        return "warning", "IMU 标定请求未受理 · " + result.get("message", "请查看标定状态")
    if result.get("kind") == "zero_imu":
        if result.get("state_confirmed"):
            return "info", "IMU 横滚、俯仰和航向已以当前姿态置零"
        return "warning", "IMU 角度置零失败 · " + result.get("message", "请检查姿态数据是否有效")
    command = result.get("command")
    if result.get("kind") == "reset_odometry":
        if result.get("state_confirmed"):
            return "info", "本地里程计已清零，旧轨迹已清空；设备里程与脉冲保持不变"
        return "warning", result.get("message", "里程计清零尚未确认")
    if command in (4, 5) and result.get("acknowledged"):
        return "info", "已收到接收确认；实际效果请查看故障状态或里程 / 脉冲反馈"
    if command == 17 and result.get("state_confirmed"):
        return "info", "模式和蜂鸣器状态已匹配；刹车 / 特殊功能状态未知"
    if command in (2, 3) and result.get("state_confirmed"):
        return "info", "设备状态已确认"
    if command in (21, 22, 23, 24):
        return "info", "反馈开关已发送；请观察对应话题是否更新"
    return "warning", "尚未确认设备状态；请区分发送、应答与实际反馈"
