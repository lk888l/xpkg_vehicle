# SPDX-License-Identifier: Apache-2.0
"""Thread-safe state and short-lived operator intent, independent of Qt and ROS."""
from collections import deque
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
    def __init__(self, capacity=3000):
        self._lock = Lock()
        self._latest = {}
        self._history = {key: deque(maxlen=capacity) for key in ("velocity", "command", "odom")}
        self._frames = {}
        self._events = deque(maxlen=200)
        self._revision = 0

    def update(self, name, value, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            self._latest[name] = Sample(dict(value), now)
            if name in self._history:
                self._history[name].append((now, dict(value)))

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

    def renew(self, vx, wz, now=None):
        now = time.monotonic() if now is None else now
        if not all(math.isfinite(x) for x in (vx, wz, now)):
            self.stop()
            return False
        with self._lock:
            self._value = (vx, wz)
            self._received = now
            self._active = True
            self._stop_pending = False
        return True

    def stop(self):
        with self._lock:
            self._active = False
            self._stop_pending = True

    def sample(self, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            if self._active and 0 <= now - self._received < self.timeout:
                return self._value
            if self._active or self._stop_pending:
                self._active = self._stop_pending = False
                return (0.0, 0.0)
            return None


COMMAND_NAMES = {2: "查询版本", 3: "设置使能", 4: "清故障", 5: "清运动计数",
                 17: "设置模式 / 蜂鸣器", 21: "遥控反馈", 22: "传感反馈",
                 23: "驱动运动反馈", 24: "驱动状态反馈"}


def describe_result(result):
    if not result.get("sent"):
        return "error", result.get("message", "命令未发送")
    command = result.get("command")
    if result.get("kind") == "reset_odometry":
        return ("info" if result.get("state_confirmed") else "warning"), result["message"]
    if command in (4, 5) and result.get("acknowledged"):
        return "info", "已收到接收确认；实际效果请查看故障状态或里程 / 脉冲反馈"
    if command == 17 and result.get("state_confirmed"):
        return "info", "模式和蜂鸣器状态已匹配；刹车 / 特殊功能状态未知"
    if command in (2, 3) and result.get("state_confirmed"):
        return "info", "设备状态已确认"
    if command in (21, 22, 23, 24):
        return "info", "反馈开关已发送；请观察对应话题是否更新"
    return "warning", "尚未确认设备状态；请区分发送、应答与实际反馈"
