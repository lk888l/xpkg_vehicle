# SPDX-License-Identifier: Apache-2.0
"""Angle control in the same odom frame used to draw and record the path."""
import math


def wrap(angle):
    return math.remainder(angle, 2 * math.pi)


class SteeringController:
    def __init__(self, gain=1.8, tolerance=math.radians(1.0)):
        self.gain, self.tolerance = gain, tolerance
        self.target = None
        self.turning = False
        self.mode = "手动"
        self.error = 0.0
        self._yaw = None
        self._unwrapped = 0.0
        self._settled_since = None

    def observe(self, yaw):
        if not math.isfinite(yaw):
            raise ValueError("航向必须是有限数值")
        self._unwrapped = yaw if self._yaw is None else self._unwrapped + wrap(yaw - self._yaw)
        self._yaw = yaw
        return self._unwrapped

    def cancel(self):
        self.target = None
        self.turning = False
        self._settled_since = None
        self.error = 0.0
        self.mode = "手动"

    def reset(self):
        self.cancel()
        self._yaw = None

    def start_turn(self, angle, relative, yaw):
        if not math.isfinite(angle):
            raise ValueError("转向角度必须是有限数值")
        current = self.observe(yaw)
        self.target = current + (angle if relative else wrap(angle - yaw))
        self.turning = True
        self._settled_since = None
        self.mode = "定角转向"
        self.error = self.target - current

    def _correction(self, current, max_rate):
        self.error = self.target - current
        return max(-max_rate, min(max_rate, self.gain * self.error))

    def manual(self, vx, wz, yaw, hold, max_rate):
        if self.turning:
            self.cancel()
        if yaw is None or not hold or wz != 0 or vx == 0:
            self.target = None
            self.error = 0.0
            self.mode = "自由转弯" if wz else "手动"
            if yaw is not None:
                self.observe(yaw)
            return vx, wz
        current = self.observe(yaw)
        if self.target is None:
            self.target = current
        self.mode = "直行保向"
        return vx, self._correction(current, max_rate)

    def step_turn(self, yaw, yaw_rate, max_rate, now):
        if not self.turning:
            return 0.0, 0.0
        if yaw is None:
            self.cancel()
            self.mode = "IMU 航向过期，定角转向已结束"
            return 0.0, 0.0
        current = self.observe(yaw)
        wz = self._correction(current, max_rate)
        if abs(self.error) <= self.tolerance:
            # Observe settling rather than declaring success while crossing the target.
            if abs(yaw_rate) < 0.05:
                if self._settled_since is None:
                    self._settled_since = now
                if now - self._settled_since >= 0.15:
                    self.turning = False
                    self.mode = "转向完成"
            else:
                self._settled_since = None
            wz = 0.0
        else:
            self._settled_since = None
        return 0.0, wz
