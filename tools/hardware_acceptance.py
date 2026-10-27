#!/usr/bin/env python3
"""Exercise the built driver on an explicitly selected device and save evidence.

Starts the driver (including normal automatic enable/CAN-mode setup), queries
feedback and services. --motion adds short, low-speed raised-chassis runs.
Does not send RESET. Always sends stop/disable on exit, including exceptions.
Run inside a ROS environment with this workspace sourced and no other controller.
"""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import signal
import socket
import statistics
import struct
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from vehicle_interfaces.msg import DriverMotion, DriverStatus, State, WheelMileage
from vehicle_interfaces.srv import Command

BASE = 0x01E10100


def stats(values):
    if not values:
        return {"samples": 0}
    return {"samples": len(values), "mean": statistics.mean(values),
            "median": statistics.median(values), "min": min(values),
            "max": max(values), "stddev": statistics.pstdev(values)}


class Acceptance:
    def __init__(self, args):
        self.args = args
        self.report = {"started_utc": datetime.now(timezone.utc).isoformat(),
                       "interface": args.interface, "phases": [], "services": [], "checks": {}}
        self.raw = []
        self.state = None
        self.velocities, self.mileages, self.drivers, self.driver_states, self.diagnostics = [], [], [], [], []
        args.output.parent.mkdir(parents=True, exist_ok=True)
        self.trace = args.output.with_suffix(".candump").open("w")
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.socket.bind((args.interface,))
        self.socket.setblocking(False)
        self.node = rclpy.create_node("vehicle_hardware_acceptance")
        self.subscriptions = [
            self.node.create_subscription(State, "/vehicle/state", self.set_state, 10),
            self.node.create_subscription(TwistStamped, "/vehicle/velocity",
                                          lambda m: self.velocities.append((time.monotonic(), m)), qos_profile_sensor_data),
            self.node.create_subscription(WheelMileage, "/vehicle/wheel_mileage",
                                          lambda m: self.mileages.append((time.monotonic(), m)), qos_profile_sensor_data),
            self.node.create_subscription(DriverMotion, "/vehicle/driver_motion",
                                          lambda m: self.drivers.append((time.monotonic(), m)), qos_profile_sensor_data),
            self.node.create_subscription(DriverStatus, "/vehicle/driver_status",
                                          lambda m: self.driver_states.append((time.monotonic(), m)), qos_profile_sensor_data),
            self.node.create_subscription(DiagnosticArray, "/diagnostics", self.diagnostics.append, 10),
        ]
        self.publisher = self.node.create_publisher(TwistStamped, "/cmd_vel", 1)
        self.client = self.node.create_client(Command, "/vehicle/command")
        prefix = Path(get_package_prefix("vehicle_driver"))
        executable = prefix / "lib/vehicle_driver/vehicle_node"
        component = prefix / "lib/libvehicle_component.so"
        self.report["driver_binary_sha256"] = hashlib.sha256(component.read_bytes()).hexdigest()
        self.log = args.output.with_suffix(".log").open("w")
        self.process = subprocess.Popen([str(executable), "--ros-args", "-p", f"can_interface:={args.interface}"],
                                        stdout=self.log, stderr=subprocess.STDOUT)

    def set_state(self, message):
        self.state = message

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0.001)
        for _ in range(1000):
            try:
                raw = self.socket.recv(16)
            except BlockingIOError:
                break
            can_id, size, data = struct.unpack("=IB3x8s", raw)
            stamp, wall = time.monotonic(), time.time()
            self.raw.append((stamp, can_id, data[:size]))
            self.trace.write(f"({wall:.6f}) {self.args.interface} {can_id & 0x1FFFFFFF:08X}#{data[:size].hex().upper()}\n")

    def pump(self, seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            self.step()

    def wait(self, predicate, timeout=5):
        until = time.monotonic() + timeout
        while not predicate() and time.monotonic() < until:
            self.step()
            if self.process.poll() is not None:
                raise RuntimeError("driver exited unexpectedly")
        if not predicate():
            raise TimeoutError("expected ROS/device feedback not observed")

    def call(self, code, **fields):
        self.wait(self.client.service_is_ready)
        start = time.monotonic()
        future = self.client.call_async(Command.Request(command=code, **fields))
        self.wait(future.done)
        response = future.result()
        entry = {"command": code, "fields": fields, "elapsed_s": time.monotonic() - start,
                 **message_to_ordereddict(response)}
        self.report["services"].append(entry)
        if not response.sent:
            raise RuntimeError(response.message)
        return response

    def publish(self, vx=0.0, wz=0.0):
        message = TwistStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.twist.linear.x, message.twist.angular.z = vx, wz
        self.publisher.publish(message)

    def phase(self, label, vx, wz, seconds):
        print(f"Phase {label}: vx={vx}, wz={wz}, {seconds}s", flush=True)
        start = time.monotonic()
        next_publish = start
        before = self.mileages[-1][1] if self.mileages else None
        last_publish = start
        while time.monotonic() - start < seconds:
            if time.monotonic() >= next_publish:
                self.publish(vx, wz)
                last_publish = time.monotonic()
                next_publish = last_publish + 0.02
            self.step()
        end = time.monotonic()
        sample_start = start + min(0.6, seconds / 2)
        values = [m for t, m in self.velocities if sample_start <= t <= end]
        after = self.mileages[-1][1] if self.mileages else None
        entry = {"label": label, "command_vx": vx, "command_yaw_rate": wz, "duration_s": end - start,
                 "vx": stats([m.twist.linear.x for m in values]),
                 "yaw_rate": stats([m.twist.angular.z for m in values])}
        if before and after:
            entry["left_delta_mm"] = after.left_mm - before.left_mm
            entry["right_delta_mm"] = after.right_mm - before.right_mm
        self.report["phases"].append(entry)
        self.publish()
        self.pump(0.65)
        return last_publish

    def count(self, function, start):
        return sum(1 for stamp, identifier, _ in self.raw
                   if stamp >= start and (identifier & 0x1FFFFFFF) == BASE | function)

    def run(self):
        self.wait(lambda: self.state and self.state.online and self.state.enabled and
                  self.state.status_received and self.state.mode == 2 and
                  self.publisher.get_subscription_count() > 0, timeout=6)
        self.pump(0.8)
        self.report["initial_state"] = message_to_ordereddict(self.state)
        version = self.call(Command.Request.QUERY_VERSION)
        self.report["checks"]["ascii_version"] = (version.hardware_version, version.software_version) == ("3.0", "4.0")
        self.report["checks"]["enable_confirmed"] = self.call(Command.Request.ENABLE, enabled=True).state_confirmed
        self.report["checks"]["clear_ack"] = self.call(Command.Request.CLEAR_FAULTS).acknowledged
        for beep in (True, False):
            result = self.call(Command.Request.SET_MODE, mode=2, beep=beep)
            self.report["checks"][f"beep_{beep}"] = result.state_confirmed
        switches = [(Command.Request.REMOTE_FEEDBACK, 0xB5),
                    (Command.Request.DRIVER_MOTION_FEEDBACK, 0xB7),
                    (Command.Request.DRIVER_STATUS_FEEDBACK, 0xB8)]
        for command, function in switches:
            counts = {}
            for enabled in (False, True):
                self.call(command, enabled=enabled)
                self.pump(0.15)
                start = time.monotonic()
                self.pump(0.5)
                counts[str(enabled)] = self.count(function, start)
            self.report["checks"][f"feedback_{function:02X}"] = counts["False"] == 0 and counts["True"] > 0
            self.report[f"feedback_{function:02X}_counts"] = counts
        self.call(Command.Request.REMOTE_FEEDBACK, enabled=False)
        self.call(Command.Request.SENSOR_FEEDBACK, enabled=True)
        start = time.monotonic()
        self.pump(0.5)
        self.report["sensor_B6_observed"] = self.count(0xB6, start)
        self.call(Command.Request.SENSOR_FEEDBACK, enabled=False)
        if self.args.motion:
            for label, vx, wz, seconds in [
                ("forward_0.10", 0.10, 0.0, 2.5), ("reverse_0.10", -0.10, 0.0, 2.5),
                ("forward_0.05", 0.05, 0.0, 3.0), ("rotate_positive_0.20", 0.0, 0.20, 2.5),
                ("rotate_negative_0.20", 0.0, -0.20, 2.5), ("rotate_positive_0.10", 0.0, 0.10, 3.0),
            ]:
                self.phase(label, vx, wz, seconds)
            # Deliberately stop publishing without sending an explicit zero.
            start = time.monotonic()
            for _ in range(20):
                self.publish(0.1, 0.0)
                last_publish = time.monotonic()
                self.pump(0.02)
            self.pump(0.8)
            motion = [(t, struct.unpack("<hhh", data)) for t, identifier, data in self.raw
                      if t >= start and identifier & 0x1FFFFFFF == BASE | 0x12 and len(data) == 6]
            nonzero = [t for t, values in motion if values[0] != 0]
            stop = [t for t, values in motion if nonzero and t > nonzero[-1] and values == (0, 0, 0)]
            if stop:
                delay = stop[0] - last_publish
                self.report["ros_command_watchdog_s"] = delay
                self.report["checks"]["command_watchdog"] = 0.25 < delay < 0.4
        self.publish()
        self.pump(0.5)
        zero = self.call(Command.Request.ZERO_MOTION)
        self.pump(0.5)
        self.report["checks"]["zero_ack"] = zero.acknowledged
        self.report["checks"]["zero_mileage_observed"] = bool(self.mileages and
            self.mileages[-1][1].left_mm == 0 and self.mileages[-1][1].right_mm == 0)
        latest_drivers = {m.channel: m for _, m in self.drivers}
        self.report["checks"]["four_driver_channels"] = set(latest_drivers) == {0, 1, 2, 3}
        self.report["checks"]["zero_pulses_observed"] = len(latest_drivers) == 4 and all(
            m.pulses == 0 for m in latest_drivers.values())
        result = self.call(Command.Request.ENABLE, enabled=False)
        self.report["checks"]["disable_confirmed"] = result.acknowledged and result.state_confirmed
        self.pump(0.3)
        if self.diagnostics:
            self.report["diagnostics"] = message_to_ordereddict(self.diagnostics[-1])
        self.report["final_state"] = message_to_ordereddict(self.state)

    def close(self):
        # Best effort stop/disable even if DDS or the child process fails.
        for function, data in [(0x12, bytes(6)), (3, bytes([1, 0xE1, 1, 0, 1]))]:
            try:
                self.socket.send(struct.pack("=IB3x8s", 0x80000000 | BASE | function, len(data), data))
            except OSError:
                pass
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        self.pump(0.7)
        self.report["frame_counts"] = dict(Counter(f"{identifier & 0x1FFFFFFF:08X}" for _, identifier, _ in self.raw))
        periodic = defaultdict(list)
        for t, identifier, data in self.raw:
            if identifier & 0x1FFFFF00 == BASE:
                periodic[f"{identifier & 0xFF:02X}"] .append((t, len(data)))
        self.report["feedback_summary"] = {code: {
            "lengths": sorted({size for _, size in rows}), "frames": len(rows),
            "median_period_s": statistics.median([b[0] - a[0] for a, b in zip(rows, rows[1:])]) if len(rows) > 1 else None,
        } for code, rows in periodic.items() if int(code, 16) >= 0xA0}
        self.report["finished_utc"] = datetime.now(timezone.utc).isoformat()
        self.trace.close()
        self.report["candump_sha256"] = hashlib.sha256(self.args.output.with_suffix(".candump").read_bytes()).hexdigest()
        self.args.output.write_text(json.dumps(self.report, indent=2, ensure_ascii=False) + "\n")
        self.log.close()
        self.socket.close()
        self.node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--motion", action="store_true")
    parser.add_argument("--output", type=Path, default=Path("artifacts/hardware_acceptance.json"))
    args = parser.parse_args()
    rclpy.init()
    runner = Acceptance(args)
    try:
        runner.run()
    except Exception as error:
        runner.report["error"] = str(error)
        raise
    finally:
        runner.close()
        rclpy.shutdown()
    print(json.dumps(runner.report["checks"], indent=2), flush=True)
    if not all(runner.report["checks"].values()):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
