"""Real SocketCAN + DDS integration against an isolated virtual vehicle.

Set VEHICLE_TEST_CAN=vcan_vehicle. Only a kernel vcan device is accepted.
The emulator never connects to a physical CAN interface.
"""
import errno
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import threading
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from rclpy.qos import qos_profile_sensor_data
from vehicle_interfaces.msg import DriverMotion, State, Version
from vehicle_interfaces.srv import Command

BASE = 0x01E10100
EFF = 0x80000000


class Emulator:
    def __init__(self, interface):
        info = json.loads(subprocess.check_output(
            ["ip", "-json", "-details", "link", "show", interface], text=True))
        assert info[0]["linkinfo"]["info_kind"] == "vcan", "Tests require a virtual CAN device"
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.socket.bind((interface,))
        self.socket.settimeout(0.002)
        self.enabled = False
        self.mode = 0
        self.beep = False
        self.ack = True
        self.apply_enable = True
        self.work_state = 0
        self.commands = []
        self.running = True
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def send(self, function, data=b"", base=BASE):
        try:
            self.socket.send(struct.pack("=IB3x8s", EFF | base | function, len(data), data))
        except OSError as error:
            if error.errno not in (errno.ENETDOWN, errno.ENODEV):
                raise

    def run(self):
        deadline = time.monotonic()
        channel = 0
        while self.running:
            try:
                raw = self.socket.recv(16)
                identifier, length, payload = struct.unpack("=IB3x8s", raw)
                function = identifier & 0xFF
                if (identifier & 0x1FFFFF00) != BASE:
                    continue
                self.commands.append((time.monotonic(), function, payload[:length]))
                if function == 2:
                    self.send(0xA2, b"3.0\x004.0\x00")
                elif function == 3:
                    if self.apply_enable:
                        self.enabled = bool(payload[3])
                    if self.ack:
                        self.send(0xA3)
                elif function in (4, 5) and self.ack:
                    self.send(0xA0 + function)
                elif function == 0x11:
                    self.mode, self.beep = payload[:2]
            except socket.timeout:
                pass
            except OSError as error:
                if error.errno not in (errno.ENETDOWN, errno.ENODEV):
                    raise
                time.sleep(0.002)
            if time.monotonic() >= deadline:
                deadline = time.monotonic() + 0.02
                self.send(0xB0, bytes([self.enabled]))
                self.send(0xB1, struct.pack("<BBhBB", self.work_state, self.mode, 282, self.beep, 0x20))
                self.send(0xB2, struct.pack("<hhh", 0, 0, 0))
                self.send(0xB3, struct.pack("<ii", 0, 0))
                self.send(0xB7, struct.pack("<Bhi", channel, -10, -100))
                channel = (channel + 1) % 4

    def close(self):
        self.running = False
        self.thread.join(timeout=2)
        self.socket.close()


class Rig:
    def __init__(self, interface, directory):
        self.interface = interface
        self.emulator = Emulator(interface)
        self.node = rclpy.create_node("vehicle_integration_probe")
        self.states, self.velocities, self.drivers, self.diagnostics, self.versions = [], [], [], [], []
        self.subscriptions = [
            self.node.create_subscription(State, "/test_vehicle/state", self.states.append, 10),
            self.node.create_subscription(TwistStamped, "/test_vehicle/velocity",
                                          self.velocities.append, qos_profile_sensor_data),
            self.node.create_subscription(DriverMotion, "/test_vehicle/driver_motion",
                                          self.drivers.append, qos_profile_sensor_data),
            self.node.create_subscription(DiagnosticArray, "/test_diagnostics", self.diagnostics.append, 10),
            self.node.create_subscription(Version, "/test_vehicle/version", self.versions.append, 10),
        ]
        self.publisher = self.node.create_publisher(TwistStamped, "/test_cmd_vel", 1)
        self.client = self.node.create_client(Command, "/test_vehicle/command")
        executable = Path(get_package_prefix("vehicle_driver")) / "lib/vehicle_driver/vehicle_node"
        self.log_path = directory / "node.log"
        self.log_file = self.log_path.open("w")
        self.process = subprocess.Popen([
            str(executable), "--ros-args", "-r", "__node:=test_vehicle",
            "-r", "cmd_vel:=test_cmd_vel", "-r", "odom:=test_odom",
            "-r", "diagnostics:=test_diagnostics", "-p", f"can_interface:={interface}",
            "-p", "publish_tf:=false", "-p", "service_timeout:=0.6",
        ], stdout=self.log_file, stderr=subprocess.STDOUT)

    def spin(self, seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            assert self.process.poll() is None, self.log_path.read_text()

    def wait(self, predicate, timeout=4):
        until = time.monotonic() + timeout
        while not predicate() and time.monotonic() < until:
            self.spin(0.01)
        assert predicate(), self.log_path.read_text()

    def request(self, command, **fields):
        self.wait(lambda: self.client.service_is_ready())
        request = Command.Request(command=command, **fields)
        return self.client.call_async(request)

    def call(self, command, **fields):
        future = self.request(command, **fields)
        self.wait(future.done)
        return future.result()

    def publish(self, vx=0.1, wz=0.0, age=0.0):
        message = TwistStamped()
        message.header.stamp = (self.node.get_clock().now() - rclpy.duration.Duration(seconds=age)).to_msg()
        message.twist.linear.x = vx
        message.twist.angular.z = wz
        self.publisher.publish(message)

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        self.log_file.close()
        self.node.destroy_node()
        self.emulator.close()


@pytest.fixture
def rig(tmp_path):
    interface = os.environ.get("VEHICLE_TEST_CAN")
    if not interface:
        pytest.skip("Set VEHICLE_TEST_CAN to an isolated vcan interface for integration tests")
    rclpy.init()
    test_rig = Rig(interface, tmp_path)
    try:
        test_rig.wait(lambda: test_rig.states and test_rig.states[-1].enabled and
                      test_rig.states[-1].mode == 2 and test_rig.publisher.get_subscription_count() > 0)
        yield test_rig
    finally:
        test_rig.close()
        rclpy.shutdown()


def test_startup_telemetry_identity_and_malformed_frames(rig):
    rig.wait(lambda: len({m.channel for m in rig.drivers}) == 4)
    state = rig.states[-1]
    assert state.battery_voltage == 28.2 and state.flags_raw == 0x20
    assert state.heartbeat_received and state.status_received and state.online
    assert {(m.channel, m.driver_id) for m in rig.drivers} == {(0, 1), (1, 2), (2, 3), (3, 4)}
    rig.emulator.send(0xB1, b"\xff" * 5)
    rig.emulator.send(0xB7, struct.pack("<Bhi", 255, 1, 2))
    rig.emulator.send(0xB2, struct.pack("<hhh", -12345, 0, 0), base=0x05E10100)
    rig.emulator.send(0xB2, struct.pack("<hhh", -12345, 0, 0), base=0x01E10200)
    rig.spin(0.6)
    assert all(m.twist.linear.x == 0 for m in rig.velocities)
    counters = {item.key: item.value for item in rig.diagnostics[-1].status[0].values}
    assert int(counters["malformed_frames"]) == 2
    assert rig.states[-1].battery_voltage == 28.2


def test_ack_and_observed_state_are_separate(rig):
    result = rig.call(Command.Request.QUERY_VERSION)
    assert result.sent and result.acknowledged and result.state_confirmed
    assert (result.hardware_version, result.software_version) == ("3.0", "4.0")
    result = rig.call(Command.Request.CLEAR_FAULTS)
    assert result.sent and result.acknowledged and not result.state_confirmed
    result = rig.call(Command.Request.ZERO_MOTION)
    assert result.sent and result.acknowledged and not result.state_confirmed
    rig.emulator.apply_enable = False
    result = rig.call(Command.Request.ENABLE, enabled=False)
    assert result.sent and result.acknowledged and not result.state_confirmed
    assert rig.states[-1].enabled
    result = rig.call(Command.Request.SET_MODE, mode=1, beep=True, brake=True)
    assert result.sent and not result.acknowledged and result.state_confirmed
    assert "brake" in result.message and "unverified" in result.message
    result = rig.call(Command.Request.SENSOR_FEEDBACK, enabled=True)
    assert result.sent and not result.acknowledged and not result.state_confirmed


def test_ack_timeout_does_not_block_motion_watchdog(rig):
    rig.emulator.ack = False
    future = rig.request(Command.Request.CLEAR_FAULTS)
    for _ in range(6):
        rig.publish()
        last_publish = time.monotonic()
        rig.spin(0.02)
    rig.wait(future.done)
    assert future.result().sent and not future.result().acknowledged
    motion = [(stamp, struct.unpack("<hhh", data)) for stamp, function, data in rig.emulator.commands
              if function == 0x12]
    positive = [(stamp, data) for stamp, data in motion if data[0] > 0]
    assert len(positive) >= 8
    zero = [(stamp, data) for stamp, data in motion if stamp > positive[-1][0] and data == (0, 0, 0)]
    assert zero and 0.25 <= zero[0][0] - last_publish <= 0.42
    assert len(rig.velocities) > 15


def test_stale_nan_commands_stop_and_are_not_replayed(rig):
    rig.publish()
    rig.spin(0.08)
    rig.publish(vx=0.2, age=5)
    rig.spin(0.1)
    commands = [struct.unpack("<hhh", data) for _, code, data in rig.emulator.commands if code == 0x12]
    assert (100, 0, 0) in commands and commands[-1] == (0, 0, 0)
    assert (200, 0, 0) not in commands
    rig.publish(vx=float("nan"))
    rig.spin(0.1)
    assert [data for _, code, data in rig.emulator.commands if code == 0x12][-1] == bytes(6)


def test_explicit_disable_is_not_overridden_and_shutdown_stops(rig):
    result = rig.call(Command.Request.ENABLE, enabled=False)
    assert result.sent and result.acknowledged and result.state_confirmed
    rig.spin(0.7)
    enables = [data[3] for _, code, data in rig.emulator.commands if code == 3]
    assert enables == [1, 0]
    assert not rig.states[-1].enabled
    rig.process.send_signal(signal.SIGINT)
    rig.process.wait(timeout=4)
    time.sleep(0.03)
    assert [data for _, code, data in rig.emulator.commands if code == 0x12][-1] == bytes(6)
    assert [data[3] for _, code, data in rig.emulator.commands if code == 3][-1] == 0


def test_interface_recovery_does_not_replay_old_motion(rig):
    rig.publish()
    rig.spin(0.08)
    start = time.monotonic()
    subprocess.run(["ip", "link", "set", rig.interface, "down"], check=True)
    try:
        rig.spin(0.5)
    finally:
        subprocess.run(["ip", "link", "set", rig.interface, "up"], check=True)
    rig.spin(1.5)
    after = [struct.unpack("<hhh", data) for stamp, code, data in rig.emulator.commands
             if stamp > start + 0.5 and code == 0x12]
    assert all(value == (0, 0, 0) for value in after)
    count = len(rig.velocities)
    rig.wait(lambda: len(rig.velocities) > count + 5)
    rig.publish(-0.1)
    rig.spin(0.08)
    assert struct.unpack("<hhh", [data for _, code, data in rig.emulator.commands if code == 0x12][-1])[0] == -100


def test_nonzero_work_state_is_reported_in_diagnostics(rig):
    rig.emulator.work_state = 1
    rig.spin(0.7)
    assert int.from_bytes(rig.diagnostics[-1].status[0].level, "little") == 2
    rig.emulator.work_state = 0
    rig.spin(0.6)
    assert int.from_bytes(rig.diagnostics[-1].status[0].level, "little") == 0
