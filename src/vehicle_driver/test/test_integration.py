"""Real SocketCAN + DDS integration against an isolated virtual vehicle.

Set VEHICLE_TEST_CAN=vcan_vehicle. Only a kernel vcan device is accepted.
The emulator never connects to a physical CAN interface.
"""
import errno
import json
import math
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
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_sensor_data
from vehicle_interfaces.msg import DriverMotion, Heading, ImuCalibration, ImuStatus, MotionStatus, State, Version
from vehicle_interfaces.srv import Command, ExecuteMotion

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
        self.motion_response = False
        self.requested_motion = (0, 0, 0)
        self.motion_time = 0.0
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
                elif function == 0x12:
                    self.requested_motion = struct.unpack("<hhh", payload[:6])
                    self.motion_time = time.monotonic()
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
                self.send(0xB2, struct.pack("<hhh", *self.current_motion()))
                self.send(0xB3, struct.pack("<ii", 0, 0))
                self.send(0xB7, struct.pack("<Bhi", channel, -10, -100))
                channel = (channel + 1) % 4

    def current_motion(self):
        if self.motion_response and self.enabled and self.mode == 2 and time.monotonic() - self.motion_time < .5:
            return self.requested_motion
        return (0, 0, 0)

    def close(self):
        self.running = False
        self.thread.join(timeout=2)
        self.socket.close()


class ImuEmulator:
    """The firmware's two default 32-byte PDOs, on the same isolated vcan only."""
    def __init__(self, interface):
        info = json.loads(subprocess.check_output(["ip", "-json", "-details", "link", "show", interface]))
        assert info[0]["linkinfo"]["info_kind"] == "vcan"
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
        self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, struct.pack("=II", 0, 0xC00007FF))
        self.socket.bind((interface,))
        self.socket.settimeout(.002)
        self.running, self.operational, self.frozen, self.valid = True, False, False, True
        self.counter, self.yaw, self.commands = 1000, 0.0, []
        self.quaternion = None
        self.motion_source = None
        self.sensor_sign = 1
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        next_heartbeat = next_data = 0
        last_physics = time.monotonic()
        while self.running:
            try:
                raw = self.socket.recv(72)
                if len(raw) == 16 and raw[4] == 2:
                    command, node = raw[8:10]
                    self.commands.append((command, node))
                    if command == 1 and node == 31:
                        self.operational = True
            except socket.timeout:
                pass
            now = time.monotonic()
            if self.motion_source is not None:
                self.yaw += self.sensor_sign * self.motion_source.current_motion()[2] / 1000 * (now - last_physics)
            last_physics = now
            if now >= next_heartbeat:
                self.socket.send(struct.pack("=IB3x8s", 0x71F, 1, bytes([5 if self.operational else 127])))
                next_heartbeat = now + .1
            if self.operational and now >= next_data:
                if not self.frozen:
                    self.counter = (self.counter + 1) & 0xFFFFFFFF
                orientation = self.quaternion or (math.cos(self.yaw/2), 0, 0, math.sin(self.yaw/2))
                q = tuple(round(value * 10000) for value in orientation)
                raw = struct.pack("<10hIBI3x", 0, 0, 1000, 0, 0, -100, 100, 200, 300, 2500, self.counter, 3, 0)
                chip = struct.pack("<10hIIB3x", *q, 0, 0, 1000, 0, 0, 0, self.counter, 0, int(self.valid))
                self.socket.send(struct.pack("=IBB2x64s", 0x19F, 32, 1, raw))
                self.socket.send(struct.pack("=IBB2x64s", 0x29F, 32, 1, chip))
                next_data = now + .008

    def close(self):
        self.running = False
        self.thread.join(timeout=2)
        self.socket.close()


class Rig:
    def __init__(self, interface, directory, auto_calibrate=False):
        self.interface = interface
        self.emulator = Emulator(interface)
        self.node = rclpy.create_node("vehicle_integration_probe")
        self.states, self.velocities, self.drivers, self.diagnostics, self.versions = [], [], [], [], []
        self.headings, self.odometry = [], []
        self.calibrations = []
        self.motions = []
        self.subscriptions = [
            self.node.create_subscription(State, "/test_vehicle/state", self.states.append, 10),
            self.node.create_subscription(TwistStamped, "/test_vehicle/velocity",
                                          self.velocities.append, qos_profile_sensor_data),
            self.node.create_subscription(DriverMotion, "/test_vehicle/driver_motion",
                                          self.drivers.append, qos_profile_sensor_data),
            self.node.create_subscription(DiagnosticArray, "/test_diagnostics", self.diagnostics.append, 10),
            self.node.create_subscription(Version, "/test_vehicle/version", self.versions.append, 10),
            self.node.create_subscription(Heading, "/test_vehicle/heading", self.headings.append, 10),
            self.node.create_subscription(ImuCalibration, "/test_vehicle/imu_calibration", self.calibrations.append, 10),
            self.node.create_subscription(MotionStatus, "/test_vehicle/motion_status", self.motions.append, 10),
            self.node.create_subscription(Odometry, "/test_odom", self.odometry.append, 10),
        ]
        self.publisher = self.node.create_publisher(TwistStamped, "/test_cmd_vel", 1)
        self.client = self.node.create_client(Command, "/test_vehicle/command")
        self.motion_client = self.node.create_client(ExecuteMotion, "/test_vehicle/execute_motion")
        executable = Path(get_package_prefix("vehicle_driver")) / "lib/vehicle_driver/vehicle_node"
        self.log_path = directory / "node.log"
        self.log_file = self.log_path.open("w")
        arguments = [
            str(executable), "--ros-args", "-r", "__node:=test_vehicle",
            "-r", "cmd_vel:=test_cmd_vel", "-r", "odom:=test_odom",
            "-r", "diagnostics:=test_diagnostics", "-p", f"can_interface:={interface}",
            "-r", "imu/data:=test_imu_data",
            "-p", "publish_tf:=false", "-p", "service_timeout:=0.6",
        ]
        if auto_calibrate is not None:
            arguments.extend(["-p", f"auto_imu_calibration:={str(auto_calibrate).lower()}"])
        self.process = subprocess.Popen(arguments, stdout=self.log_file, stderr=subprocess.STDOUT)

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

    def move(self, mode, target=0.0, **fields):
        self.wait(lambda: self.motion_client.service_is_ready())
        future = self.motion_client.call_async(ExecuteMotion.Request(mode=mode, target=target, **fields))
        self.wait(future.done)
        return future.result()

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


@pytest.fixture
def imu_rig(rig, tmp_path, request):
    emulator = ImuEmulator(rig.interface)
    data, statuses = [], []
    subscriptions = [rig.node.create_subscription(Imu, "/test_imu_data", data.append, 10),
                     rig.node.create_subscription(ImuStatus, "/test_imu_status", statuses.append, 10)]
    binary = Path(get_package_prefix("vehicle_driver")) / "lib/vehicle_driver/imu_node"
    mounting = getattr(request, "param", [0.0, 0.0, 0.0])
    rig.zero_imu_client = rig.node.create_client(Trigger, "/test_imu/zero_orientation")
    with (tmp_path / "imu.log").open("w") as output:
        process = subprocess.Popen([str(binary), "--ros-args", "-r", "__node:=test_imu",
            "-r", "imu/data:=test_imu_data", "-r", "imu/status:=test_imu_status",
            "-p", f"imu_can_interface:={rig.interface}", "-p", f"imu_mount_rpy:={mounting}"],
            stdout=output, stderr=subprocess.STDOUT)
        try:
            rig.wait(lambda: data and data[-1].orientation_covariance[0] > 0 and
                     rig.headings and rig.headings[-1].imu_active and rig.zero_imu_client.service_is_ready())
            yield rig, emulator, data, statuses
        finally:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5)
            emulator.close()
            rig.node.destroy_client(rig.zero_imu_client)
            for subscription in subscriptions:
                rig.node.destroy_subscription(subscription)


def test_motion_service_rejects_missing_imu_and_accepts_stop(rig):
    assert not rig.move(ExecuteMotion.Request.TURN_RELATIVE, .2).accepted
    assert rig.move(ExecuteMotion.Request.CANCEL).accepted
    rig.spin(.1)
    assert all(struct.unpack("<hhh", data) == (0, 0, 0)
               for _, function, data in rig.emulator.commands if function == 0x12)


def test_headless_motion_service_turns_and_drives_to_distance(imu_rig):
    rig, imu, _, _ = imu_rig
    rig.emulator.motion_response = True
    imu.motion_source = rig.emulator
    for mode, target in ((ExecuteMotion.Request.TURN_RELATIVE, -.2),
                         (ExecuteMotion.Request.TURN_HEADING, .1),
                         (ExecuteMotion.Request.DRIVE_DISTANCE, .08)):
        result = rig.move(mode, target)
        assert result.accepted, result.message
        rig.wait(lambda: rig.motions and rig.motions[-1].goal_id == result.goal_id and
                 not rig.motions[-1].active, timeout=8)
        status = rig.motions[-1]
        assert status.state == MotionStatus.SUCCEEDED, status.message
        assert abs(status.remaining) <= (.01 if mode == 3 else math.radians(1))
        assert status.feedforward_gain == pytest.approx(1.308149183334789)
        rig.wait(lambda: rig.emulator.requested_motion == (0, 0, 0))


def test_motion_lease_stale_imu_invalid_goal_and_manual_override(imu_rig):
    rig, imu, _, _ = imu_rig
    accepted = rig.move(1, .5, client_timeout=.2)
    assert accepted.accepted
    rig.wait(lambda: rig.motions[-1].goal_id == accepted.goal_id and
             rig.motions[-1].state == MotionStatus.FAILED)
    assert "续期" in rig.motions[-1].message
    accepted = rig.move(1, .5)
    assert accepted.accepted
    assert not rig.move(1, float("nan")).accepted
    rig.wait(lambda: rig.motions[-1].goal_id == accepted.goal_id and rig.motions[-1].active)
    imu.frozen = True
    rig.wait(lambda: rig.motions[-1].state == MotionStatus.FAILED)
    assert "反馈" in rig.motions[-1].message
    imu.frozen = False
    rig.wait(lambda: rig.headings[-1].imu_active)
    accepted = rig.move(3, .5)
    assert accepted.accepted
    rig.publish(vx=.12, wz=0.0)
    rig.wait(lambda: rig.motions[-1].goal_id == accepted.goal_id and
             rig.motions[-1].state == MotionStatus.CANCELED)
    rig.wait(lambda: any(struct.unpack("<hhh", data) == (120, 0, 0)
                         for _, function, data in rig.emulator.commands if function == 0x12))
    rig.publish(vx=0.0)


def test_imu_fd_nmt_units_and_fused_turn_path(imu_rig):
    rig, imu, data, statuses = imu_rig
    rig.wait(lambda: statuses and statuses[-1].orientation_valid)
    assert imu.commands == [(1, 31)]
    assert abs(data[-1].linear_acceleration.z - 9.80665) < 1e-6
    assert abs(data[-1].angular_velocity.z + math.radians(10)) < 1e-6
    start = rig.headings[-1].yaw
    for i in range(1, 31):
        imu.yaw = i * .02
        rig.emulator.send(0xB2, struct.pack("<hhh", 200, 0, 0))
        rig.spin(.02)
    rig.wait(lambda: abs(rig.headings[-1].yaw - start - .6) < .03)
    assert rig.odometry[-1].pose.pose.position.y > 0
    assert rig.headings[-1].imu_active
    assert statuses[-1].malformed_frames == 0


def test_imu_duplicate_expiry_invalid_flags_and_reboot_do_not_jump_pose(imu_rig):
    rig, imu, data, statuses = imu_rig
    imu.frozen = True
    rig.spin(.4)
    assert not rig.headings[-1].imu_active
    assert not statuses[-1].orientation_valid
    imu.frozen = False
    rig.wait(lambda: rig.headings[-1].imu_active)
    before = rig.headings[-1].yaw
    imu.valid = False
    rig.wait(lambda: not rig.headings[-1].imu_active)
    imu.yaw = 2.0
    imu.valid = True
    rig.wait(lambda: rig.headings[-1].imu_active)
    assert abs(rig.headings[-1].yaw - before) < .01
    imu.counter, imu.yaw = 0, -2.0
    rig.spin(.2)
    assert rig.headings[-1].imu_active
    assert abs(rig.headings[-1].yaw - before) < .01


def zero_imu(rig):
    future = rig.zero_imu_client.call_async(Trigger.Request())
    rig.wait(future.done)
    return future.result()


def orientation(message):
    q = message.orientation
    return (q.w, q.x, q.y, q.z)


def test_imu_zero_preserves_pose_and_sample_age_then_tracks_relative_rotation(imu_rig):
    rig, imu, data, _ = imu_rig
    imu.yaw = .9
    rig.wait(lambda: abs(rig.headings[-1].yaw - .9) < .01)
    imu.frozen = True
    rig.spin(.03)
    before = data[-1]
    pose = rig.odometry[-1].pose.pose
    heading = rig.headings[-1].yaw
    start = len(data)
    assert zero_imu(rig).success
    rig.wait(lambda: len(data) >= start + 2)
    assert data[start].orientation_covariance[0] == -1
    zero = data[start + 1]
    assert orientation(zero) == pytest.approx((1, 0, 0, 0), abs=1e-6)
    assert zero.header.stamp == before.header.stamp
    assert zero.angular_velocity == before.angular_velocity
    assert zero.linear_acceleration == before.linear_acceleration
    rig.spin(.05)
    assert rig.odometry[-1].pose.pose == pose
    assert rig.headings[-1].yaw == pytest.approx(heading, abs=1e-6)
    imu.frozen = False
    imu.yaw = 1.2
    rig.wait(lambda: abs(rig.headings[-1].yaw - heading - .3) < .01)
    assert orientation(data[-1]) == pytest.approx((math.cos(.15), 0, 0, math.sin(.15)), abs=.0002)
    assert zero_imu(rig).success
    rig.wait(lambda: abs(data[-1].orientation.z) < .0002)
    assert rig.headings[-1].yaw == pytest.approx(heading + .3, abs=.01)
    assert imu.commands == [(1, 31)]  # Zeroing adds no firmware command/reset.


@pytest.mark.parametrize("imu_rig", [[0.0, 0.0, math.pi / 2]], indirect=True)
def test_imu_zero_handles_tilt_and_mounting_rotation(imu_rig):
    rig, imu, data, _ = imu_rig
    imu.quaternion = (.5, .5, .5, .5)
    rig.spin(.08)
    assert zero_imu(rig).success
    rig.wait(lambda: abs(data[-1].orientation.w - 1) < 1e-6)
    assert orientation(data[-1]) == pytest.approx((1, 0, 0, 0), abs=1e-6)
    # A 90-degree sensor-local X turn is base-local Y with this mounting.
    root_half = math.sqrt(.5)
    imu.quaternion = (0, root_half, root_half, 0)
    rig.wait(lambda: abs(data[-1].orientation.y - root_half) < .0002)
    assert orientation(data[-1]) == pytest.approx((root_half, 0, root_half, 0), abs=.0002)
    assert zero_imu(rig).success
    rig.wait(lambda: abs(data[-1].orientation.w - 1) < 1e-6)
    assert orientation(data[-1]) == pytest.approx((1, 0, 0, 0), abs=1e-6)


def test_imu_zero_rejects_invalid_and_stale_data_and_clears_reference_on_reboot(imu_rig):
    rig, imu, data, statuses = imu_rig
    imu.yaw = .8
    rig.spin(.08)
    assert zero_imu(rig).success
    imu.valid = False
    rig.wait(lambda: data[-1].orientation_covariance[0] < 0)
    assert not zero_imu(rig).success
    imu.yaw, imu.valid = 1.0, True
    rig.wait(lambda: data[-1].orientation_covariance[0] > 0 and
             abs(data[-1].orientation.z - math.sin(.1)) < .0002)
    rig.wait(lambda: statuses and statuses[-1].orientation_valid)
    imu.frozen = True
    rig.wait(lambda: statuses and not statuses[-1].orientation_valid)
    start = len(data)
    assert not zero_imu(rig).success
    rig.spin(.03)
    assert len(data) == start
    imu.yaw, imu.frozen = 1.1, False
    rig.wait(lambda: abs(data[-1].orientation.z - math.sin(.15)) < .0002)
    rig.wait(lambda: rig.headings[-1].imu_active)
    rig.spin(.04)  # Let odometry consume the changed sample before the reboot.
    heading = rig.headings[-1].yaw
    imu.counter, imu.yaw = 0, -.7
    rig.wait(lambda: abs(data[-1].orientation.z - math.sin(-.35)) < .0002)
    rig.spin(.06)
    assert rig.headings[-1].yaw == pytest.approx(heading, abs=.01)


@pytest.fixture
def calibration_rig(request, tmp_path):
    interface = os.environ.get("VEHICLE_TEST_CAN")
    if not interface:
        pytest.skip("Set VEHICLE_TEST_CAN to an isolated vcan interface")
    rclpy.init()
    rig = Rig(interface, tmp_path, auto_calibrate=None)  # Exercise the actual startup default.
    rig.emulator.motion_response = True
    imu = ImuEmulator(interface)
    imu.sensor_sign = getattr(request, "param", -1)
    imu.motion_source = rig.emulator
    binary = Path(get_package_prefix("vehicle_driver")) / "lib/vehicle_driver/imu_node"
    with (tmp_path / "calibration_imu.log").open("w") as output:
        process = subprocess.Popen([str(binary), "--ros-args", "-r", "__node:=test_imu",
            "-r", "imu/data:=test_imu_data", "-p", f"imu_can_interface:={interface}"],
            stdout=output, stderr=subprocess.STDOUT)
        try:
            yield rig, imu
        finally:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5)
            imu.close(); rig.close(); rclpy.shutdown()


@pytest.mark.parametrize("calibration_rig", [1, -1], indirect=True)
def test_startup_auto_calibrates_both_signs_then_applies_corrected_heading(calibration_rig):
    rig, imu = calibration_rig
    rig.wait(lambda: rig.calibrations and rig.calibrations[-1].state == ImuCalibration.SUCCEEDED, timeout=8)
    result = rig.calibrations[-1]
    assert result.calibrated and result.yaw_sign == imu.sensor_sign
    assert result.positive_delta * result.negative_delta < 0
    motion = [struct.unpack("<hhh", payload) for _, code, payload in rig.emulator.commands if code == 0x12]
    assert any(wz > 0 for _, _, wz in motion) and any(wz < 0 for _, _, wz in motion)
    assert all(vx == vy == 0 for vx, vy, _ in motion)
    assert motion[-1] == (0, 0, 0)
    assert not any(code in (1, 5) for _, code, _ in rig.emulator.commands)
    assert abs(imu.yaw) < .04  # Return near the initial direction, with no odometry reset.
    rig.wait(lambda: rig.headings[-1].imu_active)
    before = rig.headings[-1].yaw
    assert abs(before) < .06
    imu.yaw += imu.sensor_sign * .3
    rig.wait(lambda: rig.headings[-1].yaw - before > .25)
    assert abs(rig.headings[-1].yaw - before - .3) < .015


def test_calibration_imu_loss_stops_without_retry_and_service_can_restart(calibration_rig):
    rig, imu = calibration_rig
    rig.wait(lambda: rig.calibrations and rig.calibrations[-1].state == ImuCalibration.POSITIVE)
    imu.frozen = True
    rig.wait(lambda: rig.calibrations[-1].state == ImuCalibration.FAILED)
    assert not rig.calibrations[-1].calibrated and rig.calibrations[-1].yaw_sign == 1
    failed = time.monotonic()
    imu.frozen = False
    rig.spin(.4)
    assert rig.calibrations[-1].state == ImuCalibration.FAILED
    assert not rig.headings[-1].imu_active  # Never close the loop with an unidentified sign.
    assert all(struct.unpack("<hhh", payload) == (0, 0, 0)
               for stamp, code, payload in rig.emulator.commands if stamp > failed and code == 0x12)
    client = rig.node.create_client(Trigger, "/test_vehicle/calibrate_imu")
    rig.wait(client.service_is_ready)
    future = client.call_async(Trigger.Request())
    rig.wait(future.done)
    assert future.result().success
    rig.wait(lambda: rig.calibrations[-1].state == ImuCalibration.SUCCEEDED, timeout=8)
    assert rig.calibrations[-1].yaw_sign == -1


@pytest.mark.parametrize("motion", [(0.0, 0.0), (.1, -.2)])
def test_calibration_keyboard_command_cancels_and_takes_over(calibration_rig, motion):
    rig, _ = calibration_rig
    rig.wait(lambda: rig.calibrations and rig.calibrations[-1].state == ImuCalibration.POSITIVE)
    rig.publish(*motion)
    rig.wait(lambda: rig.calibrations[-1].state == ImuCalibration.CANCELED)
    rig.wait(lambda: rig.emulator.requested_motion == (round(motion[0]*1000), 0, round(motion[1]*1000)))
    rig.spin(.5)
    assert rig.calibrations[-1].state == ImuCalibration.CANCELED
    assert rig.emulator.requested_motion == (0, 0, 0)
    assert not rig.headings[-1].imu_active
