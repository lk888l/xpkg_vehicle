"""Updated firmware on the SAME kernel vcan as the real chassis ROS node."""
import math
import signal
import socket
import struct
import subprocess
import time
from types import SimpleNamespace

import pytest
import rospy
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_srvs.srv import Trigger
from vehicle_interfaces.msg import ImuStatus, MotionStatus
from vehicle_interfaces.srv import ExecuteMotionRequest
from ros_test_support import executable
from test_integration import ImuEmulator, rig


class ClassicImuEmulator(ImuEmulator):
    def __init__(self, interface):
        self.pdos = {1, 2, 3, 4}
        self.heartbeat_enabled = True
        super().__init__(interface)

    def classic(self, identifier, payload):
        self.socket.send(struct.pack('=IB3x8s', identifier, len(payload), payload))

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
                self.yaw += self.sensor_sign*self.motion_source.current_motion()[2]/1000*(now-last_physics)
            last_physics = now
            if self.heartbeat_enabled and now >= next_heartbeat:
                self.classic(0x71f, bytes([5 if self.operational else 127]))
                next_heartbeat = now + .1
            if self.operational and now >= next_data:
                q = self.quaternion or (math.cos(self.yaw/2), 0, 0, math.sin(self.yaw/2))
                if not self.valid:
                    q = (0, 0, 0, 0)
                frames = [(1, 0x19f, struct.pack('<4h', -100, 200, 1000, 2567)),
                          (3, 0x39f, struct.pack('<3h', 10, -20, -100)),
                          (4, 0x49f, struct.pack('<3h', -123, 200, 300)),
                          (2, 0x29f, struct.pack('<4h', *(round(v*10000) for v in q)))]
                for pdo, identifier, payload in frames:
                    if pdo in self.pdos:
                        self.classic(identifier, payload)
                next_data = now + .01


@pytest.fixture
def classic_rig(rig, tmp_path):
    imu = ClassicImuEmulator(rig.interface)
    data, raw, statuses, mag, temp = [], [], [], [], []
    subscriptions = [rig.node.create_subscription(kind, topic, messages.append, 50)
                     for kind, topic, messages in (
                         (Imu, '/test_imu_data', data), (Imu, '/classic/raw', raw),
                         (ImuStatus, '/classic/status', statuses),
                         (MagneticField, '/classic/magnetic_field', mag),
                         (Temperature, '/classic/temperature', temp))]
    log = (tmp_path/'classic.log').open('w')
    process = subprocess.Popen([executable('imu_node'), '__name:=classic_imu',
        '_imu_can_interface:='+rig.interface, '_imu_protocol:=classic',
        'imu/data:=test_imu_data', 'imu/raw:=classic/raw', 'imu/status:=classic/status',
        'imu/magnetic_field:=classic/magnetic_field', 'imu/temperature:=classic/temperature'],
        stdout=log, stderr=subprocess.STDOUT)
    try:
        rig.wait(lambda: statuses and statuses[-1].measurement_valid and
                 statuses[-1].orientation_valid and mag and temp and rig.headings[-1].imu_active)
        rospy.wait_for_service('/classic_imu/zero_orientation', timeout=3)
        yield SimpleNamespace(rig=rig, imu=imu, data=data, raw=raw, statuses=statuses, mag=mag, temp=temp,
                              zero=rospy.ServiceProxy('/classic_imu/zero_orientation', Trigger))
        assert process.poll() is None
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5)
        imu.close()
        for sub in subscriptions:
            rig.node.destroy_subscription(sub)
        log.close()


def test_classic_shared_bus_units_metadata_and_stationary_repeats(classic_rig):
    c = classic_rig
    c.rig.spin(.5)  # Identical sensor values are valid new packets without device counters.
    s, m = c.statuses[-1], c.data[-1]
    assert s.protocol == 'classic' and not s.device_metadata_available
    assert s.online and s.heartbeat_valid and s.measurement_valid and s.orientation_valid
    assert s.sample_count == s.quaternion_count == s.faults == s.orientation_device_age_ms == 0
    assert s.received_measurements > 30 and s.received_orientations > 15
    assert s.malformed_frames == 0 and c.imu.commands == [(1, 31)]
    assert m.linear_acceleration.x == pytest.approx(-.980665)
    assert m.linear_acceleration.z == pytest.approx(9.80665)
    assert m.angular_velocity.z == pytest.approx(-math.radians(10))
    assert c.mag[-1].magnetic_field.x == pytest.approx(-123e-7)
    assert c.temp[-1].temperature == pytest.approx(25.67)
    assert c.rig.states[-1].online and c.rig.headings[-1].imu_active


@pytest.mark.parametrize('missing,flag', [(1, 'acceleration_valid'), (3, 'angular_velocity_valid'),
                                        (4, 'magnetic_field_valid')])
def test_one_missing_classic_pdo_expires_independently(classic_rig, missing, flag):
    c = classic_rig
    c.imu.pdos.remove(missing)
    c.rig.wait(lambda: not getattr(c.statuses[-1], flag))
    s = c.statuses[-1]
    assert s.orientation_valid and c.rig.headings[-1].imu_active
    assert s.measurement_valid == (missing == 4)
    c.rig.spin(.1)
    assert (c.data[-1].linear_acceleration_covariance[0] < 0) == (missing == 1)
    assert (c.data[-1].angular_velocity_covariance[0] < 0) == (missing == 3)
    c.imu.pdos.add(missing)
    c.rig.wait(lambda: getattr(c.statuses[-1], flag) and c.statuses[-1].measurement_valid)


def test_quaternion_loss_blocks_zero_and_recovers_without_freezing_measurements(classic_rig):
    c = classic_rig
    c.imu.pdos.remove(2)
    c.rig.wait(lambda: not c.statuses[-1].orientation_valid and not c.rig.headings[-1].imu_active)
    assert c.statuses[-1].measurement_valid and not c.zero().success
    c.imu.yaw = .8
    c.imu.pdos.add(2)
    c.rig.wait(lambda: c.statuses[-1].orientation_valid and c.rig.headings[-1].imu_active)
    assert c.zero().success
    c.rig.wait(lambda: abs(c.data[-1].orientation.z) < .001)


def test_invalid_quaternion_and_wrong_dlc_cannot_refresh_heading(classic_rig):
    c = classic_rig
    c.imu.valid = False
    c.rig.wait(lambda: not c.statuses[-1].orientation_valid and not c.rig.headings[-1].imu_active)
    assert not c.zero().success
    c.imu.classic(0x19f, bytes(6))
    c.imu.socket.send(struct.pack('=IBB2x64s', 0x29f, 32, 1, bytes(32)))
    c.rig.wait(lambda: c.statuses[-1].malformed_frames >= 2)
    assert not c.statuses[-1].orientation_valid
    c.imu.valid = True
    c.rig.wait(lambda: c.statuses[-1].orientation_valid)


def test_classic_pdos_without_heartbeat_remain_online(classic_rig):
    c = classic_rig
    c.imu.heartbeat_enabled = False
    c.rig.wait(lambda: not c.statuses[-1].heartbeat_valid, timeout=3)
    assert c.statuses[-1].online and c.statuses[-1].measurement_valid and c.statuses[-1].orientation_valid


def test_classic_closed_loop_motion_and_loss_stop(classic_rig):
    c = classic_rig
    c.rig.emulator.motion_response = True
    c.imu.motion_source = c.rig.emulator
    result = c.rig.move(ExecuteMotionRequest.TURN_RELATIVE, .12)
    assert result.accepted
    c.rig.wait(lambda: c.rig.motions[-1].goal_id == result.goal_id and not c.rig.motions[-1].active, timeout=8)
    assert c.rig.motions[-1].state == MotionStatus.SUCCEEDED
    result = c.rig.move(ExecuteMotionRequest.DRIVE_DISTANCE, .3)
    assert result.accepted
    c.imu.pdos.remove(2)
    c.rig.wait(lambda: c.rig.motions[-1].goal_id == result.goal_id and
               c.rig.motions[-1].state == MotionStatus.FAILED)
    c.rig.wait(lambda: c.rig.emulator.requested_motion == (0, 0, 0))
