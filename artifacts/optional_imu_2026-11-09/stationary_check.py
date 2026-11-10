#!/usr/bin/env python3
"""One-shot stationary acceptance: enable, observe, zero goals, disable.

No nonzero speed or displacement is requested. The CAN monitor records every
local chassis motion frame so the report can verify this independently.
"""
from collections import Counter
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import threading
import time

PORT = 11331
PREFIX = '/stationary_check/vehicle/'
with socket.socket() as probe:
    if probe.connect_ex(('127.0.0.1', PORT)) == 0:
        raise SystemExit('Dedicated master port occupied')
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:{}'.format(PORT)
os.environ['ROS_IP'] = '127.0.0.1'
os.environ.pop('ROS_HOSTNAME', None)
import rosgraph
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from vehicle_interfaces.msg import Heading, MotionStatus, State
from vehicle_interfaces.srv import Command, CommandRequest, ExecuteMotion, ExecuteMotionRequest

report = {'utc': datetime.now(timezone.utc).isoformat(), 'interface': 'can0',
          'use_imu': False, 'requested_motion': 'zero targets only', 'commands': []}
output = Path('/home/orin/vehicle_noetic_ws/artifacts/optional_imu_hardware_stationary.json')
states, headings, velocities, modes, motions, diagnostics = [], [], [], [], [], []
frames, nonzero, raw_velocity = [], [], []
running = threading.Event()
running.set()
bus = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, struct.pack('=II', 0x81e10100, 0xdfffff00))
bus.bind(('can0',))
bus.settimeout(.05)

def monitor():
    while running.is_set():
        try:
            data, _, flags, _ = bus.recvmsg(72)
        except socket.timeout:
            continue
        code = struct.unpack_from('=I', data)[0] & 0xff
        if code == 0xb2:
            raw_velocity.append(list(struct.unpack_from('<hhh', data, 8)))
        if flags & socket.MSG_DONTROUTE and code < 0x80:
            frames.append(code)
            if code == 0x12 and struct.unpack_from('<hhh', data, 8) != (0, 0, 0):
                nonzero.append(data.hex())

thread = threading.Thread(target=monitor, daemon=True)
thread.start()
processes, subscriptions = [], []
enabled_attempted = False
original_mode = None
command = motion = None

def wait(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        assert all(p.poll() is None for p in processes), 'Owned ROS process exited'
        time.sleep(.02)
    assert predicate(), 'Timed out waiting for stationary feedback'

def call(code, **fields):
    result = command(CommandRequest(command=code, **fields))
    report['commands'].append({'command': code, **fields, 'sent': result.sent,
        'acknowledged': result.acknowledged, 'state_confirmed': result.state_confirmed,
        'message': result.message})
    assert result.sent and result.state_confirmed, result.message
    return result

def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)

with output.with_suffix('.log').open('w') as log:
    try:
        processes.append(subprocess.Popen(['roscore', '-p', str(PORT)], stdout=log, stderr=subprocess.STDOUT))
        wait(rosgraph.is_master_online, 10)
        rospy.init_node('stationary_acceptance', disable_signals=True)
        for message_type, topic, values in (
            (State, PREFIX + 'state', states), (Heading, PREFIX + 'heading', headings),
            (Bool, PREFIX + 'imu_enabled', modes), (MotionStatus, PREFIX + 'motion_status', motions),
            (TwistStamped, PREFIX + 'velocity', velocities),
            (DiagnosticArray, '/stationary_check/diagnostics', diagnostics)):
            subscriptions.append(rospy.Subscriber(topic, message_type, values.append, queue_size=100))
        processes.append(subprocess.Popen(['roslaunch', 'vehicle_driver', 'vehicle.launch',
            'namespace:=stationary_check', 'can_interface:=can0', 'use_imu:=false',
            'receive_only:=false', 'auto_initialize:=false', 'auto_imu_calibration:=false'],
            stdout=log, stderr=subprocess.STDOUT))
        rospy.wait_for_service(PREFIX + 'command', timeout=10)
        command = rospy.ServiceProxy(PREFIX + 'command', Command)
        motion = rospy.ServiceProxy(PREFIX + 'execute_motion', ExecuteMotion)
        wait(lambda: states and states[-1].online and modes)
        assert not states[-1].enabled, 'Refuse to interrupt a previously enabled chassis'
        assert modes[-1].data is False
        report['initial_enabled'] = states[-1].enabled
        version = call(2)
        report['version'] = [version.hardware_version, version.software_version]
        assert motion(ExecuteMotionRequest(mode=0)).accepted
        enabled_attempted = True
        call(3, enabled=True)
        wait(lambda: states[-1].status_received)
        original_mode = states[-1].mode
        report['original_mode'] = original_mode
        call(17, mode=2, beep=False)
        publisher = rospy.Publisher('/stationary_check/cmd_vel', TwistStamped, queue_size=1)
        wait(lambda: publisher.get_num_connections() > 0)
        for _ in range(60):
            publisher.publish(TwistStamped())
            time.sleep(.05)
        publisher.unregister()
        wait(lambda: len(velocities) >= 20 and headings and diagnostics)
        report['last_velocity_samples'] = [[v.twist.linear.x, v.twist.angular.z] for v in velocities[-20:]]
        report['max_abs_velocity'] = max(abs(v.twist.linear.x) for v in velocities)
        report['max_abs_yaw_rate'] = max(abs(v.twist.angular.z) for v in velocities)
        wait(lambda: any(k.key == 'motion_feedback_ready' and k.value == '1'
                         for k in diagnostics[-1].status[0].values))
        report['ready_without_imu'] = True
        report['driver_diagnostics'] = {k.key: k.value for k in diagnostics[-1].status[0].values}
        assert all(abs(v.twist.linear.x) < .005 and abs(v.twist.angular.z) < .005 for v in velocities[-20:]), 'Stationary feedback exceeds threshold; zero-goal tests skipped'
        report['zero_goals'] = []
        for mode in (1, 3):
            result = motion(ExecuteMotionRequest(mode=mode, target=0.0, client_timeout=0.0))
            assert result.accepted, result.message
            wait(lambda: motions and motions[-1].goal_id == result.goal_id and not motions[-1].active)
            report['zero_goals'].append({'mode': mode, 'accepted': result.accepted,
                'state': motions[-1].state, 'message': motions[-1].message})
            assert motions[-1].state == MotionStatus.SUCCEEDED
        assert not any(h.imu_active for h in headings)
        report['heading_samples'] = len(headings)
        report['success'] = True
    except Exception as error:
        report['success'] = False
        report['error'] = str(error)
    finally:
        if enabled_attempted and command is not None:
            try:
                motion(ExecuteMotionRequest(mode=0))
                if original_mode is not None and original_mode != 2:
                    call(17, mode=original_mode, beep=False)
                call(3, enabled=False)
                wait(lambda: states and states[-1].online and not states[-1].enabled)
                report['final_disabled_confirmed'] = True
            except Exception as error:
                report['cleanup_error'] = str(error)
                report['success'] = False
        for sub in subscriptions:
            sub.unregister()
        for process in reversed(processes):
            stop(process)
        time.sleep(.1)
        running.clear()
        thread.join(timeout=1)
        bus.close()
        report['local_command_counts'] = {hex(k): v for k, v in Counter(frames).items()}
        report['nonzero_motion_frames'] = nonzero
        report['raw_b2_samples'] = raw_velocity[-20:]
        report['success'] = bool(report.get('success') and report.get('final_disabled_confirmed') and not nonzero)
        output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
print(json.dumps(report, indent=2, ensure_ascii=False))
raise SystemExit(0 if report['success'] else 1)
