#!/usr/bin/env python3
"""Run Noetic nodes in receive_only mode, observe IMU, and record CAN counters.

Uses its own localhost ROS master. It never calls a chassis command service.
The optional IMU NMT Start only addresses the configured IMU node, as in the driver.
"""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time


def counters(interface):
    directory = Path("/sys/class/net") / interface / "statistics"
    return {name: int((directory / name).read_text()) for name in
            ("tx_packets", "rx_packets", "tx_errors", "rx_errors")}


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can-interface", default="can0")
    parser.add_argument("--imu-interface", default="can1")
    parser.add_argument("--seconds", type=float, default=5)
    parser.add_argument("--port", type=int, default=11330)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--screenshot", type=Path)
    args = parser.parse_args()
    with socket.socket() as probe:
        if probe.connect_ex(("127.0.0.1", args.port)) == 0:
            parser.error("ROS master port is occupied; select another --port")
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:{}".format(args.port)
    os.environ["ROS_IP"] = "127.0.0.1"
    os.environ.pop("ROS_HOSTNAME", None)
    # Import after selecting the isolated master.
    import rosgraph
    import rospy
    from sensor_msgs.msg import Imu
    from diagnostic_msgs.msg import DiagnosticArray
    from std_srvs.srv import Trigger
    from vehicle_interfaces.msg import ImuStatus, State

    args.output.parent.mkdir(parents=True, exist_ok=True)
    before = counters(args.can_interface)
    report = {"utc": datetime.now(timezone.utc).isoformat(), "host": socket.gethostname(),
              "architecture": os.uname().machine, "ros_distro": os.environ.get("ROS_DISTRO"),
              "can_interface": args.can_interface, "imu_interface": args.imu_interface,
              "chassis_before": before, "receive_only": True}
    processes, subscriptions = [], []
    gui = None
    states, statuses, data, diagnostics = [], [], [], []
    started = time.monotonic()
    with args.output.with_suffix(".log").open("w") as log:
        try:
            processes.append(subprocess.Popen(["roscore", "-p", str(args.port)], stdout=log,
                                               stderr=subprocess.STDOUT))
            deadline = time.monotonic() + 10
            while not rosgraph.is_master_online():
                if time.monotonic() >= deadline:
                    raise RuntimeError("Isolated ROS master did not start")
                time.sleep(.1)
            rospy.init_node("noetic_hardware_observer", disable_signals=True)
            subscriptions = [
                rospy.Subscriber("/noetic_check/vehicle/state", State, states.append, queue_size=10),
                rospy.Subscriber("/noetic_check/imu/status", ImuStatus, statuses.append, queue_size=10),
                rospy.Subscriber("/noetic_check/imu/data", Imu,
                    lambda m: data.append((time.monotonic(), m)), queue_size=100),
                rospy.Subscriber("/noetic_check/diagnostics", DiagnosticArray,
                    diagnostics.append, queue_size=10),
            ]
            processes.append(subprocess.Popen([
                "roslaunch", "vehicle_driver", "vehicle.launch", "namespace:=noetic_check",
                "receive_only:=true", "auto_initialize:=false", "auto_imu_calibration:=false",
                "can_interface:=" + args.can_interface, "imu_can_interface:=" + args.imu_interface,
            ], stdout=log, stderr=subprocess.STDOUT))
            rospy.wait_for_service("/noetic_check/imu/zero_orientation", timeout=10)
            if args.screenshot:
                gui_env = dict(os.environ, QT_QPA_PLATFORM="offscreen", PYQTGRAPH_QT_LIB="PyQt5")
                gui = subprocess.Popen([
                    "rosrun", "vehicle_gui", "vehicle_gui", "__ns:=/noetic_check",
                    "--screenshot", str(args.screenshot.resolve()), "--quit-after", "4",
                ], env=gui_env, stdout=log, stderr=subprocess.STDOUT)
            time.sleep(args.seconds)
            if gui is not None:
                report["gui_exit_code"] = gui.wait(timeout=10)
                report["gui_screenshot_created"] = args.screenshot.is_file()
            report["receive_only_parameter"] = rospy.get_param("/noetic_check/vehicle/receive_only")
            report["vehicle_state_messages"] = len(states)
            report["vehicle_online"] = bool(states and states[-1].online)
            report["imu_status_messages"] = len(statuses)
            report["imu_online"] = bool(statuses and statuses[-1].online)
            report["imu_measurement_valid"] = bool(statuses and statuses[-1].measurement_valid)
            report["imu_orientation_valid"] = bool(statuses and statuses[-1].orientation_valid)
            report["imu_status_message"] = statuses[-1].message if statuses else "No status message"
            valid = [(t, m) for t, m in data if m.orientation_covariance[0] >= 0]
            report["imu_valid_orientation_samples"] = len(valid)
            report["imu_orientation_hz"] = ((len(valid) - 1) / (valid[-1][0] - valid[0][0])) \
                if len(valid) > 1 else 0
            if valid:
                m = valid[-1][1]
                report["quaternion_norm"] = math.sqrt(sum(getattr(m.orientation, a)**2 for a in "xyzw"))
                report["acceleration_m_s2"] = [getattr(m.linear_acceleration, a) for a in "xyz"]
                report["angular_velocity_rad_s"] = [getattr(m.angular_velocity, a) for a in "xyz"]
            zero = rospy.ServiceProxy("/noetic_check/imu/zero_orientation", Trigger)()
            report["imu_zero_success"] = zero.success
            report["imu_zero_message"] = zero.message
            if diagnostics:
                report["driver_diagnostics"] = {item.key: item.value for item in diagnostics[-1].status[0].values}
            report["nodes_running"] = all(p.poll() is None for p in processes)
        finally:
            if gui is not None:
                stop(gui)
            for subscription in subscriptions:
                subscription.unregister()
            for process in reversed(processes):
                stop(process)
            after = counters(args.can_interface)
            report["chassis_after_shutdown"] = after
            report["chassis_tx_delta"] = after["tx_packets"] - before["tx_packets"]
            report["chassis_rx_delta"] = after["rx_packets"] - before["rx_packets"]
            report["chassis_tx_zero"] = report["chassis_tx_delta"] == 0
            report["elapsed_seconds"] = time.monotonic() - started
            args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report.get("nodes_running") and report["chassis_tx_zero"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
