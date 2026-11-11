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
import struct
import threading


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
    parser.add_argument("--imu-interface", help="Defaults to --can-interface for the shared bus")
    parser.add_argument("--use-imu", choices=("true", "false"), default="true",
                        help="Select the same startup mode as use_imu in roslaunch")
    parser.add_argument("--seconds", type=float, default=5)
    parser.add_argument("--port", type=int, default=11330)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--screenshot", type=Path)
    args = parser.parse_args()
    use_imu = args.use_imu == "true"
    args.imu_interface = args.imu_interface or args.can_interface
    with socket.socket() as probe:
        if probe.connect_ex(("127.0.0.1", args.port)) == 0:
            parser.error("ROS master port is occupied; select another --port")
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:{}".format(args.port)
    os.environ["ROS_IP"] = "127.0.0.1"
    os.environ.pop("ROS_HOSTNAME", None)
    # Import after selecting the isolated master.
    import rosgraph
    import rosnode
    import rospy
    from sensor_msgs.msg import Imu
    from diagnostic_msgs.msg import DiagnosticArray
    from std_srvs.srv import Trigger
    from std_msgs.msg import Bool
    from vehicle_interfaces.msg import Heading, ImuStatus, State

    args.output.parent.mkdir(parents=True, exist_ok=True)
    before = counters(args.can_interface)
    report = {"utc": datetime.now(timezone.utc).isoformat(), "host": socket.gethostname(),
              "architecture": os.uname().machine, "ros_distro": os.environ.get("ROS_DISTRO"),
              "can_interface": args.can_interface, "imu_interface": args.imu_interface,
              "can_before": before, "receive_only": True, "use_imu": use_imu}
    processes, subscriptions = [], []
    gui = None
    states, statuses, data, diagnostics = [], [], [], []
    modes, headings = [], []
    # Network TX counters include IMU NMT on a shared bus. Observe local chassis
    # extended command frames separately, including shutdown, without sending.
    observed_chassis_tx = []
    monitoring = threading.Event()
    monitoring.set()
    monitor_bus = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    monitor_bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                          struct.pack('=II', 0x81e10100, 0xdfffff00))
    monitor_bus.bind((args.can_interface,))
    monitor_bus.settimeout(.05)
    def monitor():
        while monitoring.is_set():
            try:
                frame, _, flags, _ = monitor_bus.recvmsg(72)
                identifier = struct.unpack_from('=I', frame)[0]
                if flags & socket.MSG_DONTROUTE and (identifier & 0xff) < 0x80:
                    observed_chassis_tx.append(frame.hex())
            except socket.timeout:
                pass
    monitor_thread = threading.Thread(target=monitor, daemon=True)
    monitor_thread.start()
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
                rospy.Subscriber("/noetic_check/vehicle/imu_enabled", Bool, modes.append, queue_size=1),
                rospy.Subscriber("/noetic_check/vehicle/heading", Heading, headings.append, queue_size=10),
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
                "use_imu:=" + args.use_imu,
            ], stdout=log, stderr=subprocess.STDOUT))
            rospy.wait_for_service("/noetic_check/vehicle/command", timeout=10)
            if use_imu:
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
            report["vehicle_enabled"] = bool(states and states[-1].enabled)
            report["effective_imu_enabled"] = modes[-1].data if modes else None
            report["imu_node_running"] = "/noetic_check/imu" in rosnode.get_node_names()
            report["heading_samples"] = len(headings)
            report["heading_imu_active"] = bool(headings and headings[-1].imu_active)
            report["imu_status_messages"] = len(statuses)
            report["imu_online"] = bool(statuses and statuses[-1].online)
            report["imu_measurement_valid"] = bool(statuses and statuses[-1].measurement_valid)
            report["imu_orientation_valid"] = bool(statuses and statuses[-1].orientation_valid)
            report["imu_protocol"] = statuses[-1].protocol if statuses else "unknown"
            report["imu_device_metadata_available"] = bool(statuses and statuses[-1].device_metadata_available)
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
            if use_imu:
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
            report["can_after_shutdown"] = after
            report["can_tx_delta"] = after["tx_packets"] - before["tx_packets"]
            report["can_rx_delta"] = after["rx_packets"] - before["rx_packets"]
            time.sleep(.1)
            monitoring.clear()
            monitor_thread.join(timeout=1)
            monitor_bus.close()
            report["observed_chassis_command_frames"] = observed_chassis_tx
            report["chassis_tx_zero"] = not observed_chassis_tx and \
                report.get("driver_diagnostics", {}).get("tx_frames") == "0"
            report["elapsed_seconds"] = time.monotonic() - started
            args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if not report.get("nodes_running") or not report["chassis_tx_zero"]:
        return 1
    if not use_imu:
        return 0 if report.get("vehicle_online") and report["effective_imu_enabled"] is False and \
                    not report["imu_node_running"] and not report["heading_imu_active"] else 2
    return 0 if report.get("vehicle_online") and report.get("imu_measurement_valid") and \
                report.get("imu_orientation_valid") else 2


if __name__ == "__main__":
    raise SystemExit(main())
