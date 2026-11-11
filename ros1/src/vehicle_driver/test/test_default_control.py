"""Exercise the actual default startup entries through SocketCAN and TCPROS."""
import os
import signal
import struct
import subprocess
import time

import pytest
import rospy
from geometry_msgs.msg import TwistStamped
from vehicle_interfaces.msg import State
from vehicle_interfaces.srv import Command, CommandRequest
from ros_test_support import RosProbe, executable, init_ros
from test_integration import Emulator


@pytest.mark.parametrize("entry", ["node", "driver", "gui"])
def test_default_allows_explicit_enable_mode_and_motion(tmp_path, entry):
    interface = os.environ.get("VEHICLE_TEST_CAN")
    if not interface:
        pytest.skip("Set VEHICLE_TEST_CAN to an isolated vcan interface")
    init_ros()
    namespace = "/defaults_check"
    if rospy.has_param(namespace):
        rospy.delete_param(namespace)  # Do not let a previous entry supply its defaults.
    chassis = Emulator(interface)
    probe = RosProbe()
    states = []
    probe.create_subscription(State, namespace + "/vehicle/state", states.append, 10)
    publisher = probe.create_publisher(TwistStamped, namespace + "/cmd_vel", 1)
    if entry == "node":
        args = [executable("vehicle_node"), "__ns:=" + namespace,
                "_can_interface:=" + interface, "_use_imu_heading:=false"]
    else:
        package, launch = ("vehicle_driver", "vehicle.launch") if entry == "driver" else \
                          ("vehicle_gui", "console.launch")
        args = ["roslaunch", package, launch, "namespace:=" + namespace,
                "can_interface:=" + interface, "use_imu:=false"]
    # No receive_only or auto_initialize overrides: exercise shipped defaults.
    env = dict(os.environ, QT_QPA_PLATFORM="offscreen", PYQTGRAPH_QT_LIB="PyQt5")
    with (tmp_path / "default_control.log").open("w") as log:
        process = subprocess.Popen(args, env=env, stdout=log, stderr=subprocess.STDOUT)

        def wait(predicate, timeout=8):
            until = time.monotonic() + timeout
            while not predicate() and time.monotonic() < until:
                assert process.poll() is None
                time.sleep(.02)
            assert predicate()

        try:
            wait(lambda: states and states[-1].online and publisher.get_subscription_count())
            assert not chassis.enabled  # Enabling is still an explicit operator action.
            command = rospy.ServiceProxy(namespace + "/vehicle/command", Command)
            rospy.wait_for_service(namespace + "/vehicle/command", timeout=5)
            enabled = command(CommandRequest(command=3, enabled=True))
            assert enabled.sent and enabled.state_confirmed, enabled.message
            mode = command(CommandRequest(command=17, mode=2))
            assert mode.sent and mode.state_confirmed, mode.message
            before = len(chassis.commands)
            message = TwistStamped()
            message.twist.linear.x = .05
            for _ in range(5):
                publisher.publish(message)
                time.sleep(.04)
            wait(lambda: any(code == 0x12 and struct.unpack("<hhh", data) == (50, 0, 0)
                             for _, code, data in chassis.commands[before:]))
            publisher.publish(TwistStamped())
            wait(lambda: chassis.requested_motion == (0, 0, 0))
            disabled = command(CommandRequest(command=3, enabled=False))
            assert disabled.sent and disabled.state_confirmed, disabled.message
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=5)
            probe.destroy_node()
            chassis.close()
