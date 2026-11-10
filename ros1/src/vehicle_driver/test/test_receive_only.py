"""Prove the default mode emits no chassis frames, even under active requests."""
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import Trigger, TriggerRequest
from vehicle_interfaces.msg import Heading, State
from vehicle_interfaces.srv import Command, CommandRequest, ExecuteMotion, ExecuteMotionRequest
from ros_test_support import RosProbe, executable, init_ros
from test_integration import Emulator, ImuEmulator


@pytest.mark.parametrize("stamped", [True, False])
def test_default_receive_only_blocks_all_commands_and_shutdown(tmp_path, stamped):
    interface = os.environ.get("VEHICLE_TEST_CAN")
    if not interface:
        pytest.skip("Set VEHICLE_TEST_CAN to an isolated vcan interface")
    init_ros()
    chassis = Emulator(interface)  # The emulator verifies the kernel interface type.
    chassis.enabled, chassis.mode = True, 2
    imu = ImuEmulator(interface)
    probe = RosProbe()
    states, headings, diagnostics = [], [], []
    probe.create_subscription(State, "/guard_vehicle/state", states.append, 10)
    probe.create_subscription(Heading, "/guard_vehicle/heading", headings.append, 10)
    probe.create_subscription(DiagnosticArray, "/guard_diagnostics", diagnostics.append, 10)
    publisher = probe.create_publisher(TwistStamped if stamped else Twist, "/guard_cmd_vel", 1)
    processes = []
    with (tmp_path / "receive_only.log").open("w") as log:
        try:
            processes.append(subprocess.Popen([
                executable("vehicle_node"), "__name:=guard_vehicle", f"_can_interface:={interface}",
                "_auto_initialize:=true", "_auto_imu_calibration:=true", "_publish_tf:=false",
                f"_use_stamped_cmd_vel:={str(stamped).lower()}", "cmd_vel:=guard_cmd_vel",
                "imu/data:=guard_imu_data", "diagnostics:=guard_diagnostics",
            ], stdout=log, stderr=subprocess.STDOUT))
            processes.append(subprocess.Popen([
                executable("imu_node"), "__name:=guard_imu", f"_imu_can_interface:={interface}",
                "imu/data:=guard_imu_data",
            ], stdout=log, stderr=subprocess.STDOUT))
            deadline = time.monotonic() + 6
            while time.monotonic() < deadline:
                if states and states[-1].online and headings and headings[-1].imu_active and \
                        publisher.get_subscription_count():
                    break
                time.sleep(.02)
            assert states and states[-1].online and states[-1].enabled
            assert headings and headings[-1].imu_active
            assert rospy.get_param("/guard_vehicle/auto_initialize") is True
            for _ in range(8):
                message = TwistStamped() if stamped else Twist()
                twist = message.twist if stamped else message
                twist.linear.x, twist.angular.z = .3, .4
                publisher.publish(message)
                time.sleep(.02)
            command = rospy.ServiceProxy("/guard_vehicle/command", Command)
            for code in (1, 2, 3, 4, 5, 17, 21, 22, 23, 24):
                result = command(CommandRequest(command=code, enabled=True, mode=2))
                assert not result.sent and "receive_only" in result.message
            calibration = rospy.ServiceProxy("/guard_vehicle/calibrate_imu", Trigger)(TriggerRequest())
            assert not calibration.success and "receive_only" in calibration.message
            motion = rospy.ServiceProxy("/guard_vehicle/execute_motion", ExecuteMotion)
            for mode, target in ((1, .3), (2, .4), (3, .2)):
                result = motion(ExecuteMotionRequest(mode=mode, target=target))
                assert not result.accepted and "receive_only" in result.message
            assert motion(ExecuteMotionRequest(mode=0)).accepted
            assert rospy.ServiceProxy("/guard_vehicle/reset_odometry", Trigger)(TriggerRequest()).success
            time.sleep(.6)
            values = {item.key: item.value for item in diagnostics[-1].status[0].values}
            assert values["receive_only"] == "1" and values["tx_frames"] == "0"
            assert not chassis.commands
        finally:
            for process in processes:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    process.wait(timeout=5)
            time.sleep(.05)
            commands = list(chassis.commands)
            probe.destroy_node()
            imu.close()
            chassis.close()
    assert not commands, commands  # Includes startup, service calls, control and shutdown.
