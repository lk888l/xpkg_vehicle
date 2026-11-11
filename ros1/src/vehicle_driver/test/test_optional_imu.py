"""Startup composition and actual no-IMU control on isolated vcan/TCPROS."""
import os
import subprocess

import pytest
import rosgraph
import roslaunch
import rospy
import yaml
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from vehicle_interfaces.msg import MotionStatus
from ros_test_support import init_ros
from test_integration import Rig


@pytest.fixture
def chassis_rig(tmp_path):
    interface = os.environ.get("VEHICLE_TEST_CAN")
    if not interface:
        pytest.skip("Set VEHICLE_TEST_CAN to an isolated vcan interface")
    init_ros()
    # Even a leftover automatic calibration setting must not block this mode.
    rig = Rig(interface, tmp_path, auto_calibrate=True, use_imu=False)
    try:
        rig.wait(lambda: rig.states and rig.states[-1].enabled and
                 rig.states[-1].mode == 2 and rig.headings)
        yield rig
    finally:
        rig.close()


def test_no_imu_completes_turn_heading_and_distance(chassis_rig):
    rig = chassis_rig
    rig.emulator.motion_response = True
    for mode, target in ((1, -.2), (2, .1), (3, .08)):
        result = rig.move(mode, target)
        assert result.accepted, result.message
        rig.wait(lambda: rig.motions and rig.motions[-1].goal_id == result.goal_id and
                 not rig.motions[-1].active, timeout=8)
        status = rig.motions[-1]
        assert status.state == MotionStatus.SUCCEEDED, status.message
        assert abs(status.remaining) <= (.01 if mode == 3 else .018)
        assert not rig.headings[-1].imu_active and rig.headings[-1].imu_age == -1
        rig.wait(lambda: rig.emulator.requested_motion == (0, 0, 0))


def test_no_imu_ignores_external_sensor_and_runtime_parameter_edits(chassis_rig):
    rig = chassis_rig
    publisher = rig.node.create_publisher(Imu, "/test_imu_data", 1)
    # A late subscriber sees the effective startup value, not ROS parameter edits.
    modes = []
    rig.node.create_subscription(Bool, "/test_vehicle/imu_enabled", modes.append, 1)
    rig.wait(lambda: modes and rig.calibrations)
    assert modes[-1].data is False
    assert rig.calibrations[-1].state == 0
    assert not rospy.ServiceProxy("/test_vehicle/calibrate_imu", Trigger)().success
    rospy.set_param("/test_vehicle/use_imu_heading", True)
    try:
        message = Imu()
        message.orientation.w = .70710678
        message.orientation.z = .70710678
        for _ in range(15):
            message.header.stamp = rospy.Time.now()
            publisher.publish(message)
            rig.spin(.02)
        assert publisher.get_subscription_count() == 0
        subscriptions = rosgraph.Master(rospy.get_name()).getSystemState()[1]
        assert not any(topic == "/test_imu_data" and "/test_vehicle" in nodes
                       for topic, nodes in subscriptions)
        assert not rig.headings[-1].imu_active
        assert abs(rig.headings[-1].yaw) < .001
        assert rospy.wait_for_message("/test_vehicle/imu_enabled", Bool, timeout=2).data is False
        assert rig.move(1, .3, client_timeout=.2).accepted
        rig.wait(lambda: rig.motions and rig.motions[-1].state == MotionStatus.FAILED)
        assert "续期" in rig.motions[-1].message
        rig.wait(lambda: rig.emulator.requested_motion == (0, 0, 0))
    finally:
        rospy.set_param("/test_vehicle/use_imu_heading", False)


@pytest.mark.parametrize("loss", ["odometry", "disabled", "mode", "fault"])
def test_no_imu_stops_on_feedback_or_chassis_failure(chassis_rig, loss):
    rig = chassis_rig
    rig.emulator.motion_response = True
    result = rig.move(3, 1)
    assert result.accepted, result.message
    rig.wait(lambda: rig.emulator.requested_motion[0] > 0)
    if loss == "odometry":
        rig.emulator.motion_feedback = False
    elif loss == "disabled":
        rig.emulator.enabled = False
    elif loss == "mode":
        rig.emulator.mode = 1
    else:
        rig.emulator.work_state = 1
    rig.wait(lambda: rig.motions[-1].goal_id == result.goal_id and
             rig.motions[-1].state == MotionStatus.FAILED)
    rig.wait(lambda: rig.emulator.requested_motion == (0, 0, 0))
    assert not rig.move(3, .1).accepted
    rig.emulator.motion_feedback = True
    rig.emulator.enabled, rig.emulator.mode, rig.emulator.work_state = True, 2, 0
    rig.spin(.4)
    assert rig.motions[-1].state == MotionStatus.FAILED  # No automatic resume.
    assert rig.emulator.requested_motion == (0, 0, 0)


@pytest.mark.parametrize("package,launch", [
    ("vehicle_driver", "vehicle.launch"), ("vehicle_gui", "console.launch")])
@pytest.mark.parametrize("use_imu,start_imu", [(False, False), (False, True), (True, False), (True, True)])
@pytest.mark.parametrize("receive_only", [False, True])
def test_launch_composes_one_consistent_heading_mode(package, launch, use_imu, start_imu, receive_only):
    args = [package, launch, "namespace:=mode_check",
            "use_imu:=" + str(use_imu).lower(), "start_imu:=" + str(start_imu).lower()]
    if receive_only:
        args.append("receive_only:=true")
    nodes = subprocess.check_output(["roslaunch", "--nodes"] + args, text=True).splitlines()
    assert ("/mode_check/imu" in nodes) == (use_imu and start_imu)
    params = yaml.safe_load(subprocess.check_output(["roslaunch", "--dump-params"] + args, text=True))
    assert params["/mode_check/vehicle/use_imu_heading"] is use_imu
    assert params["/mode_check/vehicle/receive_only"] is receive_only
    assert params["/mode_check/vehicle/auto_initialize"] is False


@pytest.mark.parametrize("use_imu", [False, True])
def test_gui_attachment_does_not_start_another_driver_or_imu(use_imu):
    nodes = subprocess.check_output(["roslaunch", "--nodes", "vehicle_gui", "console.launch",
        "namespace:=mode_check", "start_driver:=false", "use_imu:=" + str(use_imu).lower()], text=True).splitlines()
    assert nodes == ["/mode_check/vehicle_gui"]


@pytest.mark.parametrize("package,launch", [
    ("vehicle_driver", "vehicle.launch"), ("vehicle_gui", "console.launch")])
def test_invalid_mode_fails_before_starting_nodes(package, launch):
    filename = roslaunch.rlutil.resolve_launch_arguments([package, launch])[0]
    # Noetic's --dump-params reports parse errors but still exits with code 0.
    # Use the same loader as real startup to verify rejection before any node runs.
    with pytest.raises(roslaunch.RLException, match="bool"):
        roslaunch.config.load_config_default(
            [(filename, ["use_imu:=invalid", "start_imu:=false"])], None)
