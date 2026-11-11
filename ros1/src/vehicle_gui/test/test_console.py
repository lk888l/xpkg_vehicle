"""Qt's bundled QTest + real TCPROS and vcan; no pytest-qt dependency."""
import importlib.util
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import threading
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["PYQTGRAPH_QT_LIB"] = "PyQt5"
import pytest
from PyQt5.QtCore import QEvent, Qt, QTimer
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication
from vehicle_gui.model import connection_state
from vehicle_gui.viewmodel import VehicleViewModel
from vehicle_gui.window import MainWindow
import rospy
from ros_test_support import RosProbe, executable, init_ros
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger, TriggerRequest


def wait_for(predicate, timeout=5):
    until = time.monotonic() + timeout
    while not predicate() and time.monotonic() < until:
        QApplication.processEvents()
        QTest.qWait(10)
    assert predicate()


@pytest.fixture(scope="module")
def app():
    application = QApplication.instance() or QApplication([])
    application.setStyle("Fusion")
    yield application


@pytest.fixture
def console(app, tmp_path, request):
    interface = os.environ.get("VEHICLE_TEST_CAN")
    if not interface:
        pytest.skip("Set VEHICLE_TEST_CAN to an isolated vcan interface")
    # Reuse the driver's protocol emulator instead of maintaining two wire models.
    source = Path(__file__).resolve().parents[2] / "vehicle_driver/test/test_integration.py"
    spec = importlib.util.spec_from_file_location("vehicle_driver_emulator", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    emulator = module.Emulator(interface)
    binary = Path(executable("vehicle_node"))
    output = (tmp_path / "driver.log").open("w")
    init_ros()
    use_imu = getattr(request, "param", {}).get("use_imu", True)
    driver = subprocess.Popen([
        str(binary), "__name:=test_vehicle", "cmd_vel:=test_cmd_vel", "odom:=test_odom",
        "diagnostics:=test_diagnostics", f"_can_interface:={interface}", "_publish_tf:=false",
        "_service_timeout:=0.4", "_auto_imu_calibration:=false", "_auto_initialize:=true",
        f"_use_imu_heading:={str(use_imu).lower()}",
    ], stdout=output, stderr=subprocess.STDOUT)
    rospy.set_param("/test_gui/driver_name", "test_vehicle")
    gui_args = ["__name:=test_gui", "cmd_vel:=test_cmd_vel", "odom:=test_odom",
                "diagnostics:=test_diagnostics"]
    calibration_service = getattr(request, "param", {}).get("calibration_service")
    if calibration_service:
        gui_args.append(f"test_vehicle/calibrate_imu:={calibration_service}")
    imu_driver = None
    if getattr(request, "param", {}).get("imu"):
        emulator.imu = module.ImuEmulator(interface)
        imu_driver = subprocess.Popen([
            str(binary.with_name("imu_node")), "__name:=imu", f"_imu_can_interface:={interface}",
        ], stdout=output, stderr=subprocess.STDOUT)
    model = VehicleViewModel(ros_args=gui_args)
    window = None
    try:
        window = MainWindow(model)
        window.show()
        model.start()
        def connected():
            sample = model.store.snapshot()[0].get("state")
            return sample is not None and sample.value["enabled"] and sample.value["mode"] == 2
        wait_for(connected)
        QTest.qWait(150)
        yield window, model, emulator, driver
    finally:
        if window is not None:
            window.close()
        model.shutdown()
        if driver.poll() is None:
            driver.send_signal(signal.SIGINT)
            driver.wait(timeout=5)
        if imu_driver is not None:
            imu_driver.send_signal(signal.SIGINT)
            imu_driver.wait(timeout=5)
            emulator.imu.close()
        emulator.close()
        output.close()
        if window is not None:
            window.deleteLater()
        QApplication.processEvents()


def motion_commands(emulator):
    return [(stamp, struct.unpack("<hhh", data)) for stamp, code, data in emulator.commands if code == 0x12]


@pytest.mark.parametrize("console", [{"use_imu": False}], indirect=True)
def test_no_imu_console_uses_chassis_heading_and_disables_imu_actions(console):
    window, model, emulator, _ = console
    wait_for(lambda: model.store.latest("imu_mode") is not None)
    window.refresh()
    assert "无 IMU" in window.imu_mode_label.text()
    assert "底盘反馈" in window.heading_hold.text()
    assert not window.calibration_button.isEnabled()
    assert not window.zero_imu_button.isEnabled()
    assert window._heading_feedback()[0] is not None
    emulator.motion_response = True
    window.turn_angle.setValue(10)
    QTest.mouseClick(window.turn_button, Qt.LeftButton)
    wait_for(lambda: model.last_result and model.last_result.get("request_accepted"))
    wait_for(lambda: model.store.latest("motion_status").value["state"] == 2, timeout=8)
    wait_for(lambda: window._motion_revision is None)
    window.drive_distance.setValue(.06)
    QTest.mouseClick(window.distance_button, Qt.LeftButton)
    wait_for(lambda: motion_commands(emulator)[-1][1][0] > 0)
    wait_for(lambda: model.store.latest("motion_status").value["state"] == 2, timeout=8)
    wait_for(lambda: window._motion_revision is None)
    window.setFocus()
    QTest.keyPress(window, Qt.Key_W)
    wait_for(lambda: window.steering.mode == "直行保向")
    QTest.keyRelease(window, Qt.Key_W)
    wait_for(lambda: motion_commands(emulator)[-1][1] == (0, 0, 0))
    emulator.motion_feedback = False
    QTest.qWait(350)
    assert window._heading_feedback()[0] is None
    assert not model.store.latest("heading").value["imu_active"]


def test_mouse_controls_reach_can_and_release_stops(console):
    window, model, emulator, _ = console
    QTest.mousePress(window.direction_buttons["forward"], Qt.LeftButton)
    QTest.qWait(180)
    assert any(values == (100, 0, 0) for _, values in motion_commands(emulator))
    QTest.mouseRelease(window.direction_buttons["forward"], Qt.LeftButton)
    wait_for(lambda: motion_commands(emulator)[-1][1] == (0, 0, 0))
    assert model.store.snapshot()[0]["state"].value["enabled"]
    wait_for(lambda: len(model.store.snapshot()[1]["velocity"]) > 10)
    window.refresh()
    assert window.vactual.xData is not None and len(window.vactual.xData) > 10


def test_frozen_ui_expires_lease_and_does_not_restart_on_recovery(console):
    window, model, emulator, _ = console
    QTest.mousePress(window.direction_buttons["forward"], Qt.LeftButton)
    QTest.qWait(150)
    assert motion_commands(emulator)[-1][1][0] == 100
    freeze = time.monotonic()
    time.sleep(0.38)  # Stop Qt event processing; ROS worker must keep running and stop motion.
    stops = [t for t, values in motion_commands(emulator) if t > freeze and values == (0, 0, 0)]
    assert stops and stops[0] - freeze < 0.30
    QApplication.processEvents()
    QTest.qWait(140)
    assert not window._held
    assert all(values == (0, 0, 0) for t, values in motion_commands(emulator) if t > stops[0])
    QTest.mouseRelease(window.direction_buttons["forward"], Qt.LeftButton)


def test_focus_loss_and_keyboard_stop(console):
    window, model, emulator, _ = console
    window.setFocus()
    QTest.keyPress(window, Qt.Key_A)
    QTest.qWait(120)
    assert motion_commands(emulator)[-1][1] == (0, 0, 200)
    QApplication.sendEvent(window, QEvent(QEvent.WindowDeactivate))
    wait_for(lambda: motion_commands(emulator)[-1][1] == (0, 0, 0))
    assert not window._held
    QTest.keyRelease(window, Qt.Key_A)
    QTest.mousePress(window.direction_buttons["reverse"], Qt.LeftButton)
    QTest.qWait(120)
    QTest.keyPress(window, Qt.Key_Space)
    wait_for(lambda: motion_commands(emulator)[-1][1] == (0, 0, 0))
    QTest.keyRelease(window, Qt.Key_Space)
    window.setFocus()
    QTest.keyPress(window, Qt.Key_W)
    wait_for(lambda: motion_commands(emulator)[-1][1] == (100, 0, 0))
    window.turn_angle.setFocus()
    QTest.keyRelease(window.turn_angle, Qt.Key_W)
    wait_for(lambda: motion_commands(emulator)[-1][1] == (0, 0, 0))
    assert not window._held


def test_service_ack_does_not_fake_a_disabled_state(console):
    window, model, emulator, _ = console
    emulator.apply_enable = False
    QTest.mouseClick(window.disable_button, Qt.LeftButton)
    wait_for(lambda: model.last_result is not None)
    assert model.last_result["acknowledged"] and not model.last_result["state_confirmed"]
    assert model.store.snapshot()[0]["state"].value["enabled"]
    assert "尚未确认" in window.result_detail.text()
    assert "已使能" in window.device.value.text()
    assert not model.busy


def test_version_latched_data_and_driver_disconnect(console):
    window, model, _, driver = console
    wait_for(lambda: "version" in model.store.snapshot()[0])
    assert model.store.snapshot()[0]["version"].value == {"hardware": "3.0", "software": "4.0"}
    driver.send_signal(signal.SIGINT)
    driver.wait(timeout=5)
    QTest.qWait(1700)
    sample = model.store.snapshot()[0]["state"]
    assert not connection_state(sample)[1]
    window.refresh()
    assert window.connection.text() == "反馈已过期"


def test_closing_window_joins_worker_and_sends_stop(console):
    window, model, emulator, _ = console
    QTest.mousePress(window.direction_buttons["forward"], Qt.LeftButton)
    QTest.qWait(150)
    window.close()
    assert not model.worker.isRunning()
    time.sleep(0.08)
    assert motion_commands(emulator)[-1][1] == (0, 0, 0)


def move_odometry(model, emulator):
    # Feed measured motion into the real driver; the emulator otherwise reports zero.
    for _ in range(12):
        emulator.send(0xB2, struct.pack("<hhh", 500, 0, 200))
        QTest.qWait(15)
    wait_for(lambda: model.store.snapshot()[0].get("odom") is not None and
             model.store.snapshot()[0]["odom"].value["x"] > 0.005)
    QTest.qWait(100)


def test_odometry_reset_stops_held_motion_and_starts_a_new_path(console):
    window, model, emulator, _ = console
    move_odometry(model, emulator)
    before = model.store.snapshot()
    window.setFocus()
    QTest.keyPress(window, Qt.Key_W)
    wait_for(lambda: motion_commands(emulator)[-1][1] == (100, 0, 0))
    command_count = len(emulator.commands)
    QTest.mouseClick(window.reset_odometry_button, Qt.LeftButton)
    assert not window._held
    wait_for(lambda: model.last_result is not None and model.last_result.get("kind") == "reset_odometry")
    assert model.last_result["state_confirmed"]
    wait_for(lambda: "odom" in model.store.snapshot()[0])
    wait_for(lambda: motion_commands(emulator)[-1][1] == (0, 0, 0))
    QTest.qWait(250)
    latest, history, _, _, _ = model.store.snapshot()
    assert all(abs(latest["odom"].value[k]) < 1e-9 for k in ("x", "y", "yaw"))
    assert all(abs(p["x"]) < 1e-9 and abs(p["y"]) < 1e-9 for _, p in history["odom"])
    assert len(history["velocity"]) >= len(before[1]["velocity"])
    assert latest["driver_motion_0"].value["pulses"] == -100
    assert not any(code == 5 for _, code, _ in emulator.commands[command_count:])
    assert not model.lease._active and not window._held
    assert "旧轨迹已清空" in window.result_detail.text()
    QTest.keyRelease(window, Qt.Key_W)


@pytest.mark.parametrize("command, button_name", [(4, "clear_faults_button"), (5, "zero_motion_button")])
def test_maintenance_buttons_send_their_can_command_without_resetting_pose(console, command, button_name):
    window, model, emulator, _ = console
    move_odometry(model, emulator)
    old_pose = model.store.snapshot()[0]["odom"].value
    command_count = len(emulator.commands)
    QTest.mouseClick(getattr(window, button_name), Qt.LeftButton)
    assert model.busy and all(not button.isEnabled() for button in window.service_buttons)
    wait_for(lambda: model.last_result is not None and model.last_result.get("command") == command)
    assert [(code, data) for _, code, data in emulator.commands[command_count:] if code == command] == [(command, b"\xCC")]
    assert model.last_result["acknowledged"] and not model.last_result["state_confirmed"]
    current_pose = model.store.snapshot()[0]["odom"].value
    assert all(current_pose[key] == old_pose[key] for key in ("x", "y", "yaw", "frame", "child"))
    assert "实际效果" in window.result_detail.text()
    assert not model.busy


def test_unavailable_reset_service_preserves_pose_and_path(console):
    window, model, emulator, driver = console
    move_odometry(model, emulator)
    driver.send_signal(signal.SIGINT)
    driver.wait(timeout=5)
    QTest.qWait(300)
    latest, history, _, _, _ = model.store.snapshot()
    QTest.mouseClick(window.reset_odometry_button, Qt.LeftButton)
    wait_for(lambda: model.last_result is not None and model.last_result.get("kind") == "reset_odometry")
    assert not model.last_result.get("state_confirmed")
    after, after_history, _, _, _ = model.store.snapshot()
    assert after["odom"] == latest["odom"]
    assert after_history["odom"] == history["odom"]
    assert not model.busy


@pytest.mark.parametrize("console", [{"imu": True}], indirect=True)
def test_imu_zero_button_stops_controls_rebases_angles_and_rejects_stale_data(console):
    window, model, emulator, _ = console
    emulator.imu.yaw = .6
    wait_for(lambda: window.zero_imu_button.isEnabled() and
             abs(model.store.latest("imu").value.get("yaw", 0) - .6) < .001)
    move_odometry(model, emulator)
    previous_pose = model.store.latest("odom").value
    old_path = list(model.store.snapshot()[1]["odom"])
    window.setFocus()
    QTest.keyPress(window, Qt.Key_W)
    wait_for(lambda: motion_commands(emulator)[-1][1][0] == 100)
    QTest.mouseClick(window.zero_imu_button, Qt.LeftButton)
    assert model.busy and not window.zero_imu_button.isEnabled()
    assert not window._held and not window.steering.turning
    wait_for(lambda: model.last_result is not None)
    assert model.last_result["kind"] == "zero_imu" and model.last_result["state_confirmed"]
    wait_for(lambda: model.store.latest("imu").value.get("valid") and
             abs(model.store.latest("imu").value["yaw"]) < .001)
    wait_for(lambda: motion_commands(emulator)[-1][1] == (0, 0, 0))
    assert window.result_flags.text() == "置零完成"
    assert window.result_title.text() == "IMU 角度置零"
    current = model.store.latest("odom").value
    for key in ("x", "y", "yaw", "segment"):
        assert current[key] == pytest.approx(previous_pose[key], abs=.001)
    assert list(model.store.snapshot()[1]["odom"])[:len(old_path)] == old_path
    QTest.keyRelease(window, Qt.Key_W)
    # The next operation must replace the original reference, and stop a turn.
    emulator.imu.yaw = .8
    wait_for(lambda: abs(model.store.latest("imu").value.get("yaw", 0) - .2) < .001)
    QTest.mouseClick(window.turn_button, Qt.LeftButton)
    wait_for(lambda: model.store.latest("motion_status").value["active"] and
             motion_commands(emulator)[-1][1][2] != 0 and not model.busy)
    model.last_result = None
    QTest.mouseClick(window.zero_imu_button, Qt.LeftButton)
    wait_for(lambda: model.last_result is not None)
    assert model.last_result["state_confirmed"] and window._motion_revision is None
    wait_for(lambda: not model.store.latest("motion_status").value["active"])
    wait_for(lambda: abs(model.store.latest("imu").value.get("yaw", 1)) < .001)
    emulator.imu.frozen = True
    wait_for(lambda: not window.zero_imu_button.isEnabled())
    wait_for(lambda: not model.store.latest("imu_status").value["orientation_valid"])
    model.last_result = None
    model.zero_imu()  # The service also rejects stale data independently of UI gating.
    wait_for(lambda: model.last_result is not None)
    assert model.last_result["sent"] and not model.last_result["state_confirmed"]
    assert window.result_flags.text() == "置零失败"
    assert "fresh valid IMU" in window.result_detail.text()
    assert not window.zero_imu_button.isEnabled() and not model.busy
    assert emulator.imu.commands == [(1, 31)]


def test_unavailable_imu_zero_service_reports_failure_without_clearing_path(console):
    window, model, emulator, _ = console
    move_odometry(model, emulator)
    old_path = list(model.store.snapshot()[1]["odom"])
    assert not window.zero_imu_button.isEnabled()
    model.zero_imu()
    wait_for(lambda: model.last_result is not None)
    assert not model.last_result["sent"] and not model.last_result.get("state_confirmed")
    assert window.result_flags.text() == "置零失败"
    assert "IMU 服务未连接" in window.result_detail.text()
    assert list(model.store.snapshot()[1]["odom"])[:len(old_path)] == old_path
    assert not model.busy


@pytest.fixture
def imu_feedback(console):
    window, model, emulator, _ = console
    init_ros()
    node = RosProbe("gui_imu_feedback")
    publisher = node.create_publisher(Imu, "imu/data", 10)
    yaw = [0.0]
    def publish():
        message = Imu()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "base_link"
        message.orientation.w = math.cos(yaw[0] / 2)
        message.orientation.z = math.sin(yaw[0] / 2)
        message.orientation_covariance[0] = .01
        publisher.publish(message)
    timer = QTimer()
    timer.timeout.connect(publish)
    timer.start(10)
    try:
        wait_for(lambda: model.store.latest("heading") and model.store.latest("heading").value["imu_active"])
        yield window, model, emulator, yaw, timer
    finally:
        timer.stop()
        node.destroy_node()
        pass


def test_imu_keyboard_hold_curves_and_released_turn_latch_heading(imu_feedback):
    window, model, emulator, yaw, _ = imu_feedback
    window.setFocus()
    QTest.keyPress(window, Qt.Key_W)
    wait_for(lambda: motion_commands(emulator)[-1][1] == (100, 0, 0))
    yaw[0] = .1
    wait_for(lambda: motion_commands(emulator)[-1][1][2] < -100)
    QTest.keyPress(window, Qt.Key_A)
    wait_for(lambda: motion_commands(emulator)[-1][1] == (100, 0, 200))
    yaw[0] = .4
    wait_for(lambda: model.store.latest("heading").value["yaw"] > .35)
    QTest.keyRelease(window, Qt.Key_A)
    wait_for(lambda: motion_commands(emulator)[-1][1] == (100, 0, 0))
    QTest.keyRelease(window, Qt.Key_W)
    wait_for(lambda: motion_commands(emulator)[-1][1] == (0, 0, 0))
    assert model.store.latest("odom").value["imu_active"]


def test_angle_button_finishes_and_stale_imu_stops_only_angle_mode(imu_feedback):
    window, model, emulator, yaw, timer = imu_feedback
    window.turn_angle.setValue(90)
    window.linear.setValue(0)  # In-place turns do not require a translation speed.
    QTest.mouseClick(window.turn_button, Qt.LeftButton)
    wait_for(lambda: motion_commands(emulator)[-1][1] == (0, 0, 200))
    yaw[0] = math.pi / 2
    wait_for(lambda: model.store.latest("motion_status").value["state"] == 2)
    wait_for(lambda: window._motion_revision is None)
    assert model.store.latest("motion_status").value["message"] == "转向完成"
    assert abs(model.store.latest("odom").value["yaw"] - math.pi/2) < .01
    QTest.mouseClick(window.turn_button, Qt.LeftButton)
    wait_for(lambda: motion_commands(emulator)[-1][1] == (0, 0, 200))
    timer.stop()
    wait_for(lambda: model.store.latest("motion_status").value["state"] == 4)
    wait_for(lambda: motion_commands(emulator)[-1][1] == (0, 0, 0))
    window.linear.setValue(.1)
    window.setFocus()
    QTest.keyPress(window, Qt.Key_W)
    wait_for(lambda: motion_commands(emulator)[-1][1] == (100, 0, 0))
    QTest.keyRelease(window, Qt.Key_W)


def test_distance_button_passes_parameters_and_finishes_with_heading_hold(imu_feedback):
    window, model, emulator, yaw, _ = imu_feedback
    emulator.motion_response = True
    window.drive_distance.setValue(.08)
    window.motion_fields["track_width"].setValue(535)
    window.motion_fields["wheelbase"].setValue(420)
    window.motion_fields["skid_compensation"].setValue(0.7)
    window.motion_fields["distance_tolerance"].setValue(5)
    QTest.mouseClick(window.distance_button, Qt.LeftButton)
    wait_for(lambda: model.last_result and model.last_result["kind"] == "execute_motion")
    assert model.last_result["request_accepted"]
    fields = model.last_result["fields"]
    assert fields["track_width"] == .535 and fields["wheelbase"] == .420
    assert fields["distance_tolerance"] == .005
    assert fields["target"] == .08 and fields["client_timeout"] > 0
    wait_for(lambda: motion_commands(emulator)[-1][1][0] > 0)
    # Perturb IMU heading and ensure differential correction reaches CAN.
    yaw[0] = .06
    wait_for(lambda: motion_commands(emulator)[-1][1][2] < 0)
    yaw[0] = 0
    wait_for(lambda: model.store.latest("motion_status").value["state"] == 2, timeout=8)
    wait_for(lambda: motion_commands(emulator)[-1][1] == (0, 0, 0))
    assert abs(model.store.latest("motion_status").value["remaining"]) <= .005
    assert model.store.latest("motion_status").value["feedforward_gain"] == pytest.approx(1 + .7*(.420/.535)**2)


@pytest.mark.parametrize("console", [{"imu": True}], indirect=True)
def test_automatic_motion_ui_freeze_stops_and_does_not_resume(console):
    window, model, emulator, _ = console
    # IMU emulator runs in its own thread, so freezing Qt tests the UI lease itself.
    wait_for(lambda: model.store.latest("heading") and model.store.latest("heading").value["imu_active"])
    QTest.mouseClick(window.turn_button, Qt.LeftButton)
    wait_for(lambda: model.store.latest("motion_status").value["active"] and not model.busy)
    wait_for(lambda: motion_commands(emulator)[-1][1][2] != 0)
    freeze = time.monotonic()
    time.sleep(.55)
    stops = [t for t, values in motion_commands(emulator) if t > freeze and values == (0, 0, 0)]
    assert stops and stops[0] - freeze < .4
    QApplication.processEvents()
    QTest.qWait(180)
    assert window._motion_revision is None
    assert all(values == (0, 0, 0) for t, values in motion_commands(emulator) if t > stops[0])


def test_manual_input_and_stop_supersede_queued_automatic_motion(imu_feedback, monkeypatch):
    window, model, emulator, _, _ = imu_feedback
    QTest.mouseClick(window.distance_button, Qt.LeftButton)
    wait_for(lambda: model.store.latest("motion_status").value["active"] and not model.busy)
    window.setFocus()
    QTest.keyPress(window, Qt.Key_D)
    wait_for(lambda: model.store.latest("motion_status").value["state"] == 3)
    wait_for(lambda: motion_commands(emulator)[-1][1] == (0, 0, -200))
    QTest.keyRelease(window, Qt.Key_D)
    submit = model.worker.submit
    def stop_before_enqueue(*args, **kwargs):
        window.stop_controls()
        return submit(*args, **kwargs)
    monkeypatch.setattr(model.worker, "submit", stop_before_enqueue)
    model.last_result = None
    QTest.mouseClick(window.turn_button, Qt.LeftButton)
    wait_for(lambda: model.last_result is not None)
    assert model.last_result["superseded"] and not model.last_result["sent"]


def test_calibration_button_measures_inverted_imu_and_restarts(imu_feedback):
    window, model, emulator, yaw, _ = imu_feedback
    emulator.motion_response = True
    previous = [time.monotonic()]
    def follow_motion():
        now = time.monotonic()
        yaw[0] -= emulator.current_motion()[2] / 1000 * (now - previous[0])
        previous[0] = now
    simulation = QTimer()
    simulation.timeout.connect(follow_motion)
    simulation.start(5)
    try:
        window.setFocus()
        QTest.keyPress(window, Qt.Key_W)
        wait_for(lambda: motion_commands(emulator)[-1][1][0] == 100)
        for attempt in range(2):
            model.last_result = None
            command_start = len(emulator.commands)
            QTest.mouseClick(window.calibration_button, Qt.LeftButton)
            assert not window._held and not window.steering.turning
            # Releasing a pre-calibration key must not submit a late zero cmd_vel.
            QTest.keyRelease(window, Qt.Key_W)
            wait_for(lambda: model.last_result is not None)
            assert model.last_result["kind"] == "calibrate_imu"
            assert model.last_result["request_accepted"]
            assert not model.last_result["state_confirmed"]
            assert "请求已受理" in window.result_flags.text()
            wait_for(lambda: model.store.latest("imu_calibration").value["state"] == 3)
            window.refresh()
            assert not window.calibration_button.isEnabled()
            assert window.stop_button.isEnabled()
            wait_for(lambda: model.store.latest("imu_calibration").value["state"] == 7)
            result = model.store.latest("imu_calibration").value
            assert result["calibrated"] and result["yaw_sign"] == -1
            assert result["positive_delta"] < -.15 and result["negative_delta"] > .15
            commands = [struct.unpack("<hhh", data) for _, code, data in
                        emulator.commands[command_start:] if code == 0x12]
            # A command already sent before the click can arrive before the service stop.
            start = commands.index((0, 0, 0))
            assert all(vx == 0 and vy == 0 for vx, vy, wz in commands[start:])
            assert any(wz > 0 for _, _, wz in commands[start:])
            assert any(wz < 0 for _, _, wz in commands[start:])
            window.refresh()
            assert window.calibration_button.isEnabled()
            assert window.calibration_button.text() == "重新标定 IMU"
            assert "方向标定完成" in window.calibration_label.text()
    finally:
        simulation.stop()


@pytest.mark.parametrize("takeover", ["stop", "keyboard"])
def test_calibration_button_allows_stop_and_keyboard_takeover(imu_feedback, takeover):
    window, model, emulator, _, _ = imu_feedback
    QTest.mouseClick(window.calibration_button, Qt.LeftButton)
    wait_for(lambda: model.store.latest("imu_calibration").value["state"] == 3)
    if takeover == "stop":
        QTest.mouseClick(window.stop_button, Qt.LeftButton)
        expected = (0, 0, 0)
    else:
        QTest.keyPress(window, Qt.Key_D)
        expected = (0, 0, -200)
    wait_for(lambda: model.store.latest("imu_calibration").value["state"] == 9)
    wait_for(lambda: motion_commands(emulator)[-1][1] == expected)
    QTest.keyRelease(window, Qt.Key_D)
    wait_for(lambda: motion_commands(emulator)[-1][1] == (0, 0, 0))
    QTest.qWait(200)
    window.refresh()
    assert model.store.latest("imu_calibration").value["state"] == 9
    assert window.calibration_button.isEnabled()


def test_calibration_waiting_for_imu_reports_acceptance_and_can_be_restarted(console):
    window, model, _, _ = console
    for _ in range(2):
        model.last_result = None
        QTest.mouseClick(window.calibration_button, Qt.LeftButton)
        wait_for(lambda: model.last_result is not None)
        assert model.last_result["request_accepted"] and not model.last_result["state_confirmed"]
        wait_for(lambda: model.store.latest("imu_calibration").value["state"] == 1)
        window.refresh()
        assert window.calibration_button.text() == "标定等待中…"
        assert not window.calibration_button.isEnabled()
        assert "等待底盘与 IMU" in window.calibration_label.text()
        QTest.keyClick(window, Qt.Key_Space)
        wait_for(lambda: model.store.latest("imu_calibration").value["state"] == 9)
        window.refresh()
        assert window.calibration_button.isEnabled()


def test_unavailable_calibration_service_reports_failure_and_keeps_button_usable(console):
    window, model, _, driver = console
    driver.send_signal(signal.SIGINT)
    driver.wait(timeout=5)
    QTest.qWait(300)
    QTest.mouseClick(window.calibration_button, Qt.LeftButton)
    wait_for(lambda: model.last_result is not None)
    assert model.last_result["kind"] == "calibrate_imu"
    assert not model.last_result.get("request_accepted")
    assert "请求未受理" in window.result_flags.text()
    assert not model.lease._active
    assert window.calibration_button.isEnabled() and not model.busy


def test_stop_before_queued_calibration_prevents_dispatch(console, monkeypatch):
    window, model, _, _ = console
    submit = model.worker.submit
    def stop_before_enqueue(*args, **kwargs):
        window.stop_controls()
        return submit(*args, **kwargs)
    monkeypatch.setattr(model.worker, "submit", stop_before_enqueue)
    QTest.mouseClick(window.calibration_button, Qt.LeftButton)
    wait_for(lambda: model.last_result is not None)
    assert model.last_result["superseded"] and not model.last_result["sent"]
    QTest.qWait(150)
    assert model.store.latest("imu_calibration").value["state"] == 0
    assert "请求已取消" in window.result_flags.text()


@pytest.mark.parametrize("console", [{"calibration_service": "delayed_calibration"}], indirect=True)
def test_stop_during_delayed_calibration_acceptance_is_reasserted(console):
    window, model, _, _ = console
    init_ros()
    node = RosProbe("delayed_calibration_server")
    entered, release = threading.Event(), threading.Event()
    accepted_at = []
    def accept(request, response):
        entered.set()
        release.wait(timeout=2)
        accepted_at.append(rospy.Time.now().to_nsec())
        response.success = True
        response.message = "Request accepted"
        return response
    node.create_service(Trigger, "delayed_calibration", accept)
    # Observe publication time, not receipt time: a queued pre-acceptance zero
    # alone cannot cancel a request the driver has not processed yet.
    from geometry_msgs.msg import TwistStamped
    stops = []
    node.create_subscription(TwistStamped, "test_cmd_vel", lambda message: stops.append(
        message.header.stamp.to_nsec())
        if message.twist.linear.x == 0 and message.twist.angular.z == 0 else None, 10)
    try:
        # TCPROS connection setup happens asynchronously; no driver motion is involved here.
        QTest.qWait(250)
        QTest.mouseClick(window.calibration_button, Qt.LeftButton)
        wait_for(entered.is_set)
        assert model.busy and window.stop_button.isEnabled()
        QTest.mouseClick(window.stop_button, Qt.LeftButton)
        QTest.qWait(120)
        release.set()
        wait_for(lambda: model.last_result is not None)
        assert model.last_result["request_accepted"] and model.last_result["superseded"]
        wait_for(lambda: any(stamp >= accepted_at[0] for stamp in stops))
        assert "请求已取消" in window.result_flags.text()
        assert not model.busy
    finally:
        release.set()
        node.destroy_node()
        pass
