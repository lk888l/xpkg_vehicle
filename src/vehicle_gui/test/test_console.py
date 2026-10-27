"""Qt's bundled QTest + real DDS and vcan; no pytest-qt dependency."""
import importlib.util
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["PYQTGRAPH_QT_LIB"] = "PyQt5"
import pytest
from PyQt5.QtCore import QEvent, Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication
from ament_index_python.packages import get_package_prefix
from vehicle_gui.model import connection_state
from vehicle_gui.viewmodel import VehicleViewModel
from vehicle_gui.window import MainWindow


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
def console(app, tmp_path):
    interface = os.environ.get("VEHICLE_TEST_CAN")
    if not interface:
        pytest.skip("Set VEHICLE_TEST_CAN to an isolated vcan interface")
    # Reuse the driver's protocol emulator instead of maintaining two wire models.
    source = Path(__file__).resolve().parents[2] / "vehicle_driver/test/test_integration.py"
    spec = importlib.util.spec_from_file_location("vehicle_driver_emulator", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    emulator = module.Emulator(interface)
    binary = Path(get_package_prefix("vehicle_driver")) / "lib/vehicle_driver/vehicle_node"
    output = (tmp_path / "driver.log").open("w")
    driver = subprocess.Popen([
        str(binary), "--ros-args", "-r", "__node:=test_vehicle", "-r", "cmd_vel:=test_cmd_vel",
        "-r", "odom:=test_odom", "-r", "diagnostics:=test_diagnostics",
        "-p", f"can_interface:={interface}", "-p", "publish_tf:=false", "-p", "service_timeout:=0.4",
    ], stdout=output, stderr=subprocess.STDOUT)
    model = VehicleViewModel(ros_args=["--ros-args", "-r", "__node:=test_gui",
        "-r", "cmd_vel:=test_cmd_vel", "-r", "odom:=test_odom", "-r", "diagnostics:=test_diagnostics",
        "-p", "driver_name:=test_vehicle"])
    window = MainWindow(model)
    window.show()
    model.start()
    try:
        def connected():
            sample = model.store.snapshot()[0].get("state")
            return sample is not None and sample.value["enabled"] and sample.value["mode"] == 2
        wait_for(connected)
        QTest.qWait(150)
        yield window, model, emulator, driver
    finally:
        window.close()
        model.shutdown()
        if driver.poll() is None:
            driver.send_signal(signal.SIGINT)
            driver.wait(timeout=5)
        emulator.close()
        output.close()
        window.deleteLater()
        QApplication.processEvents()


def motion_commands(emulator):
    return [(stamp, struct.unpack("<hhh", data)) for stamp, code, data in emulator.commands if code == 0x12]


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
