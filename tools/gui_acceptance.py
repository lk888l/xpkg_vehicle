#!/usr/bin/env python3
"""Exercise the actual Qt GUI against an explicitly selected CAN interface.

Uses the existing driver, no mock telemetry. --motion adds brief raised-chassis
button holds. Always stops and requests disable before terminating the driver.
"""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import time

os.environ["PYQTGRAPH_QT_LIB"] = "PyQt5"
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QFontDatabase
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication
from ament_index_python.packages import get_package_prefix
from vehicle_gui.viewmodel import VehicleViewModel
from vehicle_gui.window import MainWindow


def wait(predicate, timeout=6):
    until = time.monotonic() + timeout
    while not predicate() and time.monotonic() < until:
        QApplication.processEvents()
        QTest.qWait(10)
    if not predicate():
        raise TimeoutError("GUI/device feedback was not observed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--motion", action="store_true")
    parser.add_argument("--output", type=Path, default=Path("artifacts/gui_hardware.json"))
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = {"started_utc": datetime.now(timezone.utc).isoformat(), "interface": args.interface,
              "source": "actual Qt buttons, ROS 2 driver and physical SocketCAN device", "checks": {}}
    app = QApplication([])
    app.setStyle("Fusion")
    if os.environ.get("VEHICLE_GUI_FONT"):
        QFontDatabase.addApplicationFont(os.environ["VEHICLE_GUI_FONT"])
    app.setFont(QFont("Noto Sans CJK SC", 10))
    binary = Path(get_package_prefix("vehicle_driver")) / "lib/vehicle_driver/vehicle_node"
    log = args.output.with_suffix(".log").open("w")
    driver = subprocess.Popen([str(binary), "--ros-args", "-p", f"can_interface:={args.interface}"],
                              stdout=log, stderr=subprocess.STDOUT)
    model = VehicleViewModel(ros_args=[])
    window = MainWindow(model)
    window.show()
    model.start()
    try:
        def enabled():
            state = model.store.snapshot()[0].get("state")
            return state is not None and state.value["online"] and state.value["enabled"] and state.value["mode"] == 2
        wait(enabled)
        wait(lambda: all(f"driver_status_{i}" in model.store.snapshot()[0] for i in range(4)))
        # A query 11 ms after the startup configuration burst was observed to lose its A2.
        # Exercise the ready console after a settling period; do not hide timeouts with retries.
        QTest.qWait(300)
        report["startup_settle_s"] = 0.3
        window.tabs.setCurrentIndex(1)
        QTest.mouseClick(window.version_button, Qt.LeftButton)
        wait(lambda: model.last_result is not None and model.last_result.get("command") == 2)
        report["version_result"] = model.last_result
        report["checks"]["version_from_service"] = model.last_result["state_confirmed"]
        report["checks"]["four_driver_channels"] = all(f"driver_motion_{i}" in model.store.snapshot()[0] for i in range(4))
        window.tabs.setCurrentIndex(0)
        QTest.qWait(200)
        if args.motion:
            window.linear.setValue(0.05)
            QTest.mousePress(window.direction_buttons["forward"], Qt.LeftButton)
            QTest.qWait(1200)
            QTest.mouseRelease(window.direction_buttons["forward"], Qt.LeftButton)
            QTest.qWait(650)
            window.angular.setValue(0.10)
            QTest.mousePress(window.direction_buttons["left"], Qt.LeftButton)
            QTest.qWait(1200)
            QTest.mouseRelease(window.direction_buttons["left"], Qt.LeftButton)
            QTest.qWait(650)
        window.refresh()
        main_image = args.output.with_name(args.output.stem + "_console.png")
        window.grab().save(str(main_image))
        latest, history, frames, _, _ = model.store.snapshot()
        report["observed_state"] = latest["state"].value
        report["velocity_samples"] = len(history["velocity"])
        report["checks"]["velocity_received"] = len(history["velocity"]) > 10
        report["checks"]["odom_received"] = "odom" in latest
        report["checks"]["can_frames_received"] = bool(frames)
        report["checks"]["plots_populated"] = window.vactual.xData is not None and len(window.vactual.xData) > 10
        report["driver_states"] = {str(i): latest[f"driver_status_{i}"].value for i in range(4)}
        if args.motion:
            report["checks"]["forward_command_sent"] = any(v["vx"] == 0.05 for _, v in history["command"])
            report["checks"]["rotation_command_sent"] = any(v["wz"] == 0.1 for _, v in history["command"])
            report["checks"]["button_release_stopped"] = history["command"][-1][1] == {"vx": 0.0, "wz": 0.0}
        window.tabs.setCurrentIndex(1)
        window.refresh()
        QTest.qWait(300)
        window.grab().save(str(args.output.with_name(args.output.stem + "_diagnostics.png")))
        QTest.mouseClick(window.disable_button, Qt.LeftButton)
        wait(lambda: model.last_result and model.last_result.get("command") == 3 and
             model.last_result.get("fields", {}).get("enabled") is False)
        report["disable_result"] = model.last_result
        report["checks"]["disable_confirmed"] = model.last_result["acknowledged"] and model.last_result["state_confirmed"]
        # Diagnostics run at 2 Hz and can straddle the last B2 and disabled B0.
        QTest.qWait(650)
        report["final_state"] = model.store.snapshot()[0]["state"].value
        report["diagnostics"] = model.store.snapshot()[0].get("diagnostics").value
    except Exception as error:
        report["error"] = str(error)
        raise
    finally:
        window.stop_controls()
        window.close()
        if driver.poll() is None:
            driver.send_signal(signal.SIGINT)
            driver.wait(timeout=5)
        log.close()
        report["finished_utc"] = datetime.now(timezone.utc).isoformat()
        report["checks"]["gui_worker_joined"] = not model.worker.isRunning()
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report["checks"], ensure_ascii=False, indent=2), flush=True)
    if not all(report["checks"].values()):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
