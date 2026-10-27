# SPDX-License-Identifier: Apache-2.0
import argparse
import os
from pathlib import Path
import signal
import sys

os.environ["PYQTGRAPH_QT_LIB"] = "PyQt5"
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont, QFontDatabase
from PyQt5.QtWidgets import QApplication

from .viewmodel import VehicleViewModel
from .window import MainWindow


def main(argv=None):
    parser = argparse.ArgumentParser(description="MARK1 ROS 2 底盘控制台")
    parser.add_argument("--screenshot", type=Path, help="save a screenshot after startup")
    parser.add_argument("--quit-after", type=float, default=0, help="close after N seconds (smoke tests)")
    options, ros_args = parser.parse_known_args(sys.argv[1:] if argv is None else argv)
    app = QApplication([sys.argv[0]])
    app.setApplicationName("MARK1 Console")
    app.setStyle("Fusion")
    font = os.environ.get("VEHICLE_GUI_FONT")
    if font and Path(font).is_file():
        QFontDatabase.addApplicationFont(font)
    app.setFont(QFont("Noto Sans CJK SC", 10))
    model = VehicleViewModel(ros_args=ros_args)
    window = MainWindow(model)
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    window.show()
    model.start()
    if options.screenshot:
        def capture():
            options.screenshot.parent.mkdir(parents=True, exist_ok=True)
            window.grab().save(str(options.screenshot))
        QTimer.singleShot(2500, capture)
    if options.quit_after > 0:
        QTimer.singleShot(round(options.quit_after * 1000), window.close)
    result = app.exec_()
    model.shutdown()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
