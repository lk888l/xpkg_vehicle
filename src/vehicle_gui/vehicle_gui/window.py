# SPDX-License-Identifier: Apache-2.0
"""Qt-only presentation. ROS callbacks never touch widgets."""
import csv
import math
import time
from pathlib import Path

from PyQt5.QtCore import QEvent, Qt, QTimer
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (QAbstractItemView, QAbstractSpinBox, QApplication, QCheckBox,
    QComboBox, QDoubleSpinBox, QFileDialog, QFrame, QGridLayout, QHBoxLayout, QHeaderView,
    QLabel, QLineEdit, QMainWindow, QPlainTextEdit, QPushButton, QScrollArea, QSplitter, QTabWidget,
    QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)
import pyqtgraph as pg

from .model import COMMAND_NAMES, connection_state, describe_result

STYLE = """
QMainWindow, QWidget#root { background: #f3f5f9; color: #1c2c44; }
QWidget { font-size: 13px; }
QWidget#page { background: #ffffff; }
QScrollArea { border: 0; background: transparent; }
QFrame#header { background: #15283f; border-radius: 14px; }
QLabel#brand { color: #ffffff; font-size: 25px; font-weight: 700; }
QLabel#subtitle { color: #b5c7de; font-size: 12px; }
QLabel#chip { border-radius: 12px; padding: 6px 14px; font-weight: 600; }
QFrame#card { background: #ffffff; border: 1px solid #e0e6ef; border-radius: 12px; }
QLabel#caption { color: #6b7b90; font-size: 12px; }
QLabel#metric { color: #15283f; font-size: 26px; font-weight: 600; }
QLabel#section { font-size: 16px; font-weight: 600; color: #24384f; }
QPushButton { background: #f4f7fb; color: #2b4561; border: 1px solid #d9e2ed;
              border-radius: 7px; padding: 8px 10px; font-weight: 500; }
QPushButton:hover { background: #e9f0f9; border-color: #9eb5d1; }
QPushButton:pressed { background: #d8e6f7; }
QPushButton:disabled { color: #a1adbc; background: #f1f3f6; border-color: #e5e9ef; }
QPushButton#primary { background: #2464d7; border-color: #2464d7; color: #ffffff; }
QPushButton#primary:hover { background: #1b54bd; }
QPushButton#stop { background: #fff0ee; border-color: #f0c4be; color: #bf3e30; font-weight: 700; }
QPushButton#stop:pressed { background: #ffd5cf; }
QPushButton#direction { font-size: 14px; }
QDoubleSpinBox, QComboBox { background: #ffffff; color: #1c2c44; border: 1px solid #cdd8e6;
                          border-radius: 6px; padding: 6px; min-height: 24px; }
QCheckBox { color: #3b506a; spacing: 6px; }
QTabWidget::pane { border: 1px solid #e0e6ef; border-radius: 10px; background: #ffffff; }
QTabBar::tab { background: transparent; color: #6b7b90; padding: 11px 18px; border-bottom: 3px solid transparent; }
QTabBar::tab:selected { color: #2464d7; border-bottom-color: #2464d7; font-weight: 600; }
QTableWidget { background: #ffffff; alternate-background-color: #f6f8fb; color: #25394f;
               border: 0; gridline-color: #edf1f6; selection-background-color: #e5edfa; }
QHeaderView::section { background: #edf2f8; color: #536780; border: 0; padding: 9px 5px; }
QPlainTextEdit { border: 1px solid #e1e7ef; border-radius: 6px; background: #f8fafc; color: #475a72; }
QLabel#footer { color: #6b7b90; font-size: 11px; }
"""


def card():
    frame = QFrame()
    frame.setObjectName("card")
    return frame


def label(text, name="caption"):
    widget = QLabel(text)
    widget.setObjectName(name)
    return widget


class MetricCard(QFrame):
    def __init__(self, title, unit):
        super().__init__()
        self.setObjectName("card")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 11, 16, 11)
        layout.addWidget(label(title))
        self.value = label("—", "metric")
        layout.addWidget(self.value)
        self.detail = label(unit)
        layout.addWidget(self.detail)

    def set(self, value, detail):
        self.value.setText(value)
        self.detail.setText(detail)


class MainWindow(QMainWindow):
    def __init__(self, model):
        super().__init__()
        self.model = model
        self.setWindowTitle("MARK1 · 底盘控制台")
        self.resize(1390, 1030)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setMinimumSize(1100, 760)
        self.setStyleSheet(STYLE)
        self.started = time.monotonic()
        self._held = set()
        self._last_control_tick = self.started
        self._last_tables = 0.0
        self._event_revision = 0
        self.service_buttons = []
        self.direction_buttons = {}
        self.key_actions = {Qt.Key_W: "forward", Qt.Key_Up: "forward", Qt.Key_S: "reverse",
                            Qt.Key_Down: "reverse", Qt.Key_A: "left", Qt.Key_Left: "left",
                            Qt.Key_D: "right", Qt.Key_Right: "right"}
        root = QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(20, 18, 20, 12)
        layout.setSpacing(14)
        header = QFrame()
        header.setObjectName("header")
        row = QHBoxLayout(header)
        row.setContentsMargins(22, 15, 22, 15)
        brand = QVBoxLayout()
        brand.addWidget(label("底盘控制台", "brand"))
        brand.addWidget(label("MARK1_DIFF   /   ROS 2 JAZZY", "subtitle"))
        row.addLayout(brand)
        row.addStretch()
        self.endpoint = label("等待 ROS 连接", "subtitle")
        row.addWidget(self.endpoint)
        row.addSpacing(18)
        self.connection = label("等待设备", "chip")
        self.connection.setStyleSheet("background:#34465e;color:#dce6f2")
        row.addWidget(self.connection)
        layout.addWidget(header)
        metrics = QHBoxLayout()
        self.speed = MetricCard("实际线速度", "m/s")
        self.rotation = MetricCard("实际角速度", "rad/s")
        self.battery = MetricCard("电池电压", "V")
        self.device = MetricCard("设备状态", "等待心跳 / 模式")
        for box in (self.speed, self.rotation, self.battery, self.device):
            metrics.addWidget(box)
        layout.addLayout(metrics)
        body = QSplitter(Qt.Horizontal)
        body.setChildrenCollapsible(False)
        controls = self._controls()
        controls.setMinimumHeight(controls.minimumSizeHint().height())
        control_scroll = QScrollArea()
        control_scroll.setWidgetResizable(True)
        control_scroll.setFrameShape(QFrame.NoFrame)
        control_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        control_scroll.setMinimumWidth(305)
        control_scroll.setMaximumWidth(385)
        control_scroll.setWidget(controls)
        body.addWidget(control_scroll)
        self.tabs = QTabWidget()
        motion_page = self._motion_page()
        motion_page.setMinimumHeight(motion_page.minimumSizeHint().height())
        motion_scroll = QScrollArea()
        motion_scroll.setWidgetResizable(True)
        motion_scroll.setFrameShape(QFrame.NoFrame)
        motion_scroll.setWidget(motion_page)
        self.tabs.addTab(motion_scroll, "运动监测")
        self.tabs.addTab(self._diagnostic_page(), "诊断与 CAN")
        body.addWidget(self.tabs)
        body.setSizes([310, 1040])
        layout.addWidget(body, 1)
        self.footer = label("按住方向按钮或 W/A/S/D 运动 · 松开停止 · 空格停止", "footer")
        layout.addWidget(self.footer)
        self.model.endpoint_changed.connect(lambda text: self.endpoint.setText(text))
        self.model.busy_changed.connect(self._set_busy)
        self.model.result_changed.connect(self._result)
        QApplication.instance().installEventFilter(self)
        self.control_timer = QTimer(self)
        self.control_timer.timeout.connect(self._control_tick)
        self.control_timer.start(50)
        self.render_timer = QTimer(self)
        self.render_timer.timeout.connect(self.refresh)
        self.render_timer.start(50)
        QTimer.singleShot(0, self.setFocus)

    def _button(self, text, callback, name=None, service=True):
        button = QPushButton(text)
        if name:
            button.setObjectName(name)
        button.clicked.connect(callback)
        if service:
            self.service_buttons.append(button)
        return button

    def _controls(self):
        panel = card()
        panel.setMinimumWidth(285)
        panel.setMaximumWidth(365)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(16, 14, 16, 14)
        layout.setSpacing(10)
        layout.addWidget(label("手动控制", "section"))
        layout.addWidget(label("保持按下以运动，松开即停止"))
        speeds = QGridLayout()
        self.linear = QDoubleSpinBox()
        self.linear.setRange(0, 1)
        self.linear.setDecimals(2)
        self.linear.setSingleStep(0.05)
        self.linear.setValue(0.10)
        self.linear.setSuffix("")
        self.angular = QDoubleSpinBox()
        self.angular.setRange(0, 2)
        self.angular.setDecimals(2)
        self.angular.setSingleStep(0.1)
        self.angular.setValue(0.20)
        speeds.addWidget(label("直行 · m/s"), 0, 0)
        speeds.addWidget(self.linear, 1, 0)
        speeds.addWidget(label("旋转 · rad/s"), 0, 1)
        speeds.addWidget(self.angular, 1, 1)
        layout.addLayout(speeds)
        pad = QGridLayout()
        for name, text, y, x in (("forward", "前进 W", 0, 1), ("left", "左转 A", 1, 0),
                                 ("right", "右转 D", 1, 2), ("reverse", "后退 S", 2, 1)):
            button = QPushButton(text)
            button.setObjectName("direction")
            button.setMinimumHeight(38)
            button.pressed.connect(lambda action=name: self._press("mouse:" + action))
            button.released.connect(lambda action=name: self._release("mouse:" + action))
            self.direction_buttons[name] = button
            pad.addWidget(button, y, x)
        layout.addLayout(pad)
        self.stop_button = self._button("停止运动  /  空格", self.stop_controls, "stop", False)
        self.stop_button.setMinimumHeight(42)
        layout.addWidget(self.stop_button)
        layout.addSpacing(4)
        layout.addWidget(label("设备操作", "section"))
        enable_row = QHBoxLayout()
        self.enable_button = self._button("使能", lambda: self._command(3, enabled=True), "primary")
        self.disable_button = self._button("失能", lambda: self._command(3, enabled=False))
        enable_row.addWidget(self.enable_button)
        enable_row.addWidget(self.disable_button)
        layout.addLayout(enable_row)
        mode_row = QHBoxLayout()
        self.mode = QComboBox()
        for name, code in (("待机", 0), ("遥控", 1), ("CAN 控制", 2), ("自由", 3)):
            self.mode.addItem(name, code)
        self.mode.setCurrentIndex(2)
        self.beep = QCheckBox("蜂鸣器")
        mode_row.addWidget(self.mode, 1)
        mode_row.addWidget(self.beep)
        layout.addLayout(mode_row)
        layout.addWidget(self._button("应用模式与蜂鸣器", self._apply_mode))
        self.version_label = label("硬件 —   /   软件 —")
        layout.addWidget(self.version_label)
        layout.addSpacing(3)
        self.result_title = label("命令结果", "section")
        layout.addWidget(self.result_title)
        self.result_flags = label("发送 —   应答 —   状态 —")
        layout.addWidget(self.result_flags)
        self.result_detail = label("设备应答和实际状态分别显示")
        self.result_detail.setWordWrap(True)
        layout.addWidget(self.result_detail)
        layout.addStretch()
        return panel

    def _plot(self, unit):
        plot = pg.PlotWidget(background="w")
        plot.setLabel("left", unit, color="#52677f")
        plot.setLabel("bottom", "时间", units="s", color="#52677f")
        plot.showGrid(x=True, y=True, alpha=0.13)
        plot.setMenuEnabled(False)
        plot.addLegend(offset=(8, 4), labelTextColor="#52677f")
        plot.setMouseEnabled(x=False, y=True)
        for axis in ("left", "bottom"):
            plot.getAxis(axis).setTextPen("#65778c")
            plot.getAxis(axis).setPen("#65778c")
        command = plot.plot(pen=pg.mkPen("#e8a137", width=1.8, style=Qt.DashLine), name="指令")
        feedback = plot.plot(pen=pg.mkPen("#2868d7", width=2), name="反馈")
        return plot, command, feedback

    def _motion_page(self):
        page = QWidget()
        page.setObjectName("page")
        layout = QVBoxLayout(page)
        layout.setContentsMargins(15, 10, 15, 10)
        toolbar = QHBoxLayout()
        toolbar.addWidget(label("实时反馈 · 最近 30 秒", "section"))
        toolbar.addStretch()
        toolbar.addWidget(self._button("导出曲线 CSV", self.export_csv, service=False))
        toolbar.addWidget(self._button("清空图表", self.model.store.clear_history, service=False))
        layout.addLayout(toolbar)
        graphs = QHBoxLayout()
        self.vplot, self.vcmd, self.vactual = self._plot("线速度 / m/s")
        self.wplot, self.wcmd, self.wactual = self._plot("角速度 / rad/s")
        self.vplot.setFixedHeight(225)
        self.wplot.setFixedHeight(225)
        graphs.addWidget(self.vplot)
        graphs.addWidget(self.wplot)
        layout.addLayout(graphs, 4)
        self.mileage_label = label("主轮里程  左 — / 右 —")
        layout.addWidget(self.mileage_label)
        layout.addWidget(label("四路驱动反馈", "section"))
        self.drivers = self._table(["驱动", "RPM", "脉冲", "电压 V", "电流 A", "温度 ℃", "故障位", "更新"], 4)
        self.drivers.setMinimumHeight(155)
        self.drivers.setMaximumHeight(190)
        layout.addWidget(self.drivers)
        lower = QHBoxLayout()
        self.trajectory = pg.PlotWidget(background="w")
        self.trajectory.setAspectLocked(True)
        self.trajectory.setLabel("bottom", "X", units="m")
        self.trajectory.setLabel("left", "Y", units="m")
        self.trajectory.showGrid(x=True, y=True, alpha=0.13)
        self.trajectory.setFixedHeight(175)
        self.path_curve = self.trajectory.plot(pen=pg.mkPen("#2464d7", width=2))
        self.heading = self.trajectory.plot(pen=pg.mkPen("#e8a137", width=3))
        self.position = self.trajectory.plot(pen=None, symbol="o", symbolSize=7, symbolBrush="#e8a137", symbolPen=None)
        lower.addWidget(self.trajectory, 3)
        details = QVBoxLayout()
        details.addWidget(label("里程计轨迹", "section"))
        self.pose_label = label("X —\nY —\n航向 —\n坐标系 —")
        self.pose_label.setStyleSheet("color:#304a69;line-height:1.6")
        details.addWidget(self.pose_label)
        note = label("里程计由设备反馈计算\n低速抖动请结合原始曲线判断")
        note.setWordWrap(True)
        details.addWidget(note)
        details.addStretch()
        lower.addLayout(details, 2)
        layout.addLayout(lower, 3)
        return page

    def _table(self, headers, rows=0):
        table = QTableWidget(rows, len(headers))
        table.setHorizontalHeaderLabels(headers)
        table.verticalHeader().hide()
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        table.setSelectionBehavior(QAbstractItemView.SelectRows)
        table.setAlternatingRowColors(True)
        table.verticalHeader().setDefaultSectionSize(29)
        return table

    def _diagnostic_page(self):
        page = QWidget()
        page.setObjectName("page")
        layout = QVBoxLayout(page)
        layout.setContentsMargins(16, 14, 16, 14)
        self.health_label = label("等待诊断数据", "section")
        layout.addWidget(self.health_label)
        operations = QHBoxLayout()
        self.version_button = self._button("查询版本", lambda: self._command(2))
        operations.addWidget(self.version_button)
        operations.addWidget(self._button("清故障", lambda: self._command(4)))
        operations.addWidget(self._button("清运动计数", lambda: self._command(5)))
        operations.addWidget(self._button("重置本地位姿", self.model.reset_odometry))
        layout.addLayout(operations)
        self.diagnostics = self._table(["诊断项", "值"])
        self.diagnostics.setMaximumHeight(180)
        layout.addWidget(self.diagnostics)
        layout.addWidget(label("整车 CAN 接收帧", "section"))
        self.raw_frames = self._table(["CAN ID", "长度", "数据", "累计帧数", "距上次"])
        self.raw_frames.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        layout.addWidget(self.raw_frames, 1)
        self.unknowns = label("刹车 / 特殊功能：未知   ·   B1 D5：—   ·   故障帧：尚未收到")
        self.unknowns.setWordWrap(True)
        layout.addWidget(self.unknowns)
        layout.addWidget(label("操作与应答记录", "section"))
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.document().setMaximumBlockCount(200)
        self.log.setMaximumHeight(150)
        layout.addWidget(self.log)
        return page

    def _press(self, token):
        self._held.add(token)
        self._last_control_tick = time.monotonic()
        self._renew_motion()

    def _release(self, token):
        self._held.discard(token)
        if self._held:
            self._renew_motion()
        else:
            self.model.lease.stop()

    def _renew_motion(self):
        actions = {token.split(":", 1)[1] for token in self._held}
        vx = (int("forward" in actions) - int("reverse" in actions)) * self.linear.value()
        wz = (int("left" in actions) - int("right" in actions)) * self.angular.value()
        self.model.lease.renew(vx, wz)

    def _control_tick(self):
        now = time.monotonic()
        if now - self._last_control_tick > 0.18 and self._held:
            self.stop_controls()
            self.model.store.event("界面响应中断，运动已停止；重新按下方向键继续", "warning")
        self._last_control_tick = now
        if self._held:
            self._renew_motion()

    def stop_controls(self):
        self._held.clear()
        for button in self.direction_buttons.values():
            button.setDown(False)
        self.model.lease.stop()

    def _command(self, command, **fields):
        if command in (3, 5, 17):
            self.stop_controls()
        self.model.command(command, **fields)

    def _apply_mode(self):
        self._command(17, mode=self.mode.currentData(), beep=self.beep.isChecked())

    def eventFilter(self, watched, event):
        if event.type() == QEvent.ApplicationDeactivate or (watched is self and event.type() == QEvent.WindowDeactivate):
            self.stop_controls()
        if event.type() in (QEvent.KeyPress, QEvent.KeyRelease):
            if isinstance(watched, QWidget) and watched.window() is not self:
                return False
            key = event.key()
            if key in (Qt.Key_Space, Qt.Key_Escape):
                self.stop_controls()
                return True
            if isinstance(QApplication.focusWidget(), (QLineEdit, QAbstractSpinBox, QComboBox, QPlainTextEdit)):
                return False
            if key in self.key_actions:
                if not event.isAutoRepeat():
                    token = f"key{key}:" + self.key_actions[key]
                    (self._press if event.type() == QEvent.KeyPress else self._release)(token)
                return True
        return super().eventFilter(watched, event)

    def _set_busy(self, busy):
        for button in self.service_buttons:
            button.setEnabled(not busy)
        if busy:
            self.result_title.setText("等待设备反馈…")
            self.result_flags.setText("等待发送 / 应答 / 状态结果")

    def _result(self, result):
        _, text = describe_result(result)
        self.result_title.setText(COMMAND_NAMES.get(result.get("command"), "重置本地里程计"))
        ack_expected = result.get("command") in (2, 3, 4, 5)
        flag = lambda key: "✓" if result.get(key) else "—"
        self.result_flags.setText(f"发送 {flag('sent')}   应答 {flag('acknowledged') if ack_expected else '不适用'}   状态 {flag('state_confirmed')}")
        self.result_detail.setText(text)
        self.result_detail.setToolTip(result.get("message", ""))

    @staticmethod
    def _cell(table, row, column, text, stale=False):
        item = table.item(row, column)
        if item is None:
            item = QTableWidgetItem()
            item.setTextAlignment(Qt.AlignCenter)
            table.setItem(row, column, item)
        item.setText(str(text))
        item.setForeground(QColor("#a08557" if stale else "#30465f"))

    def refresh(self):
        now = time.monotonic()
        latest, history, frames, events, revision = self.model.store.snapshot()
        state = latest.get("state")
        status_text, online = connection_state(state, now)
        self.connection.setText(status_text)
        self.connection.setStyleSheet("background:#d9f3e6;color:#19784e" if online else "background:#34465e;color:#dce6f2")
        velocity, command = latest.get("velocity"), latest.get("command")
        for metric, key, unit in ((self.speed, "vx", "m/s"), (self.rotation, "wz", "rad/s")):
            value = f"{velocity.value[key]:+.3f}" if velocity else "—"
            target = f"{command.value[key]:+.2f}" if command else "0.00"
            stale = " · 反馈过期" if velocity and not velocity.fresh(0.3, now) else ""
            metric.set(value, f"{unit}    指令 {target}{stale}")
        if state:
            s = state.value
            known = s["status_received"]
            stale = " · 状态过期" if known and s["status_age"] + state.age(now) > 0.5 else ""
            self.battery.set(f"{s['voltage']:.1f}" if known else "—", "V" + stale)
            modes = {0: "待机", 1: "遥控", 2: "CAN 控制", 3: "自由"}
            enabled = "已使能" if s["enabled"] else "已失能"
            self.device.set(enabled if s["heartbeat_received"] else "未知",
                            (modes.get(s["mode"], "未知模式") if known else "模式未知") + ("" if online else " · 离线"))
        version = latest.get("version")
        if version:
            self.version_label.setText(f"硬件 {version.value['hardware']}   /   软件 {version.value['software']}")
        mileage = latest.get("mileage")
        if mileage:
            m = mileage.value
            self.mileage_label.setText(f"主轮累计里程    左 {m['left']/1000:+.3f} m    /    右 {m['right']/1000:+.3f} m" +
                                       ("   · 反馈过期" if not mileage.fresh(0.3, now) else ""))
        if self.tabs.currentIndex() == 0:
            self._update_plots(history, now)
            pose = latest.get("odom")
            if pose:
                p = pose.value
                self.pose_label.setText(f"X   {p['x']:+.3f} m\nY   {p['y']:+.3f} m\n航向   {math.degrees(p['yaw']):+.1f}°\n{p['frame']} → {p['child']}" +
                                       ("\n反馈过期" if not pose.fresh(0.3, now) else ""))
                self.position.setData([p['x']], [p['y']])
                self.heading.setData([p['x'], p['x'] + 0.08 * math.cos(p['yaw'])],
                                     [p['y'], p['y'] + 0.08 * math.sin(p['yaw'])])
        if now - self._last_tables > 0.25:
            self._last_tables = now
            self._update_tables(latest, frames, now)
        if revision != self._event_revision:
            for stamp, level, text in events[-min(revision - self._event_revision, len(events)):]:
                self.log.appendPlainText(f"{stamp}  {level.upper()}  {text}")
            self._event_revision = revision
        diag = latest.get("diagnostics")
        summary = "等待通信诊断" if not diag else f"CAN RX {diag.value.get('rx_frames','—')} / TX {diag.value.get('tx_frames','—')}    接收丢帧 {diag.value.get('socket_dropped_frames','—')}"
        self.footer.setText(summary + "    ·    松开方向键停止 / 空格停止    ·    刹车与特殊功能状态未知")

    def _update_plots(self, history, now):
        for source, curves in (("velocity", (self.vactual, self.wactual)), ("command", (self.vcmd, self.wcmd))):
            rows = [(t, v) for t, v in history[source] if t >= now - 30]
            for key, curve in zip(("vx", "wz"), curves):
                xs, ys = [], []
                previous = None
                for t, value in rows:
                    if source == "velocity" and previous is not None and t - previous > 0.3:
                        xs.append(t - self.started)
                        ys.append(float("nan"))
                    xs.append(t - self.started)
                    ys.append(value[key])
                    previous = t
                if rows and source == "command" and rows[-1][1][key] == 0:
                    xs.append(now - self.started)
                    ys.append(0)
                curve.setData(xs, ys, connect="finite")
        elapsed = now - self.started
        for plot in (self.vplot, self.wplot):
            plot.setXRange(max(0, elapsed - 30), max(30, elapsed), padding=0)
        # Repeated identical points can create raster artifacts with a thick Qt pen.
        # Remove only exact duplicates; preserve all nonzero measured displacement.
        points = []
        for _, pose in history["odom"]:
            point = (pose["x"], pose["y"])
            if not points or point != points[-1]:
                points.append(point)
        self.path_curve.setData([x for x, _ in points] if len(points) > 1 else [],
                                [y for _, y in points] if len(points) > 1 else [])

    def _update_tables(self, latest, frames, now):
        for channel in range(4):
            motion = latest.get(f"driver_motion_{channel}")
            status = latest.get(f"driver_status_{channel}")
            m, s = motion.value if motion else {}, status.value if status else {}
            age = max(motion.age(now) if motion else float("inf"), status.age(now) if status else float("inf"))
            values = [str(channel + 1), m.get("rpm", "—"), m.get("pulses", "—"),
                      f"{s['voltage']:.1f}" if s else "—", f"{s['current']:.1f}" if s else "—",
                      f"{s['driver_temperature']} / {s['motor_temperature']}" if s else "—",
                      f"0x{s['faults_raw']:02X}" if s else "—", "未知" if not math.isfinite(age) else f"{age:.1f}s"]
            for column, value in enumerate(values):
                self._cell(self.drivers, channel, column, value, age > 1.2)
        diag = latest.get("diagnostics")
        if diag:
            self.health_label.setText("通信诊断 · " + diag.value["message"] + (" · 已过期" if not diag.fresh(1.5, now) else ""))
            items = [(k, v) for k, v in diag.value.items() if k not in ("level", "message")]
            self.diagnostics.setRowCount(len(items))
            for row, (key, value) in enumerate(items):
                self._cell(self.diagnostics, row, 0, key)
                self._cell(self.diagnostics, row, 1, value)
        self.raw_frames.setRowCount(len(frames))
        for row, (identifier, sample) in enumerate(sorted(frames.items())):
            f = sample.value
            for column, value in enumerate((f"{identifier:08X}", f["length"], f["data"], f["count"], f"{sample.age(now):.2f}s")):
                self._cell(self.raw_frames, row, column, value, sample.age(now) > 1.5)
        state, faults = latest.get("state"), latest.get("faults")
        flags = f"0x{state.value['flags_raw']:02X}" if state and state.value["status_received"] else "—"
        fault_text = "尚未收到" if not faults else " ".join(f"{k}=0x{v:02X}" for k, v in faults.value.items())
        status = state.value if state and state.value["status_received"] else None
        observed = (f"工作状态 0x{status['work_state']:02X} · 蜂鸣器{'开' if status['beep'] else '关'}"
                    if status else "等待整车状态")
        if status and status["status_age"] + state.age(now) > 0.5:
            observed += " · 已过期"
        self.unknowns.setText(f"{observed}   ·   刹车 / 特殊功能：未知\n"
                             f"B1 D5：{flags}   ·   故障帧：{fault_text}")

    def export_csv(self):
        self.stop_controls()
        name, _ = QFileDialog.getSaveFileName(self, "导出速度曲线", "vehicle_motion.csv", "CSV (*.csv)")
        if not name:
            return
        try:
            _, history, _, _, _ = self.model.store.snapshot()
            with Path(name).open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(["elapsed_s", "source", "vx_m_s", "yaw_rate_rad_s"])
                for stamp, source, value in sorted((t, source, value) for source in ("command", "velocity") for t, value in history[source]):
                    writer.writerow([f"{stamp-self.started:.6f}", source, value["vx"], value["wz"]])
            self.model.store.event("曲线已导出 · " + name)
        except OSError as error:
            self.model.store.event("导出失败 · " + str(error), "error")

    def closeEvent(self, event):
        self.stop_controls()
        self.control_timer.stop()
        self.render_timer.stop()
        if not self.model.shutdown():
            self.result_detail.setText("正在停止通信线程…")
            event.ignore()
            QTimer.singleShot(100, self.close)
            return
        QApplication.instance().removeEventFilter(self)
        event.accept()
