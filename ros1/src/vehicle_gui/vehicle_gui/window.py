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
from .steering import SteeringController, wrap

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
        self.steering = SteeringController()
        self._motion_revision = None
        self._motion_goal_id = None
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
        brand.addWidget(label("MARK1_DIFF   /   ROS 1 NOETIC", "subtitle"))
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
        control_scroll = self._scroll_page(controls)
        control_scroll.setMinimumWidth(305)
        control_scroll.setMaximumWidth(385)
        body.addWidget(control_scroll)
        self.tabs = QTabWidget()
        self.motion_scroll = self._scroll_page(self._motion_page())
        self.tabs.addTab(self.motion_scroll, "运动监测")
        self.tabs.addTab(self._scroll_page(self._diagnostic_page()), "诊断与 CAN")
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

    @staticmethod
    def _scroll_page(page):
        page.setMinimumHeight(page.minimumSizeHint().height())
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidget(page)
        return scroll

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
        self.heading_hold = QCheckBox("直行时用 IMU 保持航向")
        self.heading_hold.setChecked(True)
        layout.addWidget(self.heading_hold)
        self.calibration_label = label("IMU 方向标定 · 等待驱动")
        self.calibration_label.setWordWrap(True)
        layout.addWidget(self.calibration_label)
        self.calibration_button = self._button("开始 IMU 标定", self._calibrate_imu)
        self.calibration_button.setToolTip("自动原地正反转，测定 yaw 方向；停止按钮或方向键可随时接管")
        layout.addWidget(self.calibration_button)
        self.zero_imu_button = self._button("IMU 角度置零", self._zero_imu)
        self.zero_imu_button.setToolTip("停止当前控制，将当前姿态设为横滚、俯仰和 IMU 航向的零点；需要有效姿态数据")
        self.zero_imu_button.setEnabled(False)
        layout.addWidget(self.zero_imu_button)
        layout.addWidget(label("角度转向 · 左正 / 右负", "section"))
        angle_row = QHBoxLayout()
        self.turn_angle = QDoubleSpinBox()
        self.turn_angle.setRange(-360, 360)
        self.turn_angle.setDecimals(1)
        self.turn_angle.setSingleStep(15)
        self.turn_angle.setValue(90)
        self.turn_angle.setSuffix("°")
        self.turn_reference = QComboBox()
        self.turn_reference.addItems(["相对转角", "路径航向"])
        self.turn_reference.setToolTip("相对转角按指定正负方向旋转；路径航向以本次里程计零点为参考，走最短转向")
        angle_row.addWidget(self.turn_angle)
        angle_row.addWidget(self.turn_reference)
        layout.addLayout(angle_row)
        self.turn_button = self._button("执行角度转向", self._start_turn)
        layout.addWidget(self.turn_button)
        layout.addWidget(label("定距离前进 · IMU 保向", "section"))
        distance_row = QHBoxLayout()
        self.drive_distance = QDoubleSpinBox()
        self.drive_distance.setRange(0.01, 100)
        self.drive_distance.setDecimals(3)
        self.drive_distance.setSingleStep(0.1)
        self.drive_distance.setValue(1.0)
        self.drive_distance.setSuffix(" m")
        self.distance_button = self._button("执行前进", self._start_distance)
        distance_row.addWidget(self.drive_distance)
        distance_row.addWidget(self.distance_button)
        layout.addLayout(distance_row)
        hint = label("上方直行 / 旋转速度作为执行上限\n参数在下次执行时生效")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        self.motion_status_label = label("自动运动 · 等待命令")
        self.motion_status_label.setWordWrap(True)
        layout.addWidget(self.motion_status_label)
        self.parameter_toggle = QPushButton("展开转向与行驶参数")
        self.parameter_toggle.setCheckable(True)
        layout.addWidget(self.parameter_toggle)
        self.parameter_panel = QWidget()
        parameters = QGridLayout(self.parameter_panel)
        parameters.setContentsMargins(0, 0, 0, 0)
        self.motion_fields = {}
        specs = [
            ("track_width", "左右轮距 · mm", 50, 5000, 535, 1, 1),
            ("wheelbase", "前后轴距 · mm", 50, 5000, 420, 1, 1),
            ("skid_compensation", "滑移前馈系数 k", 0, 5, 0.5, 0.1, 2),
            ("heading_gain", "航向增益 · 1/s", 0.01, 20, 1.8, 0.1, 2),
            ("yaw_rate_gain", "角速度反馈增益", 0, 5, 0.2, 0.1, 2),
            ("angle_tolerance", "到位角度误差 · °", 0.1, 20, 1, 0.1, 1),
            ("distance_gain", "距离增益 · 1/s", 0.01, 10, 0.8, 0.1, 2),
            ("distance_tolerance", "到位距离误差 · mm", 1, 200, 10, 1, 1),
            ("timeout", "执行超时 · s", 1, 3600, 120, 10, 0),
        ]
        for row, (name, title, low, high, default, step, decimals) in enumerate(specs):
            box = QDoubleSpinBox()
            box.setRange(low, high)
            box.setDecimals(decimals)
            box.setSingleStep(step)
            box.setValue(default)
            parameters.addWidget(label(title), row, 0)
            parameters.addWidget(box, row, 1)
            self.motion_fields[name] = box
        self.feedforward_label = label("")
        self.feedforward_label.setWordWrap(True)
        parameters.addWidget(self.feedforward_label, len(specs), 0, 1, 2)
        for key in ("track_width", "wheelbase", "skid_compensation"):
            self.motion_fields[key].valueChanged.connect(self._update_feedforward)
        self._update_feedforward()
        layout.addWidget(self.parameter_panel)
        self.parameter_panel.hide()
        self.parameter_toggle.toggled.connect(self._toggle_parameters)
        self.steering_label = label("W/S 直行保向 · A/D 自由转弯\n方向键可随时接管角度转向")
        self.steering_label.setWordWrap(True)
        layout.addWidget(self.steering_label)
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
        layout.addSpacing(4)
        layout.addWidget(label("里程计与维护", "section"))
        operations = QGridLayout()
        self.reset_odometry_button = self._button("里程计清零", self._reset_odometry)
        self.reset_odometry_button.setToolTip("停止手动运动，将 ROS 的 X / Y / 航向清零，成功后清空旧轨迹")
        self.zero_motion_button = self._button("设备计数清零", lambda: self._command(5))
        self.zero_motion_button.setToolTip("停止手动运动，清零设备累计里程和驱动脉冲；保留 ROS 位姿与轨迹")
        self.clear_faults_button = self._button("清故障", lambda: self._command(4))
        self.version_button = self._button("查询版本", lambda: self._command(2))
        for button, y, x in ((self.reset_odometry_button, 0, 0), (self.zero_motion_button, 0, 1),
                             (self.clear_faults_button, 1, 0), (self.version_button, 1, 1)):
            operations.addWidget(button, y, x)
        layout.addLayout(operations)
        reset_note = label("里程计清零：本地位姿与轨迹\n设备计数清零：设备里程与脉冲")
        reset_note.setWordWrap(True)
        layout.addWidget(reset_note)
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
        legend = plot.addLegend(offset=(8, 4))
        plot.setMouseEnabled(x=False, y=True)
        for axis in ("left", "bottom"):
            plot.getAxis(axis).setTextPen("#65778c")
            plot.getAxis(axis).setPen("#65778c")
        command = plot.plot(pen=pg.mkPen("#e8a137", width=1.8, style=Qt.DashLine), name="指令")
        feedback = plot.plot(pen=pg.mkPen("#2868d7", width=2), name="反馈")
        # Ubuntu 20.04's PyQtGraph lacks addLegend(labelTextColor=...).
        for _, text in legend.items:
            text.setText(text.text, color="#52677f")
        return plot, command, feedback

    def _motion_page(self):
        page = QWidget()
        page.setObjectName("page")
        layout = QVBoxLayout(page)
        layout.setContentsMargins(15, 10, 15, 10)
        layout.setSpacing(12)
        trajectory_toolbar = QHBoxLayout()
        trajectory_toolbar.addWidget(label("里程计与路径轨迹", "section"))
        trajectory_toolbar.addStretch()
        self.clear_trajectory_button = self._button("仅清空轨迹", self._clear_trajectory, service=False)
        self.clear_trajectory_button.setToolTip("只清除已绘制的路径，保留当前位姿、设备计数和速度曲线")
        trajectory_toolbar.addWidget(self.clear_trajectory_button)
        self.fit_trajectory_button = self._button("显示完整轨迹", self._fit_trajectory, service=False)
        trajectory_toolbar.addWidget(self.fit_trajectory_button)
        layout.addLayout(trajectory_toolbar)
        recording = QHBoxLayout()
        self.record_button = self._button("开始记录路径", self._toggle_recording, service=False)
        recording.addWidget(self.record_button)
        recording.addWidget(self._button("导出当前路径", self.export_path, service=False))
        self.record_label = label("路径记录包含航向、控制目标与 IMU 状态")
        recording.addWidget(self.record_label, 1)
        layout.addLayout(recording)
        self.imu_label = label("IMU · 等待 can1 姿态反馈")
        self.imu_label.setWordWrap(True)
        layout.addWidget(self.imu_label)
        self.pose_label = label("X —    Y —    航向 —    ·    等待里程计反馈")
        self.pose_label.setStyleSheet("color:#304a69;font-size:14px")
        self.pose_label.setWordWrap(True)
        layout.addWidget(self.pose_label)
        self.trajectory = pg.PlotWidget(background="w")
        self.trajectory.setAspectLocked(True)
        self.trajectory.setLabel("bottom", "X", units="m")
        self.trajectory.setLabel("left", "Y", units="m")
        self.trajectory.showGrid(x=True, y=True, alpha=0.13)
        self.trajectory.setMenuEnabled(False)
        self.trajectory.setMinimumHeight(560)
        self.path_curve = self.trajectory.plot(pen=pg.mkPen("#2464d7", width=2))
        self.heading = self.trajectory.plot(pen=pg.mkPen("#e8a137", width=3))
        self.target_heading = self.trajectory.plot(pen=pg.mkPen("#29926b", width=2, style=Qt.DashLine))
        self.position = self.trajectory.plot(pen=None, symbol="o", symbolSize=9, symbolBrush="#e8a137", symbolPen=None)
        layout.addWidget(self.trajectory, 1)
        note = label("轨迹内滚轮缩放 · 拖动平移 · 图外滚轮或右侧滚动条滚动页面\n"
                     "橙线为当前航向，绿虚线为目标航向。IMU 有效时修正转向，缺失时使用底盘里程计。")
        note.setWordWrap(True)
        layout.addWidget(note)
        layout.addSpacing(8)
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
        layout.addLayout(graphs)
        self.mileage_label = label("主轮里程  左 — / 右 —")
        layout.addWidget(self.mileage_label)
        layout.addWidget(label("四路驱动反馈", "section"))
        self.drivers = self._table(["驱动", "RPM", "脉冲", "电压 V", "电流 A", "温度 ℃", "故障位", "更新"], 4)
        self.drivers.setMinimumHeight(155)
        self.drivers.setMaximumHeight(190)
        layout.addWidget(self.drivers)
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
        self.diagnostics = self._table(["诊断项", "值"])
        self.diagnostics.setFixedHeight(180)
        layout.addWidget(self.diagnostics)
        layout.addWidget(label("整车 CAN 接收帧", "section"))
        self.raw_frames = self._table(["CAN ID", "长度", "数据", "累计帧数", "距上次"])
        self.raw_frames.setMinimumHeight(300)
        self.raw_frames.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        layout.addWidget(self.raw_frames, 1)
        self.unknowns = label("刹车 / 特殊功能：未知   ·   B1 D5：—   ·   故障帧：尚未收到")
        self.unknowns.setWordWrap(True)
        layout.addWidget(self.unknowns)
        layout.addWidget(label("操作与应答记录", "section"))
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.document().setMaximumBlockCount(200)
        self.log.setFixedHeight(150)
        layout.addWidget(self.log)
        return page

    def _press(self, token):
        self._motion_revision = None
        if self.steering.turning:
            self.steering.cancel()
        self._held.add(token)
        self._last_control_tick = time.monotonic()
        self._renew_motion()

    def _release(self, token):
        if token not in self._held:
            return
        self._held.discard(token)
        if self._held:
            self._renew_motion()
        else:
            self.steering.cancel()
            self.model.lease.stop()

    def _heading_feedback(self):
        sample = self.model.store.latest("heading")
        if sample and sample.fresh(0.25) and sample.value["imu_active"] and \
                0 <= sample.value["imu_age"] + sample.age() < 0.25:
            return sample.value["yaw"], sample.value["wz"]
        return None, 0.0

    def _steering_state(self):
        status = self.model.store.latest("motion_status")
        if status and status.fresh(0.5) and status.value["active"] and not self._held:
            value = status.value
            self.model.store.update("steering", {"mode": value["message"],
                "target": wrap(value["target_yaw"]), "error": value["heading_error"]})
            return
        self.model.store.update("steering", {
            "mode": self.steering.mode, "target": None if self.steering.target is None else wrap(self.steering.target),
            "error": self.steering.error})

    def _start_turn(self):
        self._start_motion(1 if self.turn_reference.currentIndex() == 0 else 2,
                           math.radians(self.turn_angle.value()))

    def _start_distance(self):
        self._start_motion(3, self.drive_distance.value())

    def _toggle_parameters(self, visible):
        self.parameter_panel.setVisible(visible)
        self.parameter_toggle.setText("收起转向与行驶参数" if visible else "展开转向与行驶参数")
        # Recompute the independently scrolling control panel after expansion.
        panel = self.parameter_panel.parentWidget()
        panel.layout().activate()
        panel.setMinimumHeight(panel.minimumSizeHint().height())

    def _update_feedforward(self):
        width = self.motion_fields["track_width"].value()
        length = self.motion_fields["wheelbase"].value()
        coefficient = self.motion_fields["skid_compensation"].value()
        gain = 1 + coefficient * (length / width) ** 2
        self.feedforward_label.setText(f"估算前馈倍率 {gain:.3f}\n四个固定动力轮 · k 需按地面调试")

    def _start_motion(self, mode, target):
        yaw, _ = self._heading_feedback()
        if yaw is None:
            self.model.store.event("暂无有效 IMU 航向，无法执行自动运动；方向键仍可使用", "warning")
            return
        if self.angular.value() <= 0:
            self.model.store.event("请将旋转速度设为大于 0", "warning")
            return
        if mode == 3 and self.linear.value() <= 0:
            self.model.store.event("请将直行速度设为大于 0", "warning")
            return
        fields = {name: box.value() for name, box in self.motion_fields.items()}
        for name in ("track_width", "wheelbase", "distance_tolerance"):
            fields[name] /= 1000
        fields["angle_tolerance"] = math.radians(fields["angle_tolerance"])
        self._clear_controls()
        self._motion_revision = self.model.execute_motion(mode=mode, target=target,
            use_custom_config=True, max_linear_speed=max(0.01, self.linear.value()), max_angular_speed=self.angular.value(),
            client_timeout=0.35, **fields)
        self._last_control_tick = time.monotonic()
        self._steering_state()
        self.setFocus()

    def _renew_motion(self):
        actions = {token.split(":", 1)[1] for token in self._held}
        vx = (int("forward" in actions) - int("reverse" in actions)) * self.linear.value()
        wz = (int("left" in actions) - int("right" in actions)) * self.angular.value()
        yaw, rate = self._heading_feedback()
        self.steering.gain = self.motion_fields["heading_gain"].value()
        vx, wz = self.steering.manual(vx, wz, yaw, self.heading_hold.isChecked(), self.angular.value())
        if self.steering.mode == "直行保向":
            width = self.motion_fields["track_width"].value()
            length = self.motion_fields["wheelbase"].value()
            gain = 1 + self.motion_fields["skid_compensation"].value() * (length / width) ** 2
            wz = max(-self.angular.value(), min(self.angular.value(), gain * wz +
                     self.motion_fields["yaw_rate_gain"].value() * (wz - rate)))
        self.model.lease.renew(vx, wz)
        self._steering_state()

    def _control_tick(self):
        now = time.monotonic()
        if now - self._last_control_tick > 0.18 and (self._held or self._motion_revision is not None):
            self.stop_controls()
            self.model.store.event("界面响应中断，运动已停止；重新按下方向键继续", "warning")
        self._last_control_tick = now
        if self._held:
            self._renew_motion()
        elif self._motion_revision is not None:
            status = self.model.store.latest("motion_status")
            finished = (status and status.value["goal_id"] == self._motion_goal_id and
                        not status.value["active"])
            if finished or not self.model.lease.keepalive(self._motion_revision, now):
                self._motion_revision = None
                self._motion_goal_id = None
            self._steering_state()

    def _clear_controls(self):
        self._motion_revision = None
        self._motion_goal_id = None
        self._held.clear()
        self.steering.cancel()
        self._steering_state()
        for button in self.direction_buttons.values():
            button.setDown(False)

    def stop_controls(self):
        self._clear_controls()
        self.model.lease.stop()

    def _calibrate_imu(self):
        self._clear_controls()
        self.model.calibrate_imu()
        self.setFocus()

    def _zero_imu(self):
        self.stop_controls()
        self.model.zero_imu()
        self.setFocus()

    def _command(self, command, **fields):
        if command in (3, 5, 17):
            self.stop_controls()
        self.model.command(command, **fields)

    def _apply_mode(self):
        self._command(17, mode=self.mode.currentData(), beep=self.beep.isChecked())

    def _reset_odometry(self):
        self.stop_controls()
        self.steering.reset()
        self.model.reset_odometry()

    def _clear_trajectory(self):
        self.model.store.clear_trajectory()
        self.model.store.event("已清空显示轨迹；当前位姿与设备计数保留")
        self.refresh()
        self._fit_trajectory()

    def _fit_trajectory(self):
        self.trajectory.autoRange()
        self.trajectory.enableAutoRange()

    def eventFilter(self, watched, event):
        if event.type() == QEvent.Wheel and not event.modifiers() & Qt.ControlModifier:
            if watched in (self.vplot.viewport(), self.wplot.viewport()):
                # Speed plots scroll the page; the trajectory keeps native wheel zoom.
                QApplication.sendEvent(self.motion_scroll.viewport(), event)
                return True
        if event.type() == QEvent.ApplicationDeactivate or (watched is self and event.type() == QEvent.WindowDeactivate):
            self.stop_controls()
        if event.type() in (QEvent.KeyPress, QEvent.KeyRelease):
            if isinstance(watched, QWidget) and watched.window() is not self:
                return False
            key = event.key()
            if key in (Qt.Key_Space, Qt.Key_Escape):
                self.stop_controls()
                return True
            if key in self.key_actions and event.type() == QEvent.KeyRelease and not event.isAutoRepeat():
                token = f"key{key}:" + self.key_actions[key]
                if token in self._held:
                    self._release(token)
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
        self._update_imu_buttons()
        if busy:
            self.result_title.setText("等待设备反馈…")
            self.result_flags.setText("等待发送 / 应答 / 状态结果")

    def _result(self, result):
        _, text = describe_result(result)
        if result.get("kind") == "execute_motion":
            if result.get("request_accepted") and not result.get("superseded"):
                self._motion_goal_id = result["goal_id"]
            self.result_title.setText("自动运动")
            self.result_flags.setText("请求已取消" if result.get("superseded") else
                                      ("请求已受理 · 完成状态见上方" if result.get("request_accepted") else "请求未受理"))
            self.result_detail.setText(text)
            self.result_detail.setToolTip(result.get("message", ""))
            return
        if result.get("kind") == "zero_imu":
            self.result_title.setText("IMU 角度置零")
            self.result_flags.setText("置零完成" if result.get("state_confirmed") else "置零失败")
            self.result_detail.setText(text)
            self.result_detail.setToolTip(result.get("message", ""))
            return
        if result.get("kind") == "calibrate_imu":
            self.result_title.setText("IMU 方向标定")
            self.result_flags.setText("请求已取消" if result.get("superseded") else
                                      ("请求已受理 · 完成状态见上方" if result.get("request_accepted") else "请求未受理"))
            self.result_detail.setText(text)
            self.result_detail.setToolTip(result.get("message", ""))
            return
        self.result_title.setText(COMMAND_NAMES.get(result.get("command"), "重置本地里程计"))
        ack_expected = result.get("command") in (2, 3, 4, 5)
        flag = lambda key: "✓" if result.get(key) else "—"
        self.result_flags.setText(f"发送 {flag('sent')}   应答 {flag('acknowledged') if ack_expected else '不适用'}   状态 {flag('state_confirmed')}")
        self.result_detail.setText(text)
        self.result_detail.setToolTip(result.get("message", ""))
        if result.get("kind") == "reset_odometry" and result.get("state_confirmed"):
            self.refresh()
            self._fit_trajectory()

    @staticmethod
    def _cell(table, row, column, text, stale=False):
        item = table.item(row, column)
        if item is None:
            item = QTableWidgetItem()
            item.setTextAlignment(Qt.AlignCenter)
            table.setItem(row, column, item)
        item.setText(str(text))
        item.setForeground(QColor("#a08557" if stale else "#30465f"))

    @staticmethod
    def _imu_orientation_available(status, imu, now=None):
        return bool(status and status.fresh(0.3, now) and status.value["orientation_valid"] and
                    0 <= status.value["orientation_age"] + status.age(now) < 0.25 and
                    imu and imu.fresh(0.25, now) and imu.value.get("valid"))

    def _update_imu_buttons(self):
        sample = self.model.store.latest("imu_calibration")
        state = sample.value["state"] if sample else 0
        running = sample is not None and sample.fresh(0.5) and 1 <= state <= 6
        self.calibration_button.setEnabled(not self.model.busy and not running)
        self.calibration_button.setText(("标定等待中…" if state == 1 else "标定进行中…") if running else
                                         ("重新标定 IMU" if state >= 7 else "开始 IMU 标定"))
        self.zero_imu_button.setEnabled(not self.model.busy and self._imu_orientation_available(
            self.model.store.latest("imu_status"), self.model.store.latest("imu")))

    def refresh(self):
        now = time.monotonic()
        latest, history, frames, events, revision = self.model.store.snapshot()
        self._update_imu_buttons()
        calibration = latest.get("imu_calibration")
        if calibration:
            c = calibration.value
            phases = {0: "自动标定关闭", 1: "等待底盘与 IMU", 2: "标定准备中", 3: "正转测量",
                      4: "停转等待", 5: "反转回位测量", 6: "检查双向结果", 7: "方向标定完成",
                      8: "标定失败，已停止", 9: "标定已取消"}
            direction = "正向 +1" if c["yaw_sign"] > 0 else "反向 -1"
            state = phases.get(c["state"], "未知状态")
            if not calibration.fresh(0.5):
                state += " · 状态过期"
            self.calibration_label.setText(f"IMU · {state}\n航向系数 {direction}" +
                                           (" · 已测定" if c["calibrated"] else " · 配置初值"))
            self.calibration_label.setToolTip(c["message"] +
                f"\n正转 yaw 变化 {math.degrees(c['positive_delta']):+.2f}°，反转 {math.degrees(c['negative_delta']):+.2f}°")
        imu_status = latest.get("imu_status")
        imu = latest.get("imu")
        if imu_status:
            s = imu_status.value
            valid = self._imu_orientation_available(imu_status, imu, now)
            angles = ("   横滚 {:+.1f}°   俯仰 {:+.1f}°   IMU 相对航向 {:+.1f}°".format(
                *(math.degrees(imu.value[k]) for k in ("roll", "pitch", "yaw"))) if valid else "")
            self.imu_label.setText(("IMU 姿态有效" if valid else "IMU 姿态不可用 · 使用底盘里程计") + angles +
                                   f"   RX {s['rx_frames']}   故障 0x{s['faults']:X}")
            self.imu_label.setToolTip(s["message"])
        steering = self.steering
        status = latest.get("motion_status")
        target_yaw = steering.target
        if status and status.value["goal_id"]:
            motion = status.value
            remaining = f"{motion['remaining']:+.3f} m" if motion["mode"] == 3 else f"{math.degrees(motion['remaining']):+.1f}°"
            stale = " · 状态过期" if not status.fresh(0.5, now) else ""
            self.motion_status_label.setText(f"{motion['message']}{stale}\n剩余 {remaining} · 前馈 {motion['feedforward_gain']:.3f}")
            if motion["active"] and not stale and not self._held:
                target_yaw = motion["target_yaw"]
        target = "" if target_yaw is None else f" · 目标 {math.degrees(wrap(target_yaw)):+.1f}°"
        self.steering_label.setText(steering.mode + target + "\nW/S 直行保向 · A/D 自由转弯")
        recording, count, error = self.model.store.recording_status()
        self.record_button.setText("结束路径记录" if recording else "开始记录路径")
        self.record_label.setText((f"记录中 · {count} 个点" if recording else f"已记录 {count} 个点") +
                                  (" · 写入失败：" + error if error else ""))
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
                self.pose_label.setText(f"X  {p['x']:+.3f} m    Y  {p['y']:+.3f} m    航向  {math.degrees(p['yaw']):+.1f}°"
                                       f"    ·    {p['frame']} → {p['child']}" +
                                       (" · IMU 修正" if p.get("imu_active") else " · 底盘积分") +
                                       ("    ·    反馈过期" if not pose.fresh(0.3, now) else ""))
                self.position.setData([p['x']], [p['y']])
                self.heading.setData([p['x'], p['x'] + 0.08 * math.cos(p['yaw'])],
                                     [p['y'], p['y'] + 0.08 * math.sin(p['yaw'])])
                if target_yaw is not None:
                    self.target_heading.setData([p['x'], p['x'] + 0.14 * math.cos(target_yaw)],
                                                [p['y'], p['y'] + 0.14 * math.sin(target_yaw)])
                else:
                    self.target_heading.setData([], [])
            else:
                self.pose_label.setText("X —    Y —    航向 —    ·    等待里程计反馈")
                self.position.setData([], [])
                self.heading.setData([], [])
                self.target_heading.setData([], [])
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
        previous_stamp = None
        for stamp, pose in history["odom"]:
            point = (pose["x"], pose["y"])
            if previous_stamp is not None and stamp - previous_stamp > 0.3:
                points.append((float("nan"), float("nan")))
            if not points or point != points[-1]:
                points.append(point)
            previous_stamp = stamp
        self.path_curve.setData([x for x, _ in points] if len(points) > 1 else [],
                                [y for _, y in points] if len(points) > 1 else [], connect="finite")

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

    def _toggle_recording(self):
        if self.model.store.recording_status()[0]:
            self.model.store.stop_recording()
            self.model.store.event("路径记录已保存")
            return
        self.stop_controls()
        name, _ = QFileDialog.getSaveFileName(self, "记录路径", "vehicle_path.csv", "CSV (*.csv)")
        if name:
            try:
                self.model.store.start_recording(name)
                self.model.store.event("开始记录路径 · " + name)
            except (OSError, RuntimeError) as error:
                self.model.store.event("无法记录路径 · " + str(error), "error")

    def export_path(self):
        self.stop_controls()
        name, _ = QFileDialog.getSaveFileName(self, "导出当前路径", "vehicle_path_snapshot.csv", "CSV (*.csv)")
        if name:
            try:
                self.model.store.export_path(name)
                self.model.store.event("路径已导出 · " + name)
            except OSError as error:
                self.model.store.event("导出失败 · " + str(error), "error")

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
