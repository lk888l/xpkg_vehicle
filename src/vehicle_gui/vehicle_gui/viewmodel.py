# SPDX-License-Identifier: Apache-2.0
from PyQt5.QtCore import QObject, pyqtSignal
from .model import COMMAND_NAMES, MotionLease, TelemetryStore, describe_result
from .ros_worker import RosWorker


class VehicleViewModel(QObject):
    result_changed = pyqtSignal(object)
    busy_changed = pyqtSignal(bool)
    endpoint_changed = pyqtSignal(str)

    def __init__(self, ros_args=None, worker_factory=RosWorker, parent=None):
        super().__init__(parent)
        self.store = TelemetryStore()
        self.lease = MotionLease()
        self.worker = worker_factory(self.store, self.lease, ros_args, self)
        self.worker.ready.connect(self.endpoint_changed)
        self.worker.outcome.connect(self._outcome)
        self.worker.failed.connect(self._failed)
        self.busy = False
        self.last_result = None

    def start(self):
        self.worker.start()

    def command(self, command, **fields):
        if command in (3, 5, 17):
            self.lease.stop()
        self._submit("command", command, **fields)

    def reset_odometry(self):
        self.lease.stop()
        self._submit("reset_odometry")

    def zero_imu(self):
        self.lease.stop()
        self._submit("zero_imu")

    def calibrate_imu(self):
        revision = self.lease.handoff()
        if not self._submit("calibrate_imu", motion_revision=revision):
            self.lease.stop(revision)

    def execute_motion(self, **fields):
        revision = self.lease.handoff()
        if not self._submit("execute_motion", motion_revision=revision, **fields):
            self.lease.stop(revision)
            return None
        return revision

    def _submit(self, kind, command=0, **fields):
        if not self.worker.isRunning():
            self.store.event("ROS 通信线程未运行，请重新启动控制台", "error")
            return False
        if self.busy:
            self.store.event("正在等待上一条设备命令；停止运动仍可使用", "warning")
            return False
        if self.worker.submit(kind, command, **fields):
            self.busy = True
            self.busy_changed.emit(True)
            name = {"calibrate_imu": "IMU 方向标定", "zero_imu": "IMU 角度置零",
                    "execute_motion": "自动运动"}.get(
                kind, COMMAND_NAMES.get(command, "重置本地里程计"))
            self.store.event("发送 · " + name)
            return True
        else:
            self.store.event("命令队列已满，请稍后重试", "warning")
            return False

    def _outcome(self, result):
        self.busy = False
        self.busy_changed.emit(False)
        self.last_result = result
        level, text = describe_result(result)
        self.store.event(text, level)
        self.result_changed.emit(result)

    def _failed(self, message):
        self.busy = False
        self.busy_changed.emit(False)
        self.lease.stop()
        self.store.event("ROS 连接异常 · " + message, "error")

    def shutdown(self):
        self.lease.stop()
        self.worker.requestInterruption()
        finished = self.worker.wait(1500)
        if finished:
            self.store.stop_recording()
        return finished
