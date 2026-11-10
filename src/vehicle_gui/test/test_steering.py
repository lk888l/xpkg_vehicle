import csv
import math
import pytest
from vehicle_gui.model import TelemetryStore
from vehicle_gui.steering import SteeringController, wrap


def test_keyboard_curves_latch_new_heading_and_reverse_corrects_same_way():
    c = SteeringController()
    assert c.manual(.2, 0, 1, True, .5) == (.2, 0)
    assert c.manual(.2, 0, 1.1, True, .5)[1] < 0
    assert c.manual(.2, .3, 1.1, True, .5) == (.2, .3)
    assert c.manual(.2, 0, 1.6, True, .5) == (.2, 0)
    assert c.manual(-.2, 0, 1.7, True, .5)[1] < 0
    assert c.manual(.2, .3, None, True, .5) == (.2, .3)
    assert c.manual(.2, 0, None, True, .5) == (.2, 0)


@pytest.mark.parametrize("degrees", [90, -90, 360, -360])
def test_relative_turn_crosses_wrap_and_converges(degrees):
    c = SteeringController()
    yaw = math.radians(179)
    c.start_turn(math.radians(degrees), True, wrap(yaw))
    rate = 0
    for tick in range(1000):
        _, rate = c.step_turn(wrap(yaw), rate, .7, tick * .05)
        yaw += rate * .05
        if not c.turning:
            break
    assert not c.turning
    assert abs(yaw - math.radians(179 + degrees)) < c.tolerance
    assert c.mode == "转向完成"


def test_absolute_heading_shortest_path_stale_cancellation_and_manual_takeover():
    c = SteeringController()
    c.start_turn(math.radians(-170), False, math.radians(170))
    assert c.step_turn(math.radians(170), 0, .4, 0)[1] > 0
    assert c.step_turn(None, 0, .4, .1) == (0, 0)
    assert not c.turning and c.target is None
    c.start_turn(1, True, 0)
    assert c.manual(.1, -.3, 0, True, .4) == (.1, -.3)
    assert not c.turning


def test_path_stream_keeps_every_point_beyond_display_capacity_and_reset_segments(tmp_path):
    store = TelemetryStore(path_capacity=3)
    path = tmp_path / "path.csv"
    store.start_recording(path)
    store.update("heading", {"yaw": 0.5, "wz": 0, "imu_active": True, "imu_age": .01}, now=1)
    store.update("steering", {"target": 1.0, "mode": "直行保向"}, now=1)
    for i in range(10):
        store.update("odom", {"x": i*.01, "y": 0, "yaw": .5}, now=1+i*.02)
    store.clear_trajectory(clear_pose=True)
    store.update("odom", {"x": 0, "y": 0, "yaw": 0}, now=2)
    store.stop_recording()
    rows = list(csv.DictReader(path.open()))
    assert len(rows) == 11
    assert rows[0]["imu_active"] == "True"
    assert rows[0]["target_yaw"] == "1.0"
    assert rows[-1]["segment"] == "1" and rows[-1]["imu_active"] == "False"
    assert len(store.snapshot()[1]["odom"]) == 1
    assert store.recording_status() == (False, 11, "")
