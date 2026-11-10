from vehicle_gui.model import MotionLease, Sample, TelemetryStore, connection_state, describe_result


def test_motion_lease_requires_fresh_input_and_stops_once():
    lease = MotionLease(0.2)
    assert lease.sample(10) is None
    assert lease.renew(0.1, -0.2, now=10)
    assert lease.sample(10.15) == (0.1, -0.2)
    assert lease.sample(10.21) == (0, 0)
    assert lease.sample(10.30) is None
    lease.renew(-0.1, 0, now=11)
    lease.stop()
    assert lease.sample(11.01) == (0, 0)
    assert lease.sample(11.02) is None


def test_invalid_input_and_clock_reversal_do_not_keep_motion_alive():
    lease = MotionLease()
    lease.renew(0.1, 0, now=10)
    assert not lease.renew(float("nan"), 0, now=10.1)
    assert lease.sample(10.1) == (0, 0)
    lease.renew(0.1, 0, now=20)
    assert lease.sample(19) == (0, 0)


def test_calibration_handoff_removes_old_motion_but_preserves_new_operator_input():
    lease = MotionLease()
    lease.renew(.1, .2, now=10)
    lease.stop()
    revision = lease.handoff()
    assert lease.sample(10.1) is None  # No trailing zero to cancel the new calibration.
    assert lease.is_current(revision)
    lease.stop()
    assert not lease.is_current(revision)
    assert lease.sample(10.2) == (0, 0)
    revision = lease.handoff()
    lease.renew(.1, -.2, now=11)
    assert not lease.is_current(revision)
    assert not lease.stop(revision)  # A failed old request must not stop newer manual input.
    assert lease.sample(11.1) == (.1, -.2)


def test_calibration_acceptance_is_not_reported_as_completion():
    result = dict(kind="calibrate_imu", sent=True, request_accepted=True, state_confirmed=False)
    level, text = describe_result(result)
    assert level == "info" and "请求已受理" in text and "标定完成" not in text
    rejected = dict(result, request_accepted=False, message="use_imu_heading is disabled")
    assert describe_result(rejected)[0] == "warning"
    assert "use_imu_heading is disabled" in describe_result(rejected)[1]
    assert "取消" in describe_result(dict(result, superseded=True))[1]


def test_goal_keepalive_expires_and_cannot_revive_after_operator_override():
    lease = MotionLease()
    revision = lease.handoff()
    assert lease.keepalive(revision, now=10)
    assert lease.fresh(revision, now=10.1)
    assert not lease.fresh(revision, now=10.3)
    assert lease.sample(10.1) is None  # Renewal must never publish cmd_vel over a goal.
    lease.stop()
    assert not lease.keepalive(revision, now=11)
    assert not lease.fresh(revision, now=11)
    assert lease.sample(11) == (0, 0)
    result = dict(kind="execute_motion", sent=True, request_accepted=True)
    assert "请求已受理" in describe_result(result)[1]
    assert "取消" in describe_result(dict(result, superseded=True))[1]


def test_cached_transient_state_is_not_falsely_online():
    assert connection_state(None, now=10) == ("等待设备", False)
    sample = Sample({"online": True, "heartbeat_age": 0.1}, 10)
    assert connection_state(sample, now=10.2)[1]
    assert not connection_state(sample, now=12)[1]
    old_heartbeat = Sample({"online": True, "heartbeat_age": 20}, 10)
    assert not connection_state(old_heartbeat, now=10.01)[1]


def test_history_raw_frames_and_event_log_are_bounded():
    store = TelemetryStore(capacity=5)
    for i in range(300):
        store.update("velocity", {"vx": i, "wz": 0}, now=i)
        store.frame(i, {"data": "00", "length": 1, "error": False})
        store.event(str(i))
    latest, history, frames, events, revision = store.snapshot()
    assert latest["velocity"].value["vx"] == 299
    assert len(history["velocity"]) == 5
    assert len(frames) == 64
    assert len(events) == 200 and revision == 300
    store.clear_history()
    assert not store.snapshot()[1]["velocity"]


def test_ack_is_not_misrepresented_as_physical_confirmation():
    clear = {"sent": True, "acknowledged": True, "state_confirmed": False, "command": 5}
    assert "实际效果" in describe_result(clear)[1]
    enable = dict(clear, command=3)
    assert describe_result(enable)[0] == "warning"
    mode = dict(clear, command=17, state_confirmed=True)
    assert "刹车" in describe_result(mode)[1] and "未知" in describe_result(mode)[1]


def test_clearing_trajectory_preserves_pose_counters_and_speed_history():
    store = TelemetryStore()
    store.update("odom", {"x": 3.0, "y": 2.0, "yaw": 0.5})
    store.update("mileage", {"left": 2000, "right": 2200})
    store.update("velocity", {"vx": 0.1, "wz": 0.0})
    store.update("command", {"vx": 0.0, "wz": 0.0})
    before, history_before, _, _, _ = store.snapshot()
    store.clear_trajectory()
    latest, history, _, _, _ = store.snapshot()
    assert latest == before
    assert not history["odom"]
    assert history["velocity"] == history_before["velocity"]
    assert history["command"] == history_before["command"]

    store.update("odom", {"x": 3.1, "y": 2.0, "yaw": 0.5})
    store.clear_trajectory(clear_pose=True)
    latest, history, _, _, _ = store.snapshot()
    assert "odom" not in latest and not history["odom"]
    assert latest["mileage"] == before["mileage"]
    assert history["velocity"] == history_before["velocity"]
