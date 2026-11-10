#!/usr/bin/env bash
# Only the vcan setup/recovery needs NET_ADMIN. No physical CAN device is used.
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export VEHICLE_TEST_CAN="${VEHICLE_TEST_CAN:-vcan_noetic}"
export ROS_MASTER_URI="${VEHICLE_TEST_MASTER_URI:-http://127.0.0.1:11329}"
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
export ROS_HOME="$PWD/test_results/ros-home"
export ROS_LOG_DIR="$PWD/test_results/ros-logs"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"
export PYQTGRAPH_QT_LIB=PyQt5
export PYTHONPATH="$PWD/test${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p test_results "$ROS_LOG_DIR"
created=0
master_pid=
cleanup() {
  if [[ -n "$master_pid" ]]; then kill -INT "$master_pid" 2>/dev/null || true; wait "$master_pid" 2>/dev/null || true; fi
  if [[ "$created" == 1 ]]; then ip link delete "$VEHICLE_TEST_CAN"; fi
}
trap cleanup EXIT
if ! ip link show "$VEHICLE_TEST_CAN" >/dev/null 2>&1; then
  ip link add dev "$VEHICLE_TEST_CAN" type vcan
  created=1
fi
python3 - "$VEHICLE_TEST_CAN" <<'PY'
import json, subprocess, sys
info = json.loads(subprocess.check_output(['ip', '-json', '-details', 'link', 'show', sys.argv[1]]))
if info[0].get('linkinfo', {}).get('info_kind') != 'vcan':
    raise SystemExit('REFUSED: integration tests require an isolated kernel vcan interface')
PY
ip link set "$VEHICLE_TEST_CAN" up
# Refuse to attach to somebody else's master: tests own their parameters and names.
python3 - <<'PY'
import socket, urllib.parse, os
uri = urllib.parse.urlparse(os.environ['ROS_MASTER_URI'])
with socket.socket() as s:
    if s.connect_ex((uri.hostname, uri.port)) == 0:
        raise SystemExit('Test master port is occupied; set VEHICLE_TEST_MASTER_URI')
PY
master_port=$(python3 -c 'import os, urllib.parse; print(urllib.parse.urlparse(os.environ["ROS_MASTER_URI"]).port)')
roscore -p "$master_port" >test_results/roscore.log 2>&1 &
master_pid=$!
python3 - <<'PY'
import rosgraph, time
deadline = time.monotonic() + 10
while time.monotonic() < deadline:
    if rosgraph.is_master_online():
        break
    time.sleep(.1)
else:
    raise SystemExit('Test roscore did not start')
PY
catkin_make -j"${VEHICLE_BUILD_JOBS:-2}" run_tests_vehicle_core
catkin_test_results build/test_results
python3 -m pytest -q src/vehicle_driver/test src/vehicle_gui/test \
  --junitxml=test_results/pytest.xml "$@"
