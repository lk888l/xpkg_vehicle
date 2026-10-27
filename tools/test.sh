#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source tools/gui_env.sh
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}"
export VEHICLE_TEST_CAN="${VEHICLE_TEST_CAN:-vcan_veh_test}"
created=0
cleanup() {
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
    raise SystemExit('Integration tests require an isolated vcan interface')
PY
ip link set "$VEHICLE_TEST_CAN" up
colcon test --return-code-on-test-failure --event-handlers console_cohesion+
colcon test-result --verbose
