#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source /opt/ros/noetic/setup.bash
source devel/setup.bash
action="${1:-driver}"
if [[ $# -gt 0 ]]; then shift; fi
case "$action" in
  driver) exec roslaunch vehicle_driver vehicle.launch "$@" ;;
  imu) exec roslaunch vehicle_driver imu.launch "$@" ;;
  gui) exec roslaunch vehicle_gui console.launch "$@" ;;
  *) printf 'Usage: %s {driver|imu|gui} [launch arguments]\n' "$0" >&2; exit 2 ;;
esac
