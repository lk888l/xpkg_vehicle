#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source /opt/ros/noetic/setup.bash
catkin_make -j"${VEHICLE_BUILD_JOBS:-2}" -DCMAKE_BUILD_TYPE=RelWithDebInfo "$@"
