#!/usr/bin/env bash
# Host-side helper for the existing host-network ros2_jazzy container.
set -eo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
container="${VEHICLE_CONTAINER:-ros2_jazzy}"
container_path="${VEHICLE_CONTAINER_PATH:-/workspace/$(basename -- "$repo")}"
action="${1:-build}"
case "$action" in
  build)
    owner=$(stat -c '%u:%g' "$repo")
    docker exec --user "$owner" "$container" bash -c '
      source /opt/ros/jazzy/setup.bash
      cd -- "$1"
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    ' _ "$container_path"
    ;;
  test)
    docker exec "$container" bash -c 'cd -- "$1"; exec bash tools/test.sh' _ "$container_path"
    ;;
  run)
    docker exec -it "$container" bash -c '
      source /opt/ros/jazzy/setup.bash
      source "$1/install/setup.bash"
      exec ros2 launch vehicle_driver vehicle.launch.py
    ' _ "$container_path"
    ;;
  gui-setup)
    docker exec "$container" bash -c 'cd -- "$1"; exec bash tools/install_gui_deps.sh' _ "$container_path"
    ;;
  gui)
    shift
    exec bash "$repo/tools/gui.sh" "$@"
    ;;
  *) printf 'Usage: %s {build|test|run|gui-setup|gui}\n' "$0" >&2; exit 2 ;;
esac
