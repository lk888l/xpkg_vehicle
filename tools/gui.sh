#!/usr/bin/env bash
# Host launcher. Pass launch arguments, e.g. start_driver:=false or namespace:=robot1.
set -eo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
container="${VEHICLE_CONTAINER:-ros2_jazzy}"
container_path="${VEHICLE_CONTAINER_PATH:-/workspace/$(basename -- "$repo")}"
auth_file=$(mktemp /tmp/vehicle-gui-xauth.XXXXXX)
auth_name=$(basename -- "$auth_file")
auth_args=()
cleanup() {
  rm -f -- "$auth_file"
  docker exec "$container" rm -f -- "/tmp/$auth_name" >/dev/null 2>&1 || true
}
trap cleanup EXIT
if [[ -z "${DISPLAY:-}" ]]; then
  printf 'DISPLAY is unset. Run this script from your graphical desktop terminal.\n' >&2
  exit 1
fi
if command -v xauth >/dev/null 2>&1; then
  xauth extract "$auth_file" "$DISPLAY" >/dev/null 2>&1
  if [[ -s "$auth_file" ]]; then
    docker cp "$auth_file" "$container:/tmp/$auth_name" >/dev/null
    auth_args=(-e "XAUTHORITY=/tmp/$auth_name" -e "XAUTHLOCALHOSTNAME=$(hostname)")
  fi
fi
# Reuse an existing host CJK font; do not download a font package.
if [[ -f /usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc ]]; then
  if ! docker exec "$container" test -f "$container_path/.gui_deps/NotoSansCJK-Regular.ttc"; then
    docker exec "$container" mkdir -p "$container_path/.gui_deps"
    docker cp /usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc \
      "$container:$container_path/.gui_deps/NotoSansCJK-Regular.ttc" >/dev/null
  fi
fi
tty_args=(-i)
if [[ -t 0 && -t 1 ]]; then tty_args=(-it); fi
docker exec "${tty_args[@]}" -e "DISPLAY=$DISPLAY" "${auth_args[@]}" "$container" bash -c '
  set -e
  source /opt/ros/jazzy/setup.bash
  source "$1/install/setup.bash"
  source "$1/tools/gui_env.sh"
  shift
  exec ros2 launch vehicle_gui console.launch.py "$@"
' _ "$container_path" "$@"
