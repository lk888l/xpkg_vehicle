#!/usr/bin/env bash
# Run with sudo on the validated Seeed Mini only; does not start ROS or enable motors.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
grep -q 'j40mini' /etc/nv_tegra_release || { echo 'Not the validated Seeed Mini image' >&2; exit 1; }
command -v gpioset >/dev/null
install -m 0755 seeed_mini_can0_config.sh /usr/local/sbin/vehicle-shared-can-config
install -m 0644 vehicle-shared-can.service /etc/systemd/system/vehicle-shared-can.service
systemctl daemon-reload
systemctl enable vehicle-shared-can.service
systemctl restart vehicle-shared-can.service
systemctl --no-pager status vehicle-shared-can.service
