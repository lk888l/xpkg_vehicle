#!/usr/bin/env bash
# Run explicitly with sudo. Configures networking; sends no application CAN frames.
set -euo pipefail
case "${1:-}" in
  chassis)
    interface="${2:-can0}"
    ip link set dev "$interface" down
    ip link set dev "$interface" type can bitrate 500000 fd off
    ip link set dev "$interface" up
    ;;
  imu)
    interface="${2:-can1}"
    ip link set dev "$interface" down
    ip link set dev "$interface" type can bitrate 1000000 sample-point 0.8 \
      dbitrate 4000000 dsample-point 0.8 fd on
    ip link set dev "$interface" up
    ;;
  *) printf 'Usage: sudo %s {chassis [can0]|imu [can1]}\n' "$0" >&2; exit 2 ;;
esac
ip -details -statistics link show dev "$interface"
