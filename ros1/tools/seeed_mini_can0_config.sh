#!/usr/bin/env bash
# Board-specific: Seeed reComputer Mini, JetPack 5 / L4T R35.
# See https://wiki.seeedstudio.com/recomputer_jetson_mini_hardware_interfaces_usage/#can
# GPIO 0:43 must be held low by vehicle-shared-can.service before this runs.
set -euo pipefail
grep -q 'j40mini' /etc/nv_tegra_release || { echo 'Not the validated Seeed Mini image' >&2; exit 1; }
ip link set dev can0 down
# NVIDIA Orin CAN0 pinmux: TX and RX respectively. Reapplied after every boot.
busybox devmem 0x0c303010 32 0x0000c400
busybox devmem 0x0c303018 32 0x0000c458
ip link set dev can0 type can bitrate 500000 sample-point 0.8 fd off \
  loopback off listen-only off berr-reporting on restart-ms 0
ip link set dev can0 up
ip -details link show can0
