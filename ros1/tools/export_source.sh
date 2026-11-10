#!/usr/bin/env bash
# Dereference shared files so the archive is a standalone catkin workspace.
set -euo pipefail
workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
archive="${1:-/tmp/vehicle_noetic_source.tgz}"
tar --dereference --exclude='__pycache__' --exclude='*.pyc' --exclude='.pytest_cache' \
  --exclude='./build' --exclude='./devel' --exclude='./install' --exclude='./test_results' \
  -czf "$archive" -C "$workspace" .
printf '%s\n' "$archive"
