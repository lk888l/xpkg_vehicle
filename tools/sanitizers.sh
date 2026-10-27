#!/usr/bin/env bash
# ROS-independent core can also be compiled and tested without ament.
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
output=$(mktemp -d /tmp/vehicle-sanitizers.XXXXXX)
trap 'rm -rf -- "$output"' EXIT
gtest="${GTEST_SOURCE:-/opt/ros/jazzy/src/gtest_vendor}"
g++ -std=c++17 -g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer \
  -Isrc/vehicle_core/include -I"$gtest/include" -I"$gtest" \
  src/vehicle_core/src/*.cpp src/vehicle_core/test/test_core.cpp \
  "$gtest/src/gtest-all.cc" "$gtest/src/gtest_main.cc" -pthread -o "$output/test_core"
"$output/test_core"
