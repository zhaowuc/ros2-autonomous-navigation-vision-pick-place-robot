#!/usr/bin/env bash
set -euo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 command not found. Source ROS2 Humble first."
  exit 1
fi

echo "Installed ROS2 packages matching N10 / LSLiDAR / Leishen / lidar:"
ros2 pkg list | grep -Ei "n10|lslidar|leishen|lidar" || true
