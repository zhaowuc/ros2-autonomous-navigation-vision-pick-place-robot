#!/usr/bin/env bash
set -euo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  echo "Please install/source ROS2 Humble first: source /opt/ros/humble/setup.bash"
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  python3-pip \
  python3-serial \
  python3-yaml \
  ros-humble-joint-state-publisher-gui \
  ros-humble-nav2-map-server \
  ros-humble-robot-state-publisher \
  ros-humble-slam-toolbox \
  ros-humble-tf2-ros \
  ros-humble-xacro

python3 - <<'PY' || python3 -m pip install --user fastapi "uvicorn[standard]"
import fastapi
import uvicorn
PY

echo "Dependencies installed. If this user is not in dialout, run:"
echo "  sudo usermod -aG dialout $USER"
echo "Then log out and back in."
