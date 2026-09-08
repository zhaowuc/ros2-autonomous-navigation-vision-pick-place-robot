#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
source "${ROSCAR_SETUP:-install/setup.bash}"

exec ros2 launch roscar_bringup mapping.launch.py "$@"
