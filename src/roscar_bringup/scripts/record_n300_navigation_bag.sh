#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "${ROSCAR_SETUP:-install/setup.bash}"
set -euo pipefail

output_root="${1:-${ROS_HOME:-${HOME}/.ros}/roscar_data/bags}"
mkdir -p "${output_root}"
mode_value=true
mode_file="${ROS_HOME:-${HOME}/.ros}/roscar_data/use_fused_odom"
if [[ -r "${mode_file}" ]]; then
  IFS= read -r mode_value < "${mode_file}" || true
fi
if [[ "${mode_value}" == "true" ]]; then
  mode_name=fused
else
  mode_name=rollback
fi
output_path="${output_root}/n300_navigation_${mode_name}_$(date +%Y%m%d_%H%M%S)"

mapfile -t live_topics < <(ros2 topic list)
declare -A is_live=()
for topic in "${live_topics[@]}"; do
  is_live["${topic}"]=1
done

candidates=(
  /wheel/odom_raw
  /odom
  /imu/n300/data
  /imu/n300/rpy
  /imu/legacy/data_raw
  /scan_raw
  /scan_slam_filtered
  /cmd_vel
  /cmd_vel_nav
  /cmd_vel_nav_raw
  /cmd_vel_selected
  /cmd_vel_smoothed
  /cmd_vel_safe
  /tf
  /tf_static
  /diagnostics
  /amcl_pose
  /particle_cloud
)
topics=()
for topic in "${candidates[@]}"; do
  if [[ -n "${is_live[${topic}]:-}" ]]; then
    topics+=("${topic}")
  fi
done

required=(/odom /imu/n300/data /tf /tf_static /diagnostics)
if [[ "${mode_value}" == "true" ]]; then
  required+=(/wheel/odom_raw)
fi
for topic in "${required[@]}"; do
  if [[ -z "${is_live[${topic}]:-}" ]]; then
    echo "Required live topic is missing: ${topic}" >&2
    exit 2
  fi
done

printf 'Recording %d live topics to %s\n' "${#topics[@]}" "${output_path}"
printf '%s\n' "${topics[@]}"
exec ros2 bag record -o "${output_path}" "${topics[@]}"
