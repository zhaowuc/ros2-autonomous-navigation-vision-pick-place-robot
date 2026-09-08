#!/usr/bin/env bash
set +e

echo "[roscar] stopping old simulation, navigation, rviz, and roscar processes..."

patterns=(
  "dynamic_nav_bringup"
  "dynamic_nav_ws"
  "gzserver"
  "gzclient"
  "gazebo"
  "spawn_entity.py"
  "nav2_container"
  "nav2_collision_monitor"
  "nav2_lifecycle_manager"
  "amcl"
  "bt_navigator"
  "controller_server"
  "planner_server"
  "smoother_server"
  "waypoint_follower"
  "velocity_smoother"
  "map_server"
  "locked_planner_server"
  "ros2 service call /global_costmap"
  "ros2 service call /local_costmap"
  "ros2 param get /controller_server"
  "[r]os2 daemon stop"
  "[r]os2 daemon start"
  "[r]os2 launch roscar_bringup"
  "[w]eb_mapping.launch.py"
  "[m]apping.launch.py"
  "[b]ase.launch.py"
  "rviz2"
  "static_transform_publisher"
  "stationary_odom_to_base"
  "robot_state_publisher"
  "c50c_base_driver_node"
  "scan_front_filter_node"
  "arena_boundary"
  "lslidar_driver_node"
  "async_slam_toolbox_node"
  "map_saver_cli"
  "operator_gui"
)

for pattern in "${patterns[@]}"; do
  pkill -TERM -f "$pattern" 2>/dev/null || true
done

sleep 2

for pattern in "${patterns[@]}"; do
  pkill -KILL -f "$pattern" 2>/dev/null || true
done

source /opt/ros/humble/setup.bash 2>/dev/null || true
timeout 5s ros2 daemon stop >/dev/null 2>&1 || true
pkill -TERM -f "[r]os2 daemon stop" 2>/dev/null || true
pkill -TERM -f "[r]os2 daemon start" 2>/dev/null || true
pkill -TERM -f "[r]os2-daemon" 2>/dev/null || true
sleep 0.5
pkill -KILL -f "[r]os2 daemon stop" 2>/dev/null || true
pkill -KILL -f "[r]os2 daemon start" 2>/dev/null || true
pkill -KILL -f "[r]os2-daemon" 2>/dev/null || true
timeout 5s ros2 daemon start >/dev/null 2>&1 || true

rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
rm -rf "${HOME}/.ros/log"/* 2>/dev/null || true

echo "[roscar] clean runtime done."
