#!/usr/bin/env bash
set -Eeo pipefail

: "${ROS_DISTRO:=humble}"
: "${ROS_WS:=/root/ros2_ws}"
: "${START_CMD:=ros2 run unitree_ros2_interface interface_node}"

# Evita 'unbound variable' nelle setup ROS se qualche env manca
: "${AMENT_TRACE_SETUP_FILES:=}"

# Sorgo ambienti ROS
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[ -f "${ROS_WS}/install/setup.bash" ] && source "${ROS_WS}/install/setup.bash"

exec bash -lc "$START_CMD"