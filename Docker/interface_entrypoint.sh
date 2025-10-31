# /usr/local/bin/interface_entrypoint.sh
#!/usr/bin/env bash
set -Eeuo pipefail

: "${ROS_DISTRO:=humble}"
: "${ROS_WS:=/root/ros2_ws}"
: "${START_CMD:=ros2 run unitree_ros2_interface interface_node}"

# Percorsi CycloneDDS (template -> finale)
: "${TEMPLATE_PATH:=${ROS_WS}/cyclonedds/cyclonedds.xml.template}"
: "${FINAL_XML:=/etc/cyclonedds/cyclonedds.xml}"

mkdir -p /etc/cyclonedds
TEMPLATE_PATH="$TEMPLATE_PATH" FINAL_XML="$FINAL_XML" "$ROS_WS/cyclonedds/setup_cyclonedds.sh"

# 2) esporta l’URI per CycloneDDS
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="file://${FINAL_XML}"

# 3) sourcia ROS e avvia
: "${AMENT_TRACE_SETUP_FILES:=}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[ -f "${ROS_WS}/install/setup.bash" ] && source "${ROS_WS}/install/setup.bash"
exec bash -lc "$START_CMD"
