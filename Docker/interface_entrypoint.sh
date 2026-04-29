#!/usr/bin/env bash
set -Eeuo pipefail

# ----------------------------
# Defaults configurabili via env
# ----------------------------
: "${ROS_DISTRO:=humble}"
: "${ROS_WS:=/root/ros2_ws}"
: "${LAUNCH_PKG:=unitree_ros2_interface}"

: "${DEBUG_MODE:=0}"  # 0: lancia ros2 launch, 1: debug (sleep infinity)

# Feature flags (0/1 oppure true/false)
: "${ENABLE_CAMERA:=1}"
: "${ENABLE_ULTRASOUND:=0}"
: "${ENABLE_FACE_LIGHTS:=1}"
: "${ENABLE_LEGGED_SDK:=1}"
: "${ENABLE_CUSTOM_SPORT:=1}"

# Camera options
: "${PUBLISH_RECTIFIED:=false}"
: "${PUBLISH_DEPTH:=false}"
: "${PUBLISH_PCL:=false}"
: "${CAMERA_BASE:=false}"
: "${NAMESPACE:=unitree_go1}"

# Nomi launch file
: "${LAUNCH_HEAD:=head_board.launch.py}"
: "${LAUNCH_BODY:=body_board.launch.py}"
: "${LAUNCH_MAIN:=main_board.launch.py}"
: "${LAUNCH_PI:=pi_board.launch.py}"
: "${LAUNCH_INTERFACE:=interface.launch.py}"

# Dove si trova lo script DDS (copiato nell’immagine)
: "${DDS_ENV_SCRIPT:=/usr/local/bin/unitree_dds_env.sh}"

to_bool() {
  case "${1,,}" in
    1|true|yes|y|on)  echo "true" ;;
    0|false|no|n|off) echo "false" ;;
    *)                echo "${1}" ;;
  esac
}

# ----------------------------
# Source ROS env
# ----------------------------
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f "${ROS_WS}/install/setup.bash" ]]; then
  source "${ROS_WS}/install/setup.bash"
fi
set -u

# ----------------------------
# DDS env (role/ip/profile selection)
# ----------------------------
if [[ ! -f "${DDS_ENV_SCRIPT}" ]]; then
  echo >&2 "[entrypoint] ERROR: DDS env script not found: ${DDS_ENV_SCRIPT}"
  exit 10
fi
source "${DDS_ENV_SCRIPT}"

# (opzionale ma consigliato) fai in modo che le shell docker exec vedano sempre le env:
# bash interattivo non-login legge ~/.bashrc 
if ! grep -q 'unitree_env.sh' /root/.bashrc 2>/dev/null; then
  echo '[[ -f /run/unitree_env.sh ]] && source /run/unitree_env.sh' >> /root/.bashrc
fi

# ----------------------------
# Selezione launch file per ruolo
# ----------------------------
case "${BOARD_ROLE}" in
  head) launch_file="${LAUNCH_HEAD}" ;;
  body) launch_file="${LAUNCH_BODY}" ;;
  main) launch_file="${LAUNCH_MAIN}" ;;
  pi)   launch_file="${LAUNCH_PI}" ;;
  *)    launch_file="${LAUNCH_INTERFACE}" ;;
esac

# ----------------------------
# Launch args
# ----------------------------
args=(
  "board_ip:=${BOARD_IP}"
  "board_role:=${BOARD_ROLE}"

  "enable_camera:=$(to_bool "${ENABLE_CAMERA}")"
  "enable_ultrasound:=$(to_bool "${ENABLE_ULTRASOUND}")"
  "enable_face_lights:=$(to_bool "${ENABLE_FACE_LIGHTS}")"
  "enable_legged_sdk:=$(to_bool "${ENABLE_LEGGED_SDK}")"
  "enable_custom_sport:=$(to_bool "${ENABLE_CUSTOM_SPORT}")"
  "publish_rectified:=$(to_bool "${PUBLISH_RECTIFIED}")"
  "publish_depth:=$(to_bool "${PUBLISH_DEPTH}")"
  "publish_pcl:=$(to_bool "${PUBLISH_PCL}")"
  "camera_base:=$(to_bool "${CAMERA_BASE}")"
  "namespace:=${NAMESPACE}"
)

if [[ "${DEBUG_MODE}" != "1" ]]; then
  echo "[entrypoint] launch: ros2 launch ${LAUNCH_PKG} ${launch_file} ${args[*]}"
  exec ros2 launch "${LAUNCH_PKG}" "${launch_file}" "${args[@]}"
else
  echo "[entrypoint] DEBUG_MODE=1 -> container alive"
  sleep infinity
fi
