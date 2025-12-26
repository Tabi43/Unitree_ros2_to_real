#!/usr/bin/env bash
set -Eeuo pipefail

# ----------------------------
# Defaults configurabili via env
# ----------------------------
: "${ROS_DISTRO:=humble}"
: "${ROS_WS:=/root/ros2_ws}"
: "${LAUNCH_PKG:=unitree_ros2_interface}"

# Se vuoi bypassare la detection:
: "${BOARD_ROLE:=}"   # head | body | main
: "${BOARD_IP:=}"     # es: 192.168.123.14

# Feature flags (0/1 oppure true/false)
: "${ENABLE_CAMERA:=0}"
: "${ENABLE_ULTRASOUND:=0}"
: "${ENABLE_FACE_LIGHTS:=0}"
: "${ENABLE_LOW:=1}"
: "${ENABLE_HIGH:=0}"

# Camera options (sempre come parametri del launchfile)
: "${PUBLISH_RECTIFIED:=false}"
: "${PUBLISH_DEPTH:=false}"
: "${PUBLISH_PCL:=false}"

# Nomi launch file (override per gestire eventuali typo nei file reali)
: "${LAUNCH_HEAD:=head_board.launch.py}"
: "${LAUNCH_BODY:=body_board.launch.py}"
: "${LAUNCH_MAIN:=main_board.launch.py}"
: "${LAUNCH_PI:=pi_board.launch.py}"
: "${LAUNCH_INTERFACE:=interface.launch.py}"

# ----------------------------
# Helpers
# ----------------------------
to_bool() {
  # Converte 0/1, yes/no, true/false in "true"/"false"
  case "${1,,}" in
    1|true|yes|y|on)  echo "true" ;;
    0|false|no|n|off) echo "false" ;;
    *)                echo "${1}" ;;
  esac
}

detect_local_ip() {
  # 1) preferisci iproute2; 2) fallback hostname -I
  # Se hai più IP, prova a prendere quello nella subnet 192.168.123.*
  local ips ip
  ips="$(ip -4 -o addr show scope global 2>/dev/null | awk '{print $4}' | cut -d/ -f1 || true)"
  if [[ -n "${ips}" ]]; then
    ip="$(echo "${ips}" | grep -E '^192\.168\.123\.' | head -n1 || true)"
    [[ -n "${ip}" ]] && { echo "${ip}"; return 0; }
    echo "${ips}" | head -n1
    return 0
  fi
  hostname -I 2>/dev/null | awk '{print $1}' || true
}

# ----------------------------
# Source ROS env
# ----------------------------
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f "${ROS_WS}/install/setup.bash" ]]; then
  source "${ROS_WS}/install/setup.bash"
fi

# ----------------------------
# Resolve board IP and role
# ----------------------------
if [[ -n "${BOARD_IP}" ]]; then
  local_ip="${BOARD_IP}"
else
  local_ip="$(detect_local_ip)"
fi

if [[ -z "${local_ip}" ]]; then
  echo >&2 "[entrypoint] ERROR: Unable to determine board IP. Use --network host or set BOARD_IP."
  exit 2
fi

role="${BOARD_ROLE}"
if [[ -z "${role}" ]]; then
  case "${local_ip}" in
    192.168.123.13)  role="head" ;;
    192.168.123.14)  role="body" ;;
    192.168.123.15)  role="main" ;;
    192.168.123.161) role="pi"  ;;
    *) echo >&2 "[entrypoint] ERROR: No role mapping for IP ${local_ip}. Set BOARD_ROLE=head|body|main."; role="interface" ;;
  esac
fi

case "${role}" in
  head) launch_file="${LAUNCH_HEAD}" ;;
  body) launch_file="${LAUNCH_BODY}" ;;
  main) launch_file="${LAUNCH_MAIN}" ;;
  pi)   launch_file="${LAUNCH_PI}" ;;
  interface) launch_file="${LAUNCH_INTERFACE}" ;;
  *) echo >&2 "[entrypoint] ERROR: Invalid BOARD_ROLE='${role}' (expected head|body|main|pi|interface)."; launch_file="${LAUNCH_INTERFACE}" ;;
esac

echo "[entrypoint] ROS_DISTRO=${ROS_DISTRO}"
echo "[entrypoint] ROS_WS=${ROS_WS}"
echo "[entrypoint] LAUNCH_PKG=${LAUNCH_PKG}"
echo "[entrypoint] local_ip=${local_ip}"
echo "[entrypoint] role=${role}"
echo "[entrypoint] launch_file=${launch_file}"

# ----------------------------
# Launch arguments (passati al launchfile)
# ros2 launch accetta key:=value per gli argomenti dichiarati nel launch file :contentReference[oaicite:2]{index=2}
# ----------------------------
args=(
  "board_ip:=${local_ip}"
  "board_role:=${role}"

  "enable_camera:=$(to_bool "${ENABLE_CAMERA}")"
  "enable_ultrasound:=$(to_bool "${ENABLE_ULTRASOUND}")"
  "enable_face_lights:=$(to_bool "${ENABLE_FACE_LIGHTS}")"
  "enable_low:=$(to_bool "${ENABLE_LOW}")"
  "enable_high:=$(to_bool "${ENABLE_HIGH}")"

  "publish_rectified:=$(to_bool "${PUBLISH_RECTIFIED}")"
  "publish_depth:=$(to_bool "${PUBLISH_DEPTH}")"
  "publish_pcl:=$(to_bool "${PUBLISH_PCL}")"
)

# Importante: exec -> ros2 launch diventa PID 1, segnali e shutdown puliti
exec ros2 launch "${LAUNCH_PKG}" "${launch_file}" "${args[@]}"
