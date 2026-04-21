#!/usr/bin/env bash
set -Eeuo pipefail

# ----------------------------
# Defaults (configurable via env)
# ----------------------------
: "${ROS_DISTRO:=humble}"

: "${DEBUG_MODE:=0}"          # 0 = launch, 1 = debug (sleep infinity)
: "${AUTOSTART:=1}"           # 1 = start on boot, 0 = wait
: "${SERIAL_PORT:=/dev/ttyUSB0}"
: "${NAMESPACE:=unitree_go1}"
: "${SCANNER_PATH:=/dev/serial/by-id/usb-Hokuyo_Data_Flex_for_USB_URG-Series_USB_Driver-if00}"

# Donde si trova lo script DDS (copiato nell'immagine)
: "${DDS_ENV_SCRIPT:=/usr/local/bin/unitree_dds_env.sh}"

# ----------------------------
# Source ROS env
# ----------------------------
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

# ----------------------------
# DDS env (role/ip/profile selection)
# ----------------------------
if [[ -f "${DDS_ENV_SCRIPT}" ]]; then
  source "${DDS_ENV_SCRIPT}"
else
  echo >&2 "[laser_entrypoint] WARN: DDS env script not found: ${DDS_ENV_SCRIPT}"
fi

# Persist env for interactive shells (docker exec)
if ! grep -q 'unitree_env.sh' /root/.bashrc 2>/dev/null; then
  echo '[[ -f /run/unitree_env.sh ]] && source /run/unitree_env.sh' >> /root/.bashrc
fi

# ----------------------------
# Launch
# ----------------------------
if [[ "${DEBUG_MODE}" == "1" ]]; then
  echo "[laser_entrypoint] DEBUG_MODE=1 -> container alive, no auto-launch"
  sleep infinity
fi

if [[ "${AUTOSTART}" != "1" ]]; then
  echo "[laser_entrypoint] AUTOSTART=0 -> container alive, waiting for manual start"
  sleep infinity
fi

echo "[laser_entrypoint] Launching Hokuyo laser scanner..."
echo "[laser_entrypoint] SCANNER_PATH=${SCANNER_PATH}"

# Launch directly instead of run_hokuyo.sh (which hardcodes jazzy setup.bash)
exec ros2 launch ~/hokuyo_bringup/hokuyo_urg.launch.py