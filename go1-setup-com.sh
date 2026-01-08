#!/usr/bin/env bash
# Laptop-only Go1 ROS 2 environment selector
# Usage:
#   source ~/go1_env.sh
#
# Optional overrides:
#   GO1_DOMAIN_ID=43 GO1_AP_GW=192.168.12.1 GO1_ROBOT_CIDR=192.168.123.0/24 \
#   GO1_DDS_DIR=$HOME/cyclonedds source ~/go1_env.sh

set -euo pipefail

# ---- Defaults ----
GO1_DOMAIN_ID="${GO1_DOMAIN_ID:-43}"
GO1_AP_GW="${GO1_AP_GW:-192.168.12.1}"
GO1_ROBOT_CIDR="${GO1_ROBOT_CIDR:-192.168.123.0/24}"
GO1_DDS_DIR="${GO1_DDS_DIR:-Docker/cyclonedds}"

DDS_ETH="${DDS_ETH:-$GO1_DDS_DIR/cyclonedds_pc_eth.xml}"
DDS_WLAN="${DDS_WLAN:-$GO1_DDS_DIR/cyclonedds_pc_wlan.xml}"
DDS_GENERIC="${DDS_GENERIC:-$GO1_DDS_DIR/cyclonedds_generic.xml}"

# ---- Detect network mode ----
IPS="$(ip -4 -o addr show scope global | awk '{print $4}' | cut -d/ -f1 || true)"

HAS_ETH=false
HAS_WLAN=false
echo "${IPS}" | grep -qE '^192\.168\.123\.' && HAS_ETH=true || true
echo "${IPS}" | grep -qE '^192\.168\.12\.'  && HAS_WLAN=true || true

# Prefer ETH if both are present
MODE="generic"
DDS_FILE="${DDS_GENERIC}"

if $HAS_ETH; then
  MODE="eth"
  DDS_FILE="${DDS_ETH}"
elif $HAS_WLAN; then
  MODE="wlan"
  DDS_FILE="${DDS_WLAN}"
fi

if [[ ! -f "${DDS_FILE}" ]]; then
  echo >&2 "[go1_env] ERROR: DDS profile not found: ${DDS_FILE}"
  echo >&2 "[go1_env] Set GO1_DDS_DIR or DDS_ETH/DDS_WLAN/DDS_GENERIC."
  return 2 2>/dev/null || exit 2
fi

# ---- Export ROS 2 env ----
export ROS_DOMAIN_ID="${GO1_DOMAIN_ID}"                
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"        
export CYCLONEDDS_URI="file://${DDS_FILE}"             

# ---- Optional: ensure route to robot LAN when on WLAN ----
if [[ "${MODE}" == "wlan" ]]; then
  if ! ip route show "${GO1_ROBOT_CIDR}" >/dev/null 2>&1; then
    echo "[go1_env] Adding route: ${GO1_ROBOT_CIDR} via ${GO1_AP_GW}"
    sudo ip route add "${GO1_ROBOT_CIDR}" via "${GO1_AP_GW}"    
  fi
fi

echo "[go1_env] MODE=${MODE}"
echo "[go1_env] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[go1_env] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "[go1_env] CYCLONEDDS_URI=${CYCLONEDDS_URI}"
