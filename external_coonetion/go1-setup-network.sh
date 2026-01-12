#!/usr/bin/env bash
# go1-setup-comm.sh
# IMPORTANT: must be sourced:  source ./go1-setup-comm.sh

set -u

# -----------------------
# Config (override via env)
# -----------------------
GO1_DOMAIN_ID="${GO1_DOMAIN_ID:-43}"

# ETH static on robot LAN
GO1_PC_ETH_IP="${GO1_PC_ETH_IP:-192.168.123.162}"
GO1_ROBOT_CIDR="${GO1_ROBOT_CIDR:-192.168.123.0/24}"
GO1_PI_ROBOT_IP="${GO1_PI_ROBOT_IP:-192.168.123.161}"

# Wi-Fi AP subnet
GO1_AP_SUBNET_PREFIX="${GO1_AP_SUBNET_PREFIX:-192.168.12.}"
GO1_AP_GW="${GO1_AP_GW:-192.168.12.1}"

# DDS profiles
GO1_DDS_DIR="${GO1_DDS_DIR:-$(pwd)}"
DDS_ETH="${DDS_ETH:-$GO1_DDS_DIR/cyclonedds_pc_eth.xml}"
DDS_WLAN="${DDS_WLAN:-$GO1_DDS_DIR/cyclonedds_pc_wlan.xml}"
DDS_GENERIC="${DDS_GENERIC:-$GO1_DDS_DIR/cyclonedds_pc_generic.xml}"

# Prefer ETH when both are present
METRIC_ETH="${METRIC_ETH:-50}"
METRIC_WLAN="${METRIC_WLAN:-200}"

# Loggin
GO1_LOG_LEVEL="${GO1_LOG_LEVEL:-INFO}"   # DEBUG|INFO|WARN|ERROR
GO1_LOG_FILE="${GO1_LOG_FILE:-}"         # e.g. ~/.cache/go1/go1-setup.log (empty disables)

# -----------------------
# Logging helpers
# -----------------------
_ts() { date +"%Y-%m-%d %H:%M:%S"; }

_lvl_to_n() {
  case "${1}" in
    DEBUG) echo 10 ;;
    INFO)  echo 20 ;;
    WARN)  echo 30 ;;
    ERROR) echo 40 ;;
    *)     echo 20 ;;
  esac
}

_log() {
  local lvl="$1"; shift
  local msg="$*"
  local cur_n req_n
  cur_n="$(_lvl_to_n "${GO1_LOG_LEVEL}")"
  req_n="$(_lvl_to_n "${lvl}")"
  [[ "${req_n}" -lt "${cur_n}" ]] && return 0

  local line="[$(_ts)] [go1_pc] [${lvl}] ${msg}"
  echo "${line}"
  if [[ -n "${GO1_LOG_FILE}" ]]; then
    mkdir -p "$(dirname "${GO1_LOG_FILE}")" 2>/dev/null || true
    echo "${line}" >> "${GO1_LOG_FILE}" 2>/dev/null || true
  fi
}

die() { _log ERROR "$*"; return 2 2>/dev/null || exit 2; }

# -----------------------
# Guard: executed vs sourced
# -----------------------
# If executed, exports won't persist. Detect and warn.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: This script must be sourced, not executed."
  echo "Use: source ${0}"
  exit 2
fi

# -----------------------
# Network detection
# -----------------------
_find_if_by_exact_ip() {
  local ip="$1"
  ip -4 -o addr show scope global 2>/dev/null \
    | awk -v ip="${ip}" '$4 ~ ("^"ip"/") {print $2; exit}'
}

_find_if_by_prefix() {
  local prefix="$1"
  ip -4 -o addr show scope global 2>/dev/null \
    | awk -v p="^"prefix '$4 ~ p {print $2; exit}'
}

_has_route_to() {
  local cidr="$1"
  ip route show "${cidr}" >/dev/null 2>&1
}

_route_replace() {
  # runtime-only, idempotent
  sudo ip route replace "$@" >/dev/null 2>&1
}

_ping_quick() {
  local host="$1"
  ping -c 1 -W 1 "${host}" >/dev/null 2>&1
}

ETH_IF="$(_find_if_by_exact_ip "${GO1_PC_ETH_IP}")"
WLAN_IF="$(_find_if_by_prefix "${GO1_AP_SUBNET_PREFIX}")"

MODE="generic"
DDS_FILE="${DDS_GENERIC}"
if [[ -n "${ETH_IF}" ]]; then
  MODE="eth"
  DDS_FILE="${DDS_ETH}"
elif [[ -n "${WLAN_IF}" ]]; then
  MODE="wlan"
  DDS_FILE="${DDS_WLAN}"
fi

_log INFO "Detected: MODE=${MODE}, ETH_IF=${ETH_IF:-none}, WLAN_IF=${WLAN_IF:-none}"

# -----------------------
# Validate DDS profile exists
# -----------------------
[[ -f "${DDS_FILE}" ]] || die "DDS profile not found: ${DDS_FILE} (set GO1_DDS_DIR or DDS_* vars)"

# Optional: basic XML sanity check if xmllint exists (non-fatal)
if command -v xmllint >/dev/null 2>&1; then
  if ! xmllint --noout "${DDS_FILE}" >/dev/null 2>&1; then
    die "DDS profile XML is not well-formed: ${DDS_FILE}"
  fi
fi

# -----------------------
# Runtime routing checks/fixes (PC is NOT a router)
# -----------------------
# Goal: ensure PC can reach 192.168.123.0/24 from whichever access is active.
#
# - If on WLAN(AP): ensure route to robot LAN via 192.168.12.1
# - If on ETH: ensure direct route via ETH (preferred metric)
# - If both: keep both routes but prefer ETH using metric

if [[ -n "${WLAN_IF}" ]]; then
  _log INFO "Ensuring WLAN path to robot LAN: ${GO1_ROBOT_CIDR} via ${GO1_AP_GW} dev ${WLAN_IF} metric ${METRIC_WLAN}"
  _route_replace "${GO1_ROBOT_CIDR}" via "${GO1_AP_GW}" dev "${WLAN_IF}" metric "${METRIC_WLAN}" || \
    die "Failed to set WLAN route (need sudo)."
fi

if [[ -n "${ETH_IF}" ]]; then
  _log INFO "Ensuring preferred ETH direct route to robot LAN: ${GO1_ROBOT_CIDR} dev ${ETH_IF} metric ${METRIC_ETH}"
  _route_replace "${GO1_ROBOT_CIDR}" dev "${ETH_IF}" metric "${METRIC_ETH}" || \
    die "Failed to set ETH route (need sudo)."
fi

# Reachability hints (non-fatal)
if _ping_quick "${GO1_PI_ROBOT_IP}"; then
  _log INFO "Ping OK: ${GO1_PI_ROBOT_IP} reachable."
else
  _log WARN "Ping FAIL: ${GO1_PI_ROBOT_IP} not reachable (check cable/AP, routes, Pi)."
fi

# -----------------------
# Export ROS 2 / CycloneDDS env
# -----------------------
export ROS_DOMAIN_ID="${GO1_DOMAIN_ID}"
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"      # ROS 2 RMW selection
export CYCLONEDDS_URI="file://${DDS_FILE}"         # CycloneDDS config

# Avoid stale daemon cache when switching config
ros2 daemon stop >/dev/null 2>&1 || true

_log INFO "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
_log INFO "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
_log INFO "CYCLONEDDS_URI=${CYCLONEDDS_URI}"

# Provide quick status
_log INFO "Route snapshot: $(ip route show "${GO1_ROBOT_CIDR}" 2>/dev/null | tr '\n' ' ')" || true

