#!/usr/bin/env bash
# go1-setup-comm.sh (v3)
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

# Wi-Fi AP gateway (robot AP)
GO1_AP_GW="${GO1_AP_GW:-192.168.12.1}"

# DDS profiles directory (default = script directory)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GO1_DDS_DIR="${GO1_DDS_DIR:-${SCRIPT_DIR}}"

DDS_ETH="${DDS_ETH:-$GO1_DDS_DIR/cyclonedds_pc_eth.xml}"
DDS_WLAN="${DDS_WLAN:-$GO1_DDS_DIR/cyclonedds_pc_wlan.xml}"
DDS_GENERIC="${DDS_GENERIC:-$GO1_DDS_DIR/cyclonedds_pc_generic.xml}"

# Prefer ETH when both are present (lower metric = higher priority)
METRIC_ETH="${METRIC_ETH:-50}"
METRIC_WLAN="${METRIC_WLAN:-200}"

# Logging
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
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: This script must be sourced, not executed."
  echo "Use: source ${0}"
  exit 2
fi

# -----------------------
# Helpers (network)
# -----------------------
_find_if_by_exact_ip() {
  local ip="$1"
  ip -4 -o addr show scope global 2>/dev/null \
    | awk -v ip="${ip}" '$4 ~ ("^"ip"/") {print $2; exit}'
}

_route_get() {
  local dst="$1"
  ip -4 route get "${dst}" 2>/dev/null | head -n1 || true
}

_route_get_field() {
  local line="$1" key="$2"
  awk -v k="${key}" '{for(i=1;i<=NF;i++) if($i==k){print $(i+1); exit}}' <<< "${line}"
}

_ping_quick() {
  local host="$1"
  ping -c 1 -W 1 "${host}" >/dev/null 2>&1
}

_route_replace() {
  sudo ip route replace "$@" >/dev/null 2>&1
}

_eth_link_up() {
  # True if link looks physically up. Uses carrier if present; fallback to operstate.
  local ifc="$1"
  [[ -z "${ifc}" ]] && return 1

  local carrier_file="/sys/class/net/${ifc}/carrier"
  local oper_file="/sys/class/net/${ifc}/operstate"

  if [[ -r "${carrier_file}" ]]; then
    [[ "$(cat "${carrier_file}" 2>/dev/null || echo 0)" == "1" ]] && return 0
    return 1
  fi

  if [[ -r "${oper_file}" ]]; then
    [[ "$(cat "${oper_file}" 2>/dev/null || echo down)" == "up" ]] && return 0
    return 1
  fi

  # If sysfs info missing, fall back to "ip link show"
  ip link show "${ifc}" 2>/dev/null | grep -q "state UP"
}

# -----------------------
# Detection
# -----------------------
ETH_IF="$(_find_if_by_exact_ip "${GO1_PC_ETH_IP}")"

RT_AP="$(_route_get "${GO1_AP_GW}")"
AP_IF=""
AP_SRC=""
if [[ -n "${RT_AP}" ]]; then
  AP_IF="$(_route_get_field "${RT_AP}" dev)"
  AP_SRC="$(_route_get_field "${RT_AP}" src)"
fi

_log DEBUG "route_get(${GO1_AP_GW})='${RT_AP}'"
_log DEBUG "Parsed AP dev='${AP_IF:-none}' src='${AP_SRC:-none}'"
_log DEBUG "ETH_IF(by exact ip ${GO1_PC_ETH_IP})='${ETH_IF:-none}'"

# Availability checks
ETH_READY=false
WLAN_READY=false

if [[ -n "${ETH_IF}" ]] && _eth_link_up "${ETH_IF}"; then
  ETH_READY=true
fi

# WLAN(AP) considered ready if gateway reachable AND routing to it uses AP dev/src
# (we also accept just ping in case route output is minimal)
if _ping_quick "${GO1_AP_GW}"; then
  if [[ -n "${AP_IF}" ]] && [[ "${AP_SRC}" =~ ^192\.168\.12\. ]]; then
    WLAN_READY=true
  else
    # still ok: gateway responds, so we can use whatever dev kernel chose
    WLAN_READY=true
  fi
fi

_log INFO "Availability: ETH_READY=${ETH_READY} (ETH_IF=${ETH_IF:-none}), WLAN_READY=${WLAN_READY} (AP_IF=${AP_IF:-none}, AP_SRC=${AP_SRC:-none})"

# -----------------------
# Mode selection (3 cases)
# -----------------------
# 1) both -> prefer eth
# 2) eth only
# 3) wlan only
MODE="generic"
DDS_FILE="${DDS_GENERIC}"

if [[ "${ETH_READY}" == "true" ]] && [[ "${WLAN_READY}" == "true" ]]; then
  MODE="eth"
  DDS_FILE="${DDS_ETH}"
elif [[ "${ETH_READY}" == "true" ]]; then
  MODE="eth"
  DDS_FILE="${DDS_ETH}"
elif [[ "${WLAN_READY}" == "true" ]]; then
  MODE="wlan"
  DDS_FILE="${DDS_WLAN}"
fi

_log INFO "Selected MODE=${MODE} (DDS_FILE=$(basename "${DDS_FILE}"))"

# -----------------------
# Validate DDS profile exists
# -----------------------
if [[ ! -f "${DDS_FILE}" ]]; then
  _log ERROR "DDS profile not found: ${DDS_FILE}"
  _log ERROR "Hints: set GO1_DDS_DIR to folder containing XMLs."
  _log ERROR "Current GO1_DDS_DIR='${GO1_DDS_DIR}' (script dir='${SCRIPT_DIR}')"
  return 3 2>/dev/null || exit 3
fi

if command -v xmllint >/dev/null 2>&1; then
  if ! xmllint --noout "${DDS_FILE}" >/dev/null 2>&1; then
    die "DDS profile XML is not well-formed: ${DDS_FILE}"
  fi
fi

# -----------------------
# Runtime routing (PC is NOT a router)
# -----------------------
# Install both routes if both are available; metrics ensure ETH is preferred.
if [[ "${WLAN_READY}" == "true" ]]; then
  # Ensure we know which dev to use for AP; if empty, re-derive now.
  if [[ -z "${AP_IF}" ]]; then
    RT_AP="$(_route_get "${GO1_AP_GW}")"
    AP_IF="$(_route_get_field "${RT_AP}" dev)"
  fi
  [[ -n "${AP_IF}" ]] || die "WLAN ready but could not parse AP interface. Try: ip route get ${GO1_AP_GW}"

  _log INFO "Ensuring WLAN route to robot LAN: ${GO1_ROBOT_CIDR} via ${GO1_AP_GW} dev ${AP_IF} metric ${METRIC_WLAN}"
  _route_replace "${GO1_ROBOT_CIDR}" via "${GO1_AP_GW}" dev "${AP_IF}" metric "${METRIC_WLAN}" \
    || die "Failed to set WLAN route (need sudo)."
fi

if [[ "${ETH_READY}" == "true" ]]; then
  _log INFO "Ensuring preferred ETH route to robot LAN: ${GO1_ROBOT_CIDR} dev ${ETH_IF} metric ${METRIC_ETH}"
  _route_replace "${GO1_ROBOT_CIDR}" dev "${ETH_IF}" metric "${METRIC_ETH}" \
    || die "Failed to set ETH route (need sudo)."
fi

# Debug routing decision to Pi
RT_PI="$(_route_get "${GO1_PI_ROBOT_IP}")"
_log INFO "route_get(${GO1_PI_ROBOT_IP})='${RT_PI}'"

# Reachability hint
if _ping_quick "${GO1_PI_ROBOT_IP}"; then
  _log INFO "Ping OK: ${GO1_PI_ROBOT_IP} reachable."
else
  _log WARN "Ping FAIL: ${GO1_PI_ROBOT_IP} not reachable (check AP route/forwarding, cable, Pi)."
fi

# -----------------------
# Export ROS 2 / CycloneDDS env
# -----------------------
export ROS_DOMAIN_ID="${GO1_DOMAIN_ID}"
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"     
export CYCLONEDDS_URI="file://${DDS_FILE}"         

ros2 daemon stop >/dev/null 2>&1 || true

_log INFO "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
_log INFO "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
_log INFO "CYCLONEDDS_URI=${CYCLONEDDS_URI}"
_log INFO "Route snapshot: $(ip route show "${GO1_ROBOT_CIDR}" 2>/dev/null | tr '\n' ' ')" || true

