#!/usr/bin/env bash
set -Eeuo pipefail

# ----------------------------
# Defaults configurabili via env (DDS)
# ----------------------------
: "${BOARD_ROLE:=}"          # head | body | main | pi | interface
: "${BOARD_IP:=}"            # override ip
: "${ROS_DOMAIN_ID:=43}"

: "${RMW_IMPLEMENTATION:=rmw_cyclonedds_cpp}"
: "${DDS_PROFILE_DIR:=/opt/cyclonedds}"
: "${DDS_PROFILE:=}"         # override esplicito, es: cyclonedds_14.xml
: "${DDS_TARGET_XML:=/etc/cyclonedds/cyclonedds.xml}"

# ----------------------------
# Helpers
# ----------------------------
detect_local_ip() {
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

resolve_role() {
  local ip="$1"
  if [[ -n "${BOARD_ROLE}" ]]; then
    echo "${BOARD_ROLE}"
    return 0
  fi
  case "${ip}" in
    192.168.123.13)  echo "head" ;;
    192.168.123.14)  echo "body" ;;
    192.168.123.15)  echo "main" ;;
    192.168.123.161) echo "pi" ;;
    *)               echo "interface" ;;
  esac
}

select_dds_profile() {
  local role="$1" ip="$2"
  if [[ -n "${DDS_PROFILE}" ]]; then
    echo "${DDS_PROFILE_DIR}/${DDS_PROFILE}"
    return 0
  fi
  case "${role}" in
    head) echo "${DDS_PROFILE_DIR}/cyclonedds_13.xml" ;;
    body) echo "${DDS_PROFILE_DIR}/cyclonedds_14.xml" ;;
    main) echo "${DDS_PROFILE_DIR}/cyclonedds_15.xml" ;;
    pi)   echo "${DDS_PROFILE_DIR}/cyclonedds_pi.xml" ;;
    interface)
      # se vuoi distinguere wlan/eth qui è il punto giusto
      echo "${DDS_PROFILE_DIR}/cyclonedds_generic.xml"
      ;;
    *)    echo "${DDS_PROFILE_DIR}/cyclonedds_generic.xml" ;;
  esac
}

# ----------------------------
# Resolve board ip + role
# ----------------------------
if [[ -n "${BOARD_IP}" ]]; then
  local_ip="${BOARD_IP}"
else
  local_ip="$(detect_local_ip)"
fi

if [[ -z "${local_ip}" ]]; then
  echo >&2 "[dds_env] ERROR: Unable to determine board IP. Use --network host or set BOARD_IP."
  exit 2
fi

role="$(resolve_role "${local_ip}")"

# ----------------------------
# CycloneDDS selection
# ----------------------------
mkdir -p "$(dirname "${DDS_TARGET_XML}")"

# Se CYCLONEDDS_URI è già settato dall’esterno, rispettalo
if [[ -z "${CYCLONEDDS_URI:-}" ]]; then
  dds_src="$(select_dds_profile "${role}" "${local_ip}")"

  if [[ ! -f "${dds_src}" ]]; then
    echo >&2 "[dds_env] ERROR: CycloneDDS profile not found: ${dds_src}"
    echo >&2 "[dds_env] Tip: mount a profile dir or set DDS_PROFILE / CYCLONEDDS_URI."
    exit 3
  fi

  cp -f "${dds_src}" "${DDS_TARGET_XML}"
  export CYCLONEDDS_URI="file://${DDS_TARGET_XML}"   # formato consigliato: file:///abs/path 
fi

export RMW_IMPLEMENTATION ROS_DOMAIN_ID
export BOARD_IP="${local_ip}"
export BOARD_ROLE="${role}"

# ----------------------------
# Persist per le shell (docker exec -> bash legge ~/.bashrc) 
# ----------------------------
cat >/run/unitree_env.sh <<EOF
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-}"
export BOARD_IP="${BOARD_IP}"
export BOARD_ROLE="${BOARD_ROLE}"
EOF
chmod 0644 /run/unitree_env.sh

echo "[dds_env] BOARD_IP=${BOARD_IP}"
echo "[dds_env] BOARD_ROLE=${BOARD_ROLE}"
echo "[dds_env] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[dds_env] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "[dds_env] CYCLONEDDS_URI=${CYCLONEDDS_URI:-<unset>}"
