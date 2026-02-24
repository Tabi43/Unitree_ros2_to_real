#!/usr/bin/env bash

#####################
##                 ##
##  Quadruped Go1  ##
##                 ##     
#####################

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Default values (override with env or CLI flags)
CONTAINER_NAME="${CONTAINER_NAME:-zenoh_ros2dds}"
IMAGE="${IMAGE:-eclipse/zenoh-bridge-ros2dds:latest}"
DOMAIN_ID="${DOMAIN_ID:-43}"

# This must already exist on disk (your “defined path”)
CONFIG_FILE="${CONFIG_FILE:-${SCRIPT_DIR}/Docker/zenoh/ros2dds_robot.json5}"

# Optional CycloneDDS XML (mount only if exists)
CYCLONE_XML="${CYCLONE_XML:-${SCRIPT_DIR}/Docker/cyclonedds/cyclonedds_pi.xml}"

# Optional REST admin API port (empty = disabled)
REST_HTTP_PORT="${REST_HTTP_PORT:-}"

usage() {
  cat <<EOF
Usage:
  $0 install --config /abs/path/ros2dds.json5 [--domain ID] [--name NAME] [--image IMG:TAG]
             [--rest-port 8000] [--cyclone-xml /abs/path/cyclonedds.xml]
  $0 start|stop|restart|status|logs|remove|update

Notes:
- Uses --net=host because the bridge discovers ROS 2 entities via DDS/UDP multicast on the Domain ID.
- Bridge default listen endpoint in router mode is tcp/0.0.0.0:7447 (can be overridden in config).
EOF
}

need_cmd() { command -v "$1" >/dev/null 2>&1 || { echo "ERROR: missing: $1"; exit 1; }; }

container_exists() {
  docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"
}

do_install() {
  need_cmd docker

  if [[ ! -f "${CONFIG_FILE}" ]]; then
    echo "ERROR: config file not found: ${CONFIG_FILE}"
    echo "Provide it with --config /abs/path/file.json5 or set CONFIG_FILE env var."
    exit 2
  fi

  echo "Pulling image: ${IMAGE}"
  sudo docker pull "${IMAGE}"

  if container_exists; then
    echo "Removing existing container: ${CONTAINER_NAME}"
    sudo docker rm -f "${CONTAINER_NAME}" >/dev/null
  fi

  run_args=(
    -d
    --name "${CONTAINER_NAME}"
    --net=host
    --restart unless-stopped
    -e "ROS_DISTRO=humble"
    -e "ROS_DOMAIN_ID=${DOMAIN_ID}"
    -v "${CONFIG_FILE}:/config.json5:ro"
  )

  # Optional CycloneDDS XML configuration for the bridge's DDS side
  if [[ -f "${CYCLONE_XML}" ]]; then
    run_args+=(
      -e "CYCLONEDDS_URI=file:///etc/cyclonedds.xml"
      -v "${CYCLONE_XML}:/etc/cyclonedds.xml:ro"
    )
    echo "Using CycloneDDS XML: ${CYCLONE_XML}"
  fi

  cmd=( -c /config.json5 )
  if [[ -n "${REST_HTTP_PORT}" ]]; then
    cmd+=( --rest-http-port "${REST_HTTP_PORT}" )
    echo "REST admin enabled on port ${REST_HTTP_PORT}"
  fi

  echo "Starting container ${CONTAINER_NAME}..."
  sudo docker run "${run_args[@]}" "${IMAGE}" "${cmd[@]}"

  sudo docker ps --filter "name=${CONTAINER_NAME}"
}

do_start()   { sudo docker start "${CONTAINER_NAME}"; }
do_stop()    { sudo docker stop  "${CONTAINER_NAME}"; }
do_restart() { sudo docker restart "${CONTAINER_NAME}"; }
do_status()  { sudo docker ps -a --filter "name=${CONTAINER_NAME}"; }
do_logs()    { sudo docker logs -f "${CONTAINER_NAME}"; }
do_remove()  { sudo docker rm -f "${CONTAINER_NAME}"; }

do_update() {
  need_cmd docker
  echo "Updating image ${IMAGE} and recreating container..."
  sudo docker pull "${IMAGE}"
  do_install
}

if [[ $# -lt 1 ]]; then usage; exit 1; fi
action="$1"; shift

if [[ "${action}" == "install" ]]; then
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --config) CONFIG_FILE="$2"; shift 2;;
      --domain) DOMAIN_ID="$2"; shift 2;;
      --name) CONTAINER_NAME="$2"; shift 2;;
      --image) IMAGE="$2"; shift 2;;
      --rest-port) REST_HTTP_PORT="$2"; shift 2;;
      --cyclone-xml) CYCLONE_XML="$2"; shift 2;;
      -h|--help) usage; exit 0;;
      *) echo "Unknown option: $1"; usage; exit 1;;
    esac
  done
  do_install
  exit 0
fi

case "${action}" in
  start) do_start;;
  stop) do_stop;;
  restart) do_restart;;
  status) do_status;;
  logs) do_logs;;
  remove) do_remove;;
  update) do_update;;
  -h|--help) usage;;
  *) echo "Unknown command: ${action}"; usage; exit 1;;
esac