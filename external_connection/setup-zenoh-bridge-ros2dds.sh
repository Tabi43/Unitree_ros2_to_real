#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-zenoh_ros2dds_pc}"
IMAGE="${IMAGE:-eclipse/zenoh-bridge-ros2dds:latest}"

# file già esistente sul PC
CONFIG_FILE="${CONFIG_FILE:-ros2dds_pc.json5}"

# Domain locale del bridge (coerente con json5)
DOMAIN_ID="${DOMAIN_ID:-44}"

# opzionale: CycloneDDS XML se vuoi controllare NIC lato DDS del bridge
CYCLONE_XML="${CYCLONE_XML:-}"

usage() {
  cat <<EOF
Usage:
  $0 install [--config /abs/path.json5] [--domain ID] [--name NAME] [--image IMG:TAG] [--cyclone-xml /abs/cyclonedds.xml]
  $0 start|stop|restart|status|logs|remove|update
EOF
}

need_cmd() { command -v "$1" >/dev/null 2>&1 || { echo "ERROR: missing: $1"; exit 1; }; }

container_exists() { docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; }

do_install() {
  need_cmd docker

  if [[ ! -f "${CONFIG_FILE}" ]]; then
    echo "ERROR: config file not found: ${CONFIG_FILE}"
    exit 2
  fi

  sudo docker pull "${IMAGE}"

  if container_exists; then
    sudo docker rm -f "${CONTAINER_NAME}" >/dev/null
  fi

  run_args=(
    -d
    --name "${CONTAINER_NAME}"
    --net=host
    --restart unless-stopped
    -e "ROS_DOMAIN_ID=${DOMAIN_ID}"
    -v "${CONFIG_FILE}:/config.json5:ro"
  )

  if [[ -n "${CYCLONE_XML}" ]]; then
    if [[ ! -f "${CYCLONE_XML}" ]]; then
      echo "ERROR: cyclone xml not found: ${CYCLONE_XML}"
      exit 3
    fi
    run_args+=(
      -e "CYCLONEDDS_URI=file:///etc/cyclonedds.xml"
      -v "${CYCLONE_XML}:/etc/cyclonedds.xml:ro"
    )
  fi

  sudo docker run "${run_args[@]}" "${IMAGE}" -c /config.json5
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
