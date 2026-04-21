#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ------------------------
# Defaults (env overridable)
# ------------------------
CONTAINER_NAME="${CONTAINER_NAME:-unitree_ros2_laser_scan}"

# Image selection
# - Option A: export IMAGE="repo:tag"
# - Option B: export IMAGE_REPO="repo" and IMAGE_TAG="tag"
IMAGE="${IMAGE:-}"
IMAGE_REPO="${IMAGE_REPO:-tabi43/unitree_ros2}"
IMAGE_TAG="${IMAGE_TAG:-hokuyo_laser}"

# Build strategy
FORCE_LOCAL_BUILD="${FORCE_LOCAL_BUILD:-0}"   # 1 = build locally, 0 = pull
PLATFORM="${PLATFORM:-}"                      # empty = auto-detect

# Runtime policy / privileges
RESTART_POLICY="${RESTART_POLICY:-unless-stopped}"  # no|unless-stopped|always|on-failure:N
PRIVILEGED="${PRIVILEGED:-1}"

# Laser scan options
AUTOSTART="${AUTOSTART:-1}"        # 1 = autostart, 0 = be ready to start manually
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyUSB0}"
SCANNER_PATH="${SCANNER_PATH:-/dev/serial/by-id/usb-Hokuyo_Data_Flex_for_USB_URG-Series_USB_Driver-if00}"
NAMESPACE="${NAMESPACE:-unitree_go1}"

# Optional role override (if empty the container can auto-detect)
BOARD_ROLE="${BOARD_ROLE:-}"

# Debug
DEBUG_MODE="${DEBUG_MODE:-0}"  # 0 = normal (launch), 1 = debug (no launch)

usage() {
  cat <<EOFU
Usage:
  $0 install [--name NAME] [--image REPO:TAG | --repo REPO --tag TAG] [--platform linux/amd64|linux/arm64]
             [--local-build] [--restart-policy POLICY] [--privileged|--no-privileged]
             [--serial-port PORT] [--namespace NS]
             [--board-role ROLE] [--debug 0|1]
  $0 start|stop|restart|status|logs|remove|update

Notes:
- Uses --net=host and binds /dev into the container for device access (laser serial port).
- For "update" it pulls/builds the image and recreates the container with the current env/flags.
- If you want to change NAME/IMAGE for start/stop/etc, set env vars (CONTAINER_NAME, IMAGE, ...).
EOFU
}

need_cmd() { command -v "$1" >/dev/null 2>&1 || { echo "ERROR: missing: $1"; exit 1; }; }

autodetect_platform() {
  if [[ -n "${PLATFORM}" ]]; then
    echo "${PLATFORM}"; return
  fi
  local arch
  arch="$(uname -m)"
  case "${arch}" in
    x86_64)        echo "linux/amd64" ;;
    aarch64|arm64) echo "linux/arm64" ;;
    *) echo "ERROR: unsupported arch: ${arch}"; exit 1 ;;
  esac
}

image_ref() {
  if [[ -n "${IMAGE}" ]]; then
    echo "${IMAGE}"
  else
    echo "${IMAGE_REPO}:${IMAGE_TAG}"
  fi
}

container_exists() {
  docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"
}

do_install() {
  need_cmd docker

  local ref plat
  ref="$(image_ref)"
  plat="$(autodetect_platform)"

  local do_build=0
  if [[ "${FORCE_LOCAL_BUILD}" == "1" ]]; then
    do_build=1
  else
    echo "Pulling image: ${ref} (platform=${plat})"
    sudo docker pull --platform "${plat}" "${ref}" || do_build=1
  fi

  if [[ "${do_build}" == "1" ]]; then
    echo "Building local image: ${ref} (platform=${plat})"
    sudo docker build -f "${SCRIPT_DIR}/Docker/laser.Dockerfile" -t "${ref}" "${SCRIPT_DIR}"
  fi

  if container_exists; then
    echo "Removing existing container: ${CONTAINER_NAME}"
    sudo docker rm -f "${CONTAINER_NAME}" >/dev/null
  fi

  local run_args=(
    -d
    --name "${CONTAINER_NAME}"
    --net=host
    --ipc=host
    --restart "${RESTART_POLICY}"
    --cap-add SYS_NICE
    --ulimit rtprio=99
    --ulimit memlock=-1
    -v /dev:/dev

    -e AUTOSTART="${AUTOSTART}"
    -e SERIAL_PORT="${SERIAL_PORT}"
    -e SCANNER_PATH="${SCANNER_PATH}"
    -e NAMESPACE="${NAMESPACE}"
    -e DEBUG_MODE="${DEBUG_MODE}"
  )

  if [[ -n "${BOARD_ROLE}" ]]; then
    run_args+=( -e BOARD_ROLE="${BOARD_ROLE}" )
  fi

  if [[ "${PRIVILEGED}" == "1" ]]; then
    run_args+=( --privileged )
  fi

  echo "Starting container ${CONTAINER_NAME}..."
  sudo docker run "${run_args[@]}" "${ref}"

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
  echo "Updating image and recreating container..."
  do_install
}

if [[ $# -lt 1 ]]; then usage; exit 1; fi
action="$1"; shift

if [[ "${action}" == "install" ]]; then
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --name) CONTAINER_NAME="$2"; shift 2;;
      --image) IMAGE="$2"; shift 2;;
      --repo) IMAGE_REPO="$2"; shift 2;;
      --tag) IMAGE_TAG="$2"; shift 2;;
      --platform) PLATFORM="$2"; shift 2;;
      --local-build) FORCE_LOCAL_BUILD=1; shift;;
      --restart-policy) RESTART_POLICY="$2"; shift 2;;
      --privileged) PRIVILEGED=1; shift;;
      --no-privileged) PRIVILEGED=0; shift;;

      --serial-port) SERIAL_PORT="$2"; shift 2;;
      --namespace) NAMESPACE="$2"; shift 2;;
      --board-role) BOARD_ROLE="$2"; shift 2;;
      --debug) DEBUG_MODE="$2"; shift 2;;

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
