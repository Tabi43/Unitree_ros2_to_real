#!/usr/bin/env bash
set -Eeuo pipefail

# Variabili configurabili
: "${ROS_DISTRO:=humble}"
: "${IMAGE_BASE:=udp_ros2_if}"
: "${CONTAINER_NAME:=udp_ros2_if}"
: "${START_CMD:=ros2 run unitree_ros2_interface interface_node}"

# Abilita privilegi estesi (opzionale): esporta PRIVILEGED=1 se ti serve
: "${PRIVILEGED:=1}"

arch="$(uname -m)"
case "$arch" in
  x86_64)  ARCH_TAG="amd64" ;;
  aarch64|arm64) ARCH_TAG="arm64" ;;
  *) echo "Arch non supportata: $arch"; exit 1 ;;
esac

IMAGE_TAG="${IMAGE_BASE}:${ROS_DISTRO}-${ARCH_TAG}"

# Se il container esiste già
if docker ps -a --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "[INFO] Il container ${CONTAINER_NAME} esiste già."
  # Assicura la policy di autostart
  docker update --restart unless-stopped "${CONTAINER_NAME}" >/dev/null
  # Se non è in esecuzione, avvialo
  if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
    echo "[START] Avvio container esistente..."
    docker start "${CONTAINER_NAME}" >/dev/null
  else
    echo "[OK] È già in esecuzione."
  fi
  exit 0
fi

# Opzioni di run
RUN_OPTS=(
  --name "${CONTAINER_NAME}"
  --detach
  --restart unless-stopped             # autostart al boot/riavvio Docker
  --network host                       # rete host per ROS 2/DDS
  --ipc host
  --cap-add SYS_NICE                   # per thread real-time
  --ulimit rtprio=99
  --ulimit memlock=-1
  -v /dev:/dev                         # accesso ai device dell'host (se serve)
  -e START_CMD="${START_CMD}"
)

if [ "${PRIVILEGED}" = "1" ]; then
  RUN_OPTS+=( --privileged )
fi

echo "[RUN] Creo e avvio ${CONTAINER_NAME} dalla ${IMAGE_TAG}..."
docker run "${RUN_OPTS[@]}" "${IMAGE_TAG}"
echo "[DONE] Container avviato con autostart (restart=unless-stopped)."
