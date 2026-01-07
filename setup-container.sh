#!/usr/bin/env bash
set -Eeuo pipefail

: "${IMAGE_REPO:=tabi43/unitree_ros2}"
: "${IF_TAG:=if}"
: "${CONTAINER_NAME:=udp_ros2_if}"

# Flags modulari
: "${ENABLE_CAMERA:=0}"
: "${ENABLE_ULTRASOUND:=0}"
: "${ENABLE_FACE_LIGHTS:=0}"
: "${ENABLE_LOW:=1}"
: "${ENABLE_HIGH:=0}"

# Opzioni camera
: "${PUBLISH_RECTIFIED:=false}"
: "${PUBLISH_DEPTH:=false}"
: "${PUBLISH_PCL:=false}"

# Forza ruolo (opzionale): se vuoto usa IP detection
: "${BOARD_ROLE:=}"

# Force build locale: se 1, builda l'immagine localmente invece di fare pull
: "${FORCE_LOCAL_BUILD:=1}"

# restart policy (per debug puoi fare RESTART_POLICY=no)
: "${RESTART_POLICY:=unless-stopped}"

# privilegio
: "${PRIVILEGED:=1}"

: "${DEBUG_MODE:=1}"  # 0 : no debug (launchfile), 1: debug (no launchfile)

IF_REF="${IMAGE_REPO}:${IF_TAG}"

arch="$(uname -m)"
case "$arch" in
  x86_64)        PLATFORM="linux/amd64" ;;
  aarch64|arm64) PLATFORM="linux/arm64" ;;
  *) echo >&2 "Arch non supportata: $arch"; exit 1 ;;
esac

# Pull best-effort per aggiornare il tag locale (solo se non forziamo build locale)
if [[ "${FORCE_LOCAL_BUILD}" != "1" ]]; then
  echo "[PULL] ${IF_REF} (platform=${PLATFORM})"
  docker image pull --platform "${PLATFORM}" "${IF_REF}" || echo "[WARN] Pull fallito, uso immagine locale se presente."
else
  echo "[BUILD] Buildo immagine locale ${IF_REF}..."
  docker build -f Docker/if.Dockerfile -t "${IF_REF}" .
fi

# Se container esiste già, e vuoi solo avviarlo, mantieni la tua logica; qui la rendo più “container-friendly”:
if docker ps -a --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "[INFO] Il container ${CONTAINER_NAME} esiste già."
  docker update --restart "${RESTART_POLICY}" "${CONTAINER_NAME}" >/dev/null || true
  if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
    echo "[START] Avvio container esistente..."
    docker start "${CONTAINER_NAME}" >/dev/null
  else
    echo "[OK] È già in esecuzione."
  fi
  exit 0
fi

RUN_OPTS=(
  --name "${CONTAINER_NAME}"
  --detach
  --restart "${RESTART_POLICY}"
  --network host
  --ipc host
  --cap-add SYS_NICE
  --ulimit rtprio=99
  --ulimit memlock=-1
  -v /dev:/dev

  # feature flags
  -e ENABLE_CAMERA="${ENABLE_CAMERA}"
  -e ENABLE_ULTRASOUND="${ENABLE_ULTRASOUND}"
  -e ENABLE_FACE_LIGHTS="${ENABLE_FACE_LIGHTS}"
  -e ENABLE_LOW="${ENABLE_LOW}"
  -e ENABLE_HIGH="${ENABLE_HIGH}"

  # camera opts
  -e PUBLISH_RECTIFIED="${PUBLISH_RECTIFIED}"
  -e PUBLISH_DEPTH="${PUBLISH_DEPTH}"
  -e PUBLISH_PCL="${PUBLISH_PCL}"

  -e DEBUG_MODE="${DEBUG_MODE}"
)

if [[ -n "${BOARD_ROLE}" ]]; then
  RUN_OPTS+=( -e BOARD_ROLE="${BOARD_ROLE}" )
fi

if [[ "${PRIVILEGED}" == "1" ]]; then
  RUN_OPTS+=( --privileged )
fi

echo "[RUN] Creo e avvio ${CONTAINER_NAME} dalla ${IF_REF}..."
docker run "${RUN_OPTS[@]}" "${IF_REF}"

echo "[DONE] Container avviato con restart=${RESTART_POLICY}."
