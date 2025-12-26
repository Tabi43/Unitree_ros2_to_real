#!/usr/bin/env bash
set -Eeuo pipefail

# ----------------------------
# Configurazione (override via env)
# ----------------------------
: "${IMAGE_REPO:=tabi43/unitree_ros2}"
: "${BASE_TAG:=base}"
: "${IF_TAG:=if}"

: "${ROS_DISTRO:=humble}"  # usato solo se nei Dockerfile hai ARG ROS_DISTRO

: "${CONTAINER_NAME:=unitree_ros2_interface}"

: "${PRIVILEGED:=1}"          # 1 per --privileged
: "${FORCE_BUILD:=0}"         # 1 = non prova pull, build locale
: "${RECREATE_ON_UPDATE:=1}"  # 1 = se l'immagine cambia, ricrea il container
: "${EXTRA_RUN_ARGS:=}"        # stringa con argomenti extra a docker run (opzionale)

BASE_REF="${IMAGE_REPO}:${BASE_TAG}"
IF_REF="${IMAGE_REPO}:${IF_TAG}"

# ----------------------------
# Platform locale
# ----------------------------
arch="$(uname -m)"
case "$arch" in
  x86_64)        PLATFORM="linux/amd64" ;;
  aarch64|arm64) PLATFORM="linux/arm64" ;;
  *) echo >&2 "Arch non supportata: $arch"; exit 1 ;;
esac

have_buildx() { docker buildx version >/dev/null 2>&1; }

try_pull() {
  local ref="$1"
  echo "[PULL] ${ref} (platform=${PLATFORM})"
  # docker image pull supporta --platform (API 1.32+) :contentReference[oaicite:2]{index=2}
  docker image pull --platform "${PLATFORM}" "${ref}"
}

build_image() {
  local dockerfile="$1"
  local tag="$2"

  echo "[BUILD] ${tag} (from ${dockerfile}) platform=${PLATFORM}"

  if have_buildx; then
    docker buildx build \
      --load \
      --platform "${PLATFORM}" \
      -f "${dockerfile}" \
      -t "${tag}" \
      --build-arg ROS_DISTRO="${ROS_DISTRO}" \
      --build-arg BASE_IMAGE="${BASE_REF}" \
      .
  else
    docker build \
      -f "${dockerfile}" \
      -t "${tag}" \
      --build-arg ROS_DISTRO="${ROS_DISTRO}" \
      --build-arg BASE_IMAGE="${BASE_REF}" \
      .
  fi
}

ensure_images() {
  if [[ "${FORCE_BUILD}" -eq 0 ]]; then
    # Pull IF di default: ripullare un tag serve ad avere la versione più aggiornata :contentReference[oaicite:3]{index=3}
    if try_pull "${IF_REF}"; then
      echo "[OK] IF pronta (pulled/updated): ${IF_REF}"
      return 0
    fi
    echo "[WARN] Pull IF fallito. Fallback a build locale."
  else
    echo "[INFO] FORCE_BUILD=1 -> skip pull, build locale."
  fi

  # Assicura BASE: prova pull, se fallisce build
  if [[ "${FORCE_BUILD}" -eq 0 ]]; then
    if ! try_pull "${BASE_REF}"; then
      echo "[WARN] Pull BASE fallito. Build base locale."
      build_image "Docker/base.Dockerfile" "${BASE_REF}"
    else
      echo "[OK] Base pulled: ${BASE_REF}"
    fi
  else
    build_image "Docker/base.Dockerfile" "${BASE_REF}"
  fi

  # Build IF locale (dipende dalla base locale)
  build_image "Docker/if.Dockerfile" "${IF_REF}"
  echo "[OK] IF buildata localmente: ${IF_REF}"
}

container_exists() {
  docker ps -a --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"
}

container_running() {
  docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"
}

image_id() {
  docker image inspect --format '{{.Id}}' "$1"
}

container_image_id() {
  docker container inspect --format '{{.Image}}' "$1"
}

recreate_container() {
  echo "[RECREATE] Stop & remove ${CONTAINER_NAME}..."
  docker rm -f "${CONTAINER_NAME}" >/dev/null
}

run_container() {
  # Opzioni di run
  RUN_OPTS=(
    --name "${CONTAINER_NAME}"
    --detach
    --restart unless-stopped                # autostart :contentReference[oaicite:4]{index=4}
    --network host                          # host networking (utile per DDS/ROS2) :contentReference[oaicite:5]{index=5}
    --ipc host
    --cap-add SYS_NICE                      # realtime scheduling helpers :contentReference[oaicite:6]{index=6}
    --ulimit rtprio=99                      # realtime prio :contentReference[oaicite:7]{index=7}
    --ulimit memlock=-1
    -v /dev:/dev                            # accesso a dispositivi USB/seriali
    --pull=never                            # non tentare pull implicito: l'abbiamo già gestito noi :contentReference[oaicite:8]{index=8}
  )

  if [[ "${PRIVILEGED}" == "1" ]]; then
    RUN_OPTS+=( --privileged )
  fi

  # Args extra (se forniti)
  if [[ -n "${EXTRA_RUN_ARGS}" ]]; then
    # shellcheck disable=SC2206
    RUN_OPTS+=( ${EXTRA_RUN_ARGS} )
  fi

  echo "[RUN] ${CONTAINER_NAME} from ${IF_REF}"
  docker container run "${RUN_OPTS[@]}" "${IF_REF}" >/dev/null
  echo "[DONE] Container avviato: ${CONTAINER_NAME}"
}

# ----------------------------
# Main
# ----------------------------
echo "[INFO] IF_REF=${IF_REF}"
echo "[INFO] PLATFORM=${PLATFORM}"

# 1) Assicura immagine (pull di default; fallback a build)
ensure_images

# 2) Se il container esiste, verifica se è allineato all'immagine corrente
if container_exists; then
  echo "[INFO] Il container ${CONTAINER_NAME} esiste già."
  docker update --restart unless-stopped "${CONTAINER_NAME}" >/dev/null || true  # :contentReference[oaicite:9]{index=9}

  if [[ "${RECREATE_ON_UPDATE}" -eq 1 ]]; then
    current_img="$(container_image_id "${CONTAINER_NAME}")"
    latest_img="$(image_id "${IF_REF}")"

    if [[ "${current_img}" != "${latest_img}" ]]; then
      echo "[INFO] Immagine aggiornata (container=${current_img}, latest=${latest_img}) -> ricreo container."
      recreate_container
      run_container
      exit 0
    fi
  fi

  # Se non serve ricreare: start se fermo
  if ! container_running; then
    echo "[START] Avvio container esistente..."
    docker start "${CONTAINER_NAME}" >/dev/null
  else
    echo "[OK] È già in esecuzione."
  fi
  exit 0
fi

# 3) Container non esiste: crealo
run_container
