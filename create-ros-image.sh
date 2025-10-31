#!/usr/bin/env bash
set -Eeuo pipefail

# Configurabili via env
: "${ROS_DISTRO:=humble}"
: "${IMAGE_BASE:=udp_ros2_if}"

arch="$(uname -m)"
case "$arch" in
  x86_64)  ARCH_TAG="amd64"; PLATFORM="linux/amd64";     DF="Docker/amd/Dockerfile" ;;
  aarch64|arm64) ARCH_TAG="arm64"; PLATFORM="linux/arm64/v8"; DF="Docker/arm/Dockerfile" ;;
  *) echo "Arch non supportata: $arch"; exit 1 ;;
esac

TAG="${IMAGE_BASE}:${ROS_DISTRO}-${ARCH_TAG}"

# Controllo se l'immagine esiste già
if docker image inspect "${TAG}" >/dev/null 2>&1; then
  echo "[INFO] L'immagine ${TAG} esiste già."
  read -p "Vuoi eliminarla e ricrearla? [y/N]: " -n 1 -r
  echo
  if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "[REMOVE] Eliminazione immagine ${TAG}..."
    docker rmi "${TAG}" || {
      echo "[ERROR] Impossibile eliminare l'immagine. Potrebbe essere in uso da un container."
      echo "Prova a fermare i container che usano questa immagine:"
      docker ps --filter ancestor="${TAG}" --format "table {{.ID}}\t{{.Names}}\t{{.Status}}"
      exit 1
    }
    echo "[OK] Immagine eliminata."
  else
    echo "[OK] Mantengo l'immagine esistente. Uscita."
    exit 0
  fi
fi

echo "[BUILD] Costruisco ${TAG} con ${DF} (platform ${PLATFORM})..."
if docker buildx version >/dev/null 2>&1; then
  # --load = carica l'immagine nel Docker locale
  docker buildx build --load --platform "${PLATFORM}" -f "${DF}" \
    --build-arg ROS_DISTRO="${ROS_DISTRO}" \
    -t "${TAG}" .
else
  # fallback senza buildx
  docker build -f "${DF}" \
    --build-arg ROS_DISTRO="${ROS_DISTRO}" \
    -t "${TAG}" .
fi

echo "[DONE] Immagine pronta: ${TAG}"
