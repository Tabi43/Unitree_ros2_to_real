#!/usr/bin/env bash
set -Eeuo pipefail

# Configurable via env
: "${ROS_DISTRO:=humble}"
: "${IMAGE_BASE:=unitree_ros2_if}"

arch="$(uname -m)"
case "$arch" in
  x86_64)  ARCH_TAG="amd64"; PLATFORM="linux/amd64";     DF="Docker/Dockerfile" ;;
  aarch64|arm64) ARCH_TAG="arm64"; PLATFORM="linux/arm64/v8"; DF="Docker/Dockerfile" ;;
  *) echo "Unsupported architecture: $arch"; exit 1 ;;
esac

TAG="${IMAGE_BASE}:${ROS_DISTRO}-${ARCH_TAG}"

# Check if the image already exists
if docker image inspect "${TAG}" >/dev/null 2>&1; then
  echo "[INFO] Image ${TAG} already exists."
  read -p "Do you want to remove it and rebuild? [y/N]: " -n 1 -r
  echo
  if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "[REMOVE] Removing image ${TAG}..."
    docker rmi "${TAG}" || {
      echo "[ERROR] Unable to remove the image. It might be in use by a container."
      echo "Try stopping the containers using this image:"
      docker ps --filter ancestor="${TAG}" --format "table {{.ID}}\t{{.Names}}\t{{.Status}}"
      exit 1
    }
    echo "[OK] Image removed."
  else
    echo "[OK] Keeping the existing image. Exiting."
    exit 0
  fi
fi

echo "[BUILD] Building ${TAG} using ${DF} (platform ${PLATFORM})..."
if docker buildx version >/dev/null 2>&1; then
  # --load = load the image into the local Docker image store
  docker buildx build --load --platform "${PLATFORM}" -f "${DF}" \
    --build-arg ROS_DISTRO="${ROS_DISTRO}" \
    -t "${TAG}" .
else
  # fallback without buildx
  docker build -f "${DF}" \
    --build-arg ROS_DISTRO="${ROS_DISTRO}" \
    -t "${TAG}" .
fi

echo "[DONE] Image ready: ${TAG}"