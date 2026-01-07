#!/usr/bin/env bash
set -Eeuo pipefail

# ----------------------------
# Configurazione (override via env)
# ----------------------------
: "${IMAGE_REPO:=tabi43/unitree_ros2}"      # es. "tabi43/unitree_ros2"
: "${BASE_TAG:=base}"
: "${IF_TAG:=if}"
: "${ROS_DISTRO:=humble}"                   # se usato nei Dockerfile come ARG
: "${FORCE_BUILD:=1}"                       # 1 = non provare pull, build locale

BASE_REF="${IMAGE_REPO}:${BASE_TAG}"
IF_REF="${IMAGE_REPO}:${IF_TAG}"

# ----------------------------
# Platform locale
# ----------------------------
arch="$(uname -m)"
case "$arch" in
  x86_64)        PLATFORM="linux/amd64" ;;
  aarch64|arm64) PLATFORM="linux/arm64" ;;
  *) echo >&2 "Unsupported architecture: $arch"; exit 1 ;;
esac

# ----------------------------
# Helper
# ----------------------------
have_buildx() { docker buildx version >/dev/null 2>&1; }

try_pull() {
  local ref="$1"
  echo "[PULL] ${ref} (platform=${PLATFORM})"
  # --platform è supportato per multi-platform (API 1.32+). :contentReference[oaicite:3]{index=3}
  # Timeout best-effort per evitare hang infiniti su DNS/proxy rotti.
  docker pull --platform "${PLATFORM}" "${ref}"
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

# ----------------------------
# Main
# ----------------------------
echo "[INFO] BASE_REF=${BASE_REF}"
echo "[INFO] IF_REF=${IF_REF}"
echo "[INFO] PLATFORM=${PLATFORM}"

if [[ "${FORCE_BUILD}" -eq 0 ]]; then
  # 1) Pull IF di default -> se va, sei automaticamente “up-to-date”
  # (pullare di nuovo un tag serve a prendere la versione più recente). :contentReference[oaicite:4]{index=4}
  if try_pull "${IF_REF}"; then
    echo "[DONE] Using remote image (updated if needed): ${IF_REF}"
    exit 0
  fi

  echo "[WARN] Pull failed for ${IF_REF}. Will try to build locally."
else
  echo "[INFO] FORCE_BUILD=1 -> skipping pulls, building locally."
fi

# 2) Assicura base: prova pull, altrimenti build
if [[ "${FORCE_BUILD}" -eq 0 ]]; then
  if ! try_pull "${BASE_REF}"; then
    echo "[WARN] Pull failed for ${BASE_REF}. Building base locally."
    build_image "Docker/base.Dockerfile" "${BASE_REF}"
  else
    echo "[OK] Base pulled: ${BASE_REF}"
  fi
else
  build_image "Docker/base.Dockerfile" "${BASE_REF}"
fi

# 3) Build IF (dipende dalla base locale)
build_image "Docker/if.Dockerfile" "${IF_REF}"

echo "[DONE] Built locally: ${IF_REF}"