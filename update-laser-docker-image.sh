#!/usr/bin/env bash
set -euo pipefail

############################################
# Multi-arch Docker build & push script
# for the Hokuyo laser scan image
#
# Usage:
#   ./update-laser-docker-image.sh [ACTION] [OPTIONS]
#
# Actions:
#   --update          Build and push the laser image (default)
#
# Options:
#   --amd64-only      Build only for linux/amd64
#   --arm64-only      Build only for linux/arm64
#   --native-only     Build only for the host architecture (fast dev mode)
#   --no-cache        Disable BuildKit layer cache
#   --purge-builder   Destroy and recreate the buildx builder (clears all cache)
############################################

ACTION=""
ARCH=""
NATIVE_ONLY=false
NO_CACHE=false
PURGE_BUILDER=false

for arg in "$@"; do
    case "${arg}" in
        --update)
            ACTION="${arg}" ;;
        --amd64-only)
            ARCH="amd64" ;;
        --arm64-only)
            ARCH="arm64" ;;
        --native-only)
            NATIVE_ONLY=true ;;
        --no-cache)
            NO_CACHE=true ;;
        --purge-builder)
            PURGE_BUILDER=true ;;
        *)
            echo "Unknown option: ${arg}"
            echo "Usage: $0 [--update] [--amd64-only|--arm64-only|--native-only] [--no-cache] [--purge-builder]"
            exit 1
            ;;
    esac
done
ACTION="${ACTION:---update}"

# -------- CONFIG --------
DOCKERHUB_USER="${DOCKERHUB_USER:-tabi43}"

IMAGE_NAME="${IMAGE_NAME:-unitree_ros2}"
TAG="${TAG:-hokuyo_laser}"
DOCKERFILE="${DOCKERFILE:-Docker/laser.Dockerfile}"

PLATFORMS="${PLATFORMS:-linux/amd64,linux/arm64}"
if [[ -n "${ARCH}" ]]; then
    PLATFORMS="linux/${ARCH}"
    echo "*** Arch-only mode: building for ${PLATFORMS} ***"
elif [[ "${NATIVE_ONLY}" == true ]]; then
    PLATFORMS="linux/$(uname -m | sed 's/x86_64/amd64/' | sed 's/aarch64/arm64/')"
    echo "*** Native-only mode: building for ${PLATFORMS} ***"
fi
BUILDER_NAME="${BUILDER_NAME:-multiarch_builder}"
CONTEXT_DIR="${CONTEXT_DIR:-.}"

# Local filesystem cache
CACHE_DIR="${CACHE_DIR:-/tmp/buildkit-cache}"
# ------------------------

FULL_IMAGE_NAME="${DOCKERHUB_USER}/${IMAGE_NAME}:${TAG}"

CACHE_FROM=("--cache-from" "type=local,src=${CACHE_DIR}/laser")
CACHE_TO=("--cache-to"   "type=local,dest=${CACHE_DIR}/laser,mode=max")

if [[ "${NO_CACHE}" == true ]]; then
    CACHE_FROM=() CACHE_TO=()
    echo "*** No-cache mode: rebuilding from scratch ***"
fi

# ---- Pre-flight checks ----
echo "==> Checking Docker login"
if ! docker login --username "${DOCKERHUB_USER}" 2>/dev/null; then
    if ! docker buildx imagetools inspect "docker.io/${FULL_IMAGE_NAME}" >/dev/null 2>&1 \
       && ! docker info 2>/dev/null | grep -qi "username"; then
        echo "ERROR: Cannot verify Docker Hub credentials for ${DOCKERHUB_USER}."
        echo "Run:  docker login"
        exit 1
    fi
fi

echo "==> Ensuring buildx is available"
docker buildx version > /dev/null

echo "==> Creating or selecting builder: ${BUILDER_NAME}"
if [[ "${PURGE_BUILDER}" == true ]]; then
    echo "    Purging builder '${BUILDER_NAME}' and local cache..."
    docker buildx rm "${BUILDER_NAME}" 2>/dev/null || true
    rm -rf "${CACHE_DIR}/laser"
    docker buildx create --name "${BUILDER_NAME}" --driver docker-container --use
elif ! docker buildx inspect "${BUILDER_NAME}" > /dev/null 2>&1; then
    docker buildx create --name "${BUILDER_NAME}" --driver docker-container --use
else
    docker buildx use "${BUILDER_NAME}"
fi

echo "==> Bootstrapping builder"
docker buildx inspect --bootstrap

# Helper: elapsed-time wrapper with retry
MAX_RETRIES="${MAX_RETRIES:-3}"
_build() {
    local label="$1"; shift
    local attempt start end elapsed
    for (( attempt=1; attempt<=MAX_RETRIES; attempt++ )); do
        start=$(date +%s)
        echo ""
        echo "==> ${label} (attempt ${attempt}/${MAX_RETRIES})"
        if "$@"; then
            end=$(date +%s); elapsed=$((end - start))
            echo "    ${label} succeeded in $((elapsed/60))m$((elapsed%60))s"
            return 0
        fi
        end=$(date +%s); elapsed=$((end - start))
        echo "!!! ${label} attempt ${attempt} FAILED after $((elapsed/60))m$((elapsed%60))s"
        if (( attempt < MAX_RETRIES )); then
            echo "    Retrying in 10s..."
            sleep 10
        fi
    done
    echo "!!! ${label} FAILED after ${MAX_RETRIES} attempts"
    return 1
}

# ---- Build & push laser image ----
if [[ "${ACTION}" == "--update" ]]; then
    _build "Building & pushing laser: ${FULL_IMAGE_NAME}" \
    docker buildx build \
        --platform "${PLATFORMS}" \
        "${CACHE_FROM[@]}" \
        "${CACHE_TO[@]}" \
        -t "${FULL_IMAGE_NAME}" \
        -f "${DOCKERFILE}" \
        "${CONTEXT_DIR}" \
        --push
fi

echo ""
echo "==> Done."
echo "  Laser: ${FULL_IMAGE_NAME}"
