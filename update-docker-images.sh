#!/usr/bin/env bash
set -euo pipefail

############################################
# Multi-arch Docker build & push script
# Step 1: build & push the shared base image
# Step 2: build & push the interface image
# Builds for: linux/amd64, linux/arm64
#
# Usage:
#   ./update-docker-images.sh [ACTION] [OPTIONS]
#
# Actions:
#   --update-all         Build and push both base and interface images (default)
#   --update-base        Build and push only the base image
#   --update-interface   Build and push only the interface image
#   --quick-update       Pull existing interface image and do incremental source-only
#                        rebuild.  Falls back to full build if no image exists on Hub.
#
# Options:
#   --amd64-only         Build only for linux/amd64
#   --arm64-only         Build only for linux/arm64
#   --native-only        Build only for the host architecture (fast dev mode)
#   --no-cache           Disable BuildKit layer cache
#   --purge-builder      Destroy and recreate the buildx builder (clears all cache)
############################################

ACTION=""
ARCH=""       # empty = both amd64+arm64, "amd64", or "arm64"
NATIVE_ONLY=false
NO_CACHE=false
PURGE_BUILDER=false

for arg in "$@"; do
    case "${arg}" in
        --update-all|--update-base|--update-interface|--quick-update)
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
            echo "Usage: $0 [--update-all|--update-base|--update-interface|--quick-update] [--amd64-only|--arm64-only|--native-only] [--no-cache] [--purge-builder]"
            exit 1
            ;;
    esac
done
ACTION="${ACTION:---update-all}"

# -------- CONFIG --------
DOCKERHUB_USER="${DOCKERHUB_USER:-tabi43}"

# Base image
BASE_IMAGE_NAME="${BASE_IMAGE_NAME:-unitree_ros2}"
BASE_TAG="${BASE_TAG:-base-if}"
BASE_DOCKERFILE="${BASE_DOCKERFILE:-Docker/base.Dockerfile}"

# Interface image
IMAGE_NAME="${IMAGE_NAME:-unitree_ros2}"
TAG="${TAG:-if}"
DOCKERFILE="${DOCKERFILE:-Docker/if.Dockerfile}"

# Quick-update (incremental) image
QUICK_DOCKERFILE="${QUICK_DOCKERFILE:-Docker/if-quick.Dockerfile}"

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

# Local filesystem cache — avoids Docker Hub blob-size limits ("400 Bad request")
# that plague type=registry,mode=max with large ROS2 images.
CACHE_DIR="${CACHE_DIR:-/tmp/buildkit-cache}"
# ------------------------

FULL_BASE_IMAGE="${DOCKERHUB_USER}/${BASE_IMAGE_NAME}:${BASE_TAG}"
FULL_IMAGE_NAME="${DOCKERHUB_USER}/${IMAGE_NAME}:${TAG}"

# Build cache flags — local disk, mode=max caches all intermediate layers
CACHE_FROM_BASE=("--cache-from" "type=local,src=${CACHE_DIR}/base")
CACHE_TO_BASE=("--cache-to"   "type=local,dest=${CACHE_DIR}/base,mode=max")
CACHE_FROM_IF=("--cache-from" "type=local,src=${CACHE_DIR}/if")
CACHE_TO_IF=("--cache-to"   "type=local,dest=${CACHE_DIR}/if,mode=max")

if [[ "${NO_CACHE}" == true ]]; then
    CACHE_FROM_BASE=() CACHE_TO_BASE=()
    CACHE_FROM_IF=()   CACHE_TO_IF=()
    echo "*** No-cache mode: rebuilding from scratch ***"
fi

# ---- Pre-flight checks ----
echo "==> Checking Docker login"
if ! docker login --username "${DOCKERHUB_USER}" 2>/dev/null; then
    # Already logged in? Try a token-based check.
    if ! docker buildx imagetools inspect "docker.io/${DOCKERHUB_USER}/${BASE_IMAGE_NAME}:${BASE_TAG}" >/dev/null 2>&1 \
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
    rm -rf "${CACHE_DIR}"
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

# Helper: check that an image manifest contains ALL requested platforms
_has_all_platforms() {
    local image="$1"
    local manifest
    manifest=$(docker buildx imagetools inspect "${image}" 2>/dev/null) || return 1
    # PLATFORMS is comma-separated, e.g. "linux/amd64,linux/arm64"
    IFS=',' read -ra plats <<< "${PLATFORMS}"
    for p in "${plats[@]}"; do
        if ! grep -q "${p}" <<< "${manifest}"; then
            echo "    Platform ${p} NOT found in ${image}"
            return 1
        fi
    done
    return 0
}

# ---- Quick incremental update (pull existing image + overlay source + incremental build) ----
# Falls back to standard full build (base + interface) when the image is missing or incomplete.
if [[ "${ACTION}" == "--quick-update" ]]; then
    QUICK_OK=false
    echo "==> Quick update: checking if ${FULL_IMAGE_NAME} has all platforms (${PLATFORMS})..."
    if _has_all_platforms "${FULL_IMAGE_NAME}"; then
        echo "    All platforms found — will overlay source and do incremental build."
        if _build "Quick-updating: ${FULL_IMAGE_NAME}" \
            docker buildx build \
                --platform "${PLATFORMS}" \
                "${CACHE_FROM_IF[@]}" \
                "${CACHE_TO_IF[@]}" \
                --build-arg "BASE_IMAGE=${FULL_IMAGE_NAME}" \
                -t "${FULL_IMAGE_NAME}" \
                -f "${QUICK_DOCKERFILE}" \
                "${CONTEXT_DIR}" \
                --push; then
            QUICK_OK=true
        fi
    else
        echo "    Image missing or incomplete for requested platforms."
    fi

    if [[ "${QUICK_OK}" == false ]]; then
        echo "==> Falling back to standard full build (base + interface)..."
        _build "Building & pushing base: ${FULL_BASE_IMAGE}" \
        docker buildx build \
            --platform "${PLATFORMS}" \
            "${CACHE_FROM_BASE[@]}" \
            "${CACHE_TO_BASE[@]}" \
            -t "${FULL_BASE_IMAGE}" \
            -f "${BASE_DOCKERFILE}" \
            "${CONTEXT_DIR}" \
            --push

        _build "Building & pushing interface: ${FULL_IMAGE_NAME}" \
        docker buildx build \
            --platform "${PLATFORMS}" \
            "${CACHE_FROM_IF[@]}" \
            "${CACHE_TO_IF[@]}" \
            --build-arg "BASE_IMAGE=${FULL_BASE_IMAGE}" \
            -t "${FULL_IMAGE_NAME}" \
            -f "${DOCKERFILE}" \
            "${CONTEXT_DIR}" \
            --push
    fi
fi

# ---- Step 1: base image ----
if [[ "${ACTION}" == "--update-all" || "${ACTION}" == "--update-base" ]]; then
    _build "Building & pushing base: ${FULL_BASE_IMAGE}" \
    docker buildx build \
        --platform "${PLATFORMS}" \
        "${CACHE_FROM_BASE[@]}" \
        "${CACHE_TO_BASE[@]}" \
        -t "${FULL_BASE_IMAGE}" \
        -f "${BASE_DOCKERFILE}" \
        "${CONTEXT_DIR}" \
        --push
fi

# ---- Step 2: interface image ----
if [[ "${ACTION}" == "--update-all" || "${ACTION}" == "--update-interface" ]]; then
    _build "Building & pushing interface: ${FULL_IMAGE_NAME}" \
    docker buildx build \
        --platform "${PLATFORMS}" \
        "${CACHE_FROM_IF[@]}" \
        "${CACHE_TO_IF[@]}" \
        --build-arg "BASE_IMAGE=${FULL_BASE_IMAGE}" \
        -t "${FULL_IMAGE_NAME}" \
        -f "${DOCKERFILE}" \
        "${CONTEXT_DIR}" \
        --push
fi

echo ""
echo "==> Done."
if [[ "${ACTION}" == "--update-all" || "${ACTION}" == "--update-base" ]]; then
    echo "  Base:      ${FULL_BASE_IMAGE}"
fi
if [[ "${ACTION}" == "--update-all" || "${ACTION}" == "--update-interface" ]]; then
    echo "  Interface: ${FULL_IMAGE_NAME}"
fi
if [[ "${ACTION}" == "--quick-update" ]]; then
    echo "  Interface (quick): ${FULL_IMAGE_NAME}"
fi