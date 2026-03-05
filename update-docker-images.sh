#!/usr/bin/env bash
set -euo pipefail

############################################
# Multi-arch Docker build & push script
# Step 1: build & push the shared base image
# Step 2: build & push the interface image
# Builds for: linux/amd64, linux/arm64
#
# Usage:
#   ./update-docker-images.sh [ACTION]
#
# Actions:
#   --update-all         Build and push both base and interface images (default)
#   --update-base        Build and push only the base image
#   --update-interface   Build and push only the interface image
############################################

ACTION="${1:---update-all}"

case "${ACTION}" in
    --update-all|--update-base|--update-interface) ;;
    *)
        echo "Unknown action: ${ACTION}"
        echo "Usage: $0 [--update-all | --update-base | --update-interface]"
        exit 1
        ;;
esac

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

PLATFORMS="${PLATFORMS:-linux/amd64,linux/arm64}"
BUILDER_NAME="${BUILDER_NAME:-multiarch_builder}"
CONTEXT_DIR="${CONTEXT_DIR:-.}"
# ------------------------

FULL_BASE_IMAGE="${DOCKERHUB_USER}/${BASE_IMAGE_NAME}:${BASE_TAG}"
FULL_IMAGE_NAME="${DOCKERHUB_USER}/${IMAGE_NAME}:${TAG}"

echo "==> Checking Docker login"
if ! docker info | grep -q "Username"; then
    echo "You are not logged in. Run: docker login"
    exit 1
fi

echo "==> Ensuring buildx is available"
docker buildx version > /dev/null

echo "==> Creating or selecting builder: ${BUILDER_NAME}"
if ! docker buildx inspect "${BUILDER_NAME}" > /dev/null 2>&1; then
    docker buildx create --name "${BUILDER_NAME}" --use
else
    docker buildx use "${BUILDER_NAME}"
fi

echo "==> Bootstrapping builder"
docker buildx inspect --bootstrap

# ---- Step 1: base image ----
if [[ "${ACTION}" == "--update-all" || "${ACTION}" == "--update-base" ]]; then
    echo ""
    echo "==> Building and pushing base image: ${FULL_BASE_IMAGE}"
    docker buildx build \
        --platform "${PLATFORMS}" \
        -t "${FULL_BASE_IMAGE}" \
        -f "${BASE_DOCKERFILE}" \
        "${CONTEXT_DIR}" \
        --push
    echo "    Base image pushed: ${FULL_BASE_IMAGE}"
fi

# ---- Step 2: interface image ----
if [[ "${ACTION}" == "--update-all" || "${ACTION}" == "--update-interface" ]]; then
    echo ""
    echo "==> Building and pushing interface image: ${FULL_IMAGE_NAME}"
    docker buildx build \
        --platform "${PLATFORMS}" \
        --build-arg "BASE_IMAGE=${FULL_BASE_IMAGE}" \
        -t "${FULL_IMAGE_NAME}" \
        -f "${DOCKERFILE}" \
        "${CONTEXT_DIR}" \
        --push
    echo "    Interface image pushed: ${FULL_IMAGE_NAME}"
fi

echo ""
echo "==> Done."
if [[ "${ACTION}" == "--update-all" || "${ACTION}" == "--update-base" ]]; then
    echo "  Base:      ${FULL_BASE_IMAGE}"
fi
if [[ "${ACTION}" == "--update-all" || "${ACTION}" == "--update-interface" ]]; then
    echo "  Interface: ${FULL_IMAGE_NAME}"
fi