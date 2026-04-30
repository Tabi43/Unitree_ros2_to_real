#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# update-docker-images-v2.sh
#
# Simple Docker Buildx updater for Unitree ROS2 images.
#
# Modes:
#   --update-all        Build base + interface from scratch-ish Dockerfiles.
#   --update-base       Build only the base image.
#   --update-interface  Build only the interface image from the base image.
#   --quick-update      Rebuild the interface image starting FROM the existing
#                       published interface image. Fast path for source changes.
#
# Optional non-interactive Docker Hub login:
#   export DOCKERHUB_TOKEN=...
#
# Common usage:
#   ./update-docker-images-v2.sh --quick-update --native-only
#   ./update-docker-images-v2.sh --quick-update
#   ./update-docker-images-v2.sh --update-all

#######################################
# Defaults
#######################################

ACTION="all"
ACTION_WAS_SET=false

PLATFORMS="${PLATFORMS:-linux/amd64,linux/arm64}"
NO_CACHE=false
PURGE_BUILDER=false
DRY_RUN=false

DOCKERHUB_USER="${DOCKERHUB_USER:-tabi43}"

BASE_IMAGE_NAME="${BASE_IMAGE_NAME:-unitree_ros2}"
BASE_TAG="${BASE_TAG:-base-if}"
BASE_DOCKERFILE="${BASE_DOCKERFILE:-${SCRIPT_DIR}/Docker/base.Dockerfile}"

IMAGE_NAME="${IMAGE_NAME:-unitree_ros2}"
TAG="${TAG:-if}"
DOCKERFILE="${DOCKERFILE:-${SCRIPT_DIR}/Docker/if.Dockerfile}"
QUICK_DOCKERFILE="${QUICK_DOCKERFILE:-${SCRIPT_DIR}/Docker/if-quick.Dockerfile}"

CONTEXT_DIR="${CONTEXT_DIR:-${SCRIPT_DIR}}"
BUILDER_NAME="${BUILDER_NAME:-unitree_multiarch_builder}"
CACHE_DIR="${CACHE_DIR:-${HOME:-/tmp}/.cache/buildx-unitree-ros2}"

DOCKER_CONFIG_DIR="${DOCKER_CONFIG:-${HOME:-}/.docker}"
DOCKER=(docker)

FULL_BASE_IMAGE="${DOCKERHUB_USER}/${BASE_IMAGE_NAME}:${BASE_TAG}"
FULL_INTERFACE_IMAGE="${DOCKERHUB_USER}/${IMAGE_NAME}:${TAG}"

#######################################
# Utilities
#######################################

usage() {
    cat <<USAGE
Usage:
  $0 [ACTION] [PLATFORM] [OPTIONS]

Actions:
  --update-all          Build and push base + interface images. Default.
  --update-base         Build and push only the base image.
  --update-interface    Build and push only the interface image from the base image.
  --quick-update        Rebuild interface starting FROM the existing interface image.

Platforms:
  --amd64-only          Build only linux/amd64.
  --arm64-only          Build only linux/arm64.
  --native-only         Build only the current machine architecture.
  --platforms VALUE     Build exactly these Buildx platforms.
                         Example: --platforms linux/amd64,linux/arm64

Options:
  --no-cache            Disable BuildKit cache for this run.
  --purge-builder       Remove and recreate the Buildx builder and local cache.
  --dry-run             Print commands without executing them.
  -h, --help            Show this help.

Environment:
  DOCKERHUB_USER        Current: ${DOCKERHUB_USER}
  BASE_IMAGE_NAME       Current: ${BASE_IMAGE_NAME}
  BASE_TAG              Current: ${BASE_TAG}
  BASE_DOCKERFILE       Current: ${BASE_DOCKERFILE}
  IMAGE_NAME            Current: ${IMAGE_NAME}
  TAG                   Current: ${TAG}
  DOCKERFILE            Current: ${DOCKERFILE}
  QUICK_DOCKERFILE      Current: ${QUICK_DOCKERFILE}
  CONTEXT_DIR           Current: ${CONTEXT_DIR}
  BUILDER_NAME          Current: ${BUILDER_NAME}
  CACHE_DIR             Current: ${CACHE_DIR}
USAGE
}

log()  { printf '\n==> %s\n' "$*"; }
info() { printf '    %s\n' "$*"; }
warn() { printf 'WARNING: %s\n' "$*" >&2; }
die()  { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

run() {
    if [[ "${DRY_RUN}" == true ]]; then
        printf '+'
        printf ' %q' "$@"
        printf '\n'
    else
        "$@"
    fi
}

on_error() {
    local exit_code=$?
    printf '\nERROR: command failed at line %s with exit code %s\n' "${BASH_LINENO[0]}" "${exit_code}" >&2
    printf 'Failed command: %s\n' "${BASH_COMMAND}" >&2
    exit "${exit_code}"
}
trap on_error ERR

set_action_once() {
    local new_action="$1"
    if [[ "${ACTION_WAS_SET}" == true ]]; then
        die "Choose only one action."
    fi
    ACTION="${new_action}"
    ACTION_WAS_SET=true
}

native_platform() {
    case "$(uname -m)" in
        x86_64|amd64)  printf 'linux/amd64' ;;
        aarch64|arm64) printf 'linux/arm64' ;;
        *) die "Unsupported native architecture '$(uname -m)'. Use --platforms explicitly." ;;
    esac
}

require_command() {
    command -v "$1" >/dev/null 2>&1 || die "Required command not found: $1"
}

select_docker_client() {
    require_command docker

    if docker info >/dev/null 2>&1; then
        DOCKER=(docker)
        return
    fi

    if command -v sudo >/dev/null 2>&1; then
        if sudo -n env "DOCKER_CONFIG=${DOCKER_CONFIG_DIR}" docker info >/dev/null 2>&1; then
            warn "Docker requires sudo. Using sudo docker."
            DOCKER=(sudo env "DOCKER_CONFIG=${DOCKER_CONFIG_DIR}" docker)
            return
        fi

        warn "Docker requires sudo. You may be asked for your password."
        if sudo env "DOCKER_CONFIG=${DOCKER_CONFIG_DIR}" docker info >/dev/null; then
            DOCKER=(sudo env "DOCKER_CONFIG=${DOCKER_CONFIG_DIR}" docker)
            return
        fi
    fi

    die "Cannot access Docker daemon. Check Docker permissions or sudo access."
}

dockerhub_credentials_look_present() {
    local cfg="${DOCKER_CONFIG_DIR}/config.json"
    [[ -f "${cfg}" ]] || return 1
    grep -Eq 'credsStore|credHelpers|index\.docker\.io|https://index\.docker\.io/v1/|docker\.io' "${cfg}"
}

login_if_token_is_available() {
    if [[ -n "${DOCKERHUB_TOKEN:-}" ]]; then
        log "Logging in to Docker Hub using DOCKERHUB_TOKEN"
        printf '%s' "${DOCKERHUB_TOKEN}" | run "${DOCKER[@]}" login docker.io \
            --username "${DOCKERHUB_USER}" \
            --password-stdin
        return
    fi

    if dockerhub_credentials_look_present; then
        info "Docker Hub credentials found in ${DOCKER_CONFIG_DIR}/config.json"
    else
        warn "No Docker Hub credentials detected in ${DOCKER_CONFIG_DIR}/config.json."
        warn "If push fails, run manually: docker login --username ${DOCKERHUB_USER}"
    fi
}

check_paths() {
    [[ -d "${CONTEXT_DIR}" ]] || die "Build context directory not found: ${CONTEXT_DIR}"

    case "${ACTION}" in
        all)
            [[ -f "${BASE_DOCKERFILE}" ]] || die "Base Dockerfile not found: ${BASE_DOCKERFILE}"
            [[ -f "${DOCKERFILE}" ]] || die "Interface Dockerfile not found: ${DOCKERFILE}"
            ;;
        base)
            [[ -f "${BASE_DOCKERFILE}" ]] || die "Base Dockerfile not found: ${BASE_DOCKERFILE}"
            ;;
        interface)
            [[ -f "${DOCKERFILE}" ]] || die "Interface Dockerfile not found: ${DOCKERFILE}"
            ;;
        quick)
            [[ -f "${QUICK_DOCKERFILE}" ]] || die "Quick Dockerfile not found: ${QUICK_DOCKERFILE}"
            ;;
        *)
            die "Internal error: unknown ACTION=${ACTION}"
            ;;
    esac
}

ensure_buildx() {
    log "Checking Docker Buildx"
    run "${DOCKER[@]}" buildx version >/dev/null
}

ensure_builder() {
    log "Preparing Buildx builder: ${BUILDER_NAME}"

    if [[ "${PURGE_BUILDER}" == true ]]; then
        info "Removing builder and local cache"
        run "${DOCKER[@]}" buildx rm "${BUILDER_NAME}" >/dev/null 2>&1 || true
        run rm -rf "${CACHE_DIR}"
    fi

    if ! "${DOCKER[@]}" buildx inspect "${BUILDER_NAME}" >/dev/null 2>&1; then
        run "${DOCKER[@]}" buildx create \
            --name "${BUILDER_NAME}" \
            --driver docker-container \
            --use >/dev/null
    else
        run "${DOCKER[@]}" buildx use "${BUILDER_NAME}" >/dev/null
    fi

    run "${DOCKER[@]}" buildx inspect "${BUILDER_NAME}" --bootstrap >/dev/null
}

warn_if_platforms_look_unsupported() {
    local inspect_output
    inspect_output="$("${DOCKER[@]}" buildx inspect "${BUILDER_NAME}" --bootstrap 2>/dev/null || true)"

    IFS=',' read -r -a requested_platforms <<< "${PLATFORMS}"
    for platform in "${requested_platforms[@]}"; do
        platform="$(printf '%s' "${platform}" | xargs)"
        if ! grep -Fq "${platform}" <<< "${inspect_output}"; then
            warn "Builder output does not explicitly list platform '${platform}'. If it fails, enable QEMU/binfmt."
        fi
    done
}

cache_args_for() {
    local name="$1"
    local cache_path="${CACHE_DIR}/${name}"

    if [[ "${NO_CACHE}" == true ]]; then
        printf '%s\0' "--no-cache"
        return
    fi

    mkdir -p "${CACHE_DIR}"

    if [[ -d "${cache_path}" ]]; then
        printf '%s\0' "--cache-from" "type=local,src=${cache_path}"
    fi

    printf '%s\0' "--cache-to" "type=local,dest=${cache_path},mode=max"
}

image_has_requested_platforms() {
    local image="$1"
    local manifest

    manifest="$("${DOCKER[@]}" buildx imagetools inspect "${image}" 2>/dev/null)" || return 1

    IFS=',' read -r -a requested_platforms <<< "${PLATFORMS}"
    for platform in "${requested_platforms[@]}"; do
        platform="$(printf '%s' "${platform}" | xargs)"
        grep -Fq "${platform}" <<< "${manifest}" || return 1
    done

    return 0
}

require_existing_interface_for_quick_update() {
    log "Checking existing interface image for quick update"
    info "Image     : ${FULL_INTERFACE_IMAGE}"
    info "Platforms : ${PLATFORMS}"

    if ! image_has_requested_platforms "${FULL_INTERFACE_IMAGE}"; then
        die "Quick update needs an already-pushed ${FULL_INTERFACE_IMAGE} manifest containing ${PLATFORMS}. Run --update-all first, or select an existing platform with --native-only/--amd64-only/--arm64-only."
    fi
}

verify_manifest_platforms() {
    local image="$1"

    [[ "${DRY_RUN}" == true ]] && return

    log "Verifying pushed manifest: ${image}"

    local manifest
    manifest="$("${DOCKER[@]}" buildx imagetools inspect "${image}" 2>/dev/null)" \
        || die "Cannot inspect pushed image: ${image}"

    IFS=',' read -r -a requested_platforms <<< "${PLATFORMS}"
    for platform in "${requested_platforms[@]}"; do
        platform="$(printf '%s' "${platform}" | xargs)"
        if grep -Fq "${platform}" <<< "${manifest}"; then
            info "OK: ${image} contains ${platform}"
        else
            die "Pushed image ${image} does not contain platform ${platform}"
        fi
    done
}

build_and_push() {
    local label="$1"
    local dockerfile="$2"
    local image="$3"
    local cache_name="$4"
    local base_image="${5:-}"

    local cache_args=()
    while IFS= read -r -d '' item; do
        cache_args+=("${item}")
    done < <(cache_args_for "${cache_name}")

    local args=(
        buildx build
        --builder "${BUILDER_NAME}"
        --platform "${PLATFORMS}"
        --file "${dockerfile}"
        --tag "${image}"
    )

    if [[ -n "${base_image}" ]]; then
        args+=(--build-arg "BASE_IMAGE=${base_image}")
    fi

    args+=("${cache_args[@]}")
    args+=(--push "${CONTEXT_DIR}")

    log "${label}"
    info "Dockerfile : ${dockerfile}"
    info "Image      : ${image}"
    info "Platforms  : ${PLATFORMS}"
    [[ -n "${base_image}" ]] && info "Base image : ${base_image}"

    run "${DOCKER[@]}" "${args[@]}"
    verify_manifest_platforms "${image}"
}

print_summary() {
    cat <<SUMMARY

Done.

Images:
  Base image      : ${FULL_BASE_IMAGE}
  Interface image : ${FULL_INTERFACE_IMAGE}

Action:
  ${ACTION}

Platforms:
  ${PLATFORMS}

Builder:
  ${BUILDER_NAME}

Cache:
  ${CACHE_DIR}
SUMMARY
}

#######################################
# Argument parsing
#######################################

while [[ $# -gt 0 ]]; do
    case "$1" in
        --update-all|--all)
            set_action_once "all"; shift ;;
        --update-base|--base)
            set_action_once "base"; shift ;;
        --update-interface|--interface)
            set_action_once "interface"; shift ;;
        --quick-update|--quick)
            set_action_once "quick"; shift ;;
        --amd64-only|--amd64)
            PLATFORMS="linux/amd64"; shift ;;
        --arm64-only|--arm64)
            PLATFORMS="linux/arm64"; shift ;;
        --native-only|--native)
            PLATFORMS="$(native_platform)"; shift ;;
        --platforms)
            [[ $# -ge 2 ]] || die "--platforms requires a value."
            PLATFORMS="$2"; shift 2 ;;
        --platforms=*)
            PLATFORMS="${1#*=}"; shift ;;
        --no-cache)
            NO_CACHE=true; shift ;;
        --purge-builder)
            PURGE_BUILDER=true; shift ;;
        --dry-run)
            DRY_RUN=true; shift ;;
        -h|--help)
            usage; exit 0 ;;
        *)
            usage >&2; die "Unknown option: $1" ;;
    esac
done

#######################################
# Main
#######################################

log "Docker image update"
info "Action     : ${ACTION}"
info "Base       : ${FULL_BASE_IMAGE}"
info "Interface  : ${FULL_INTERFACE_IMAGE}"
info "Platforms  : ${PLATFORMS}"
info "Context    : ${CONTEXT_DIR}"

check_paths
select_docker_client
login_if_token_is_available
ensure_buildx
ensure_builder
warn_if_platforms_look_unsupported

case "${ACTION}" in
    all)
        build_and_push \
            "Building and pushing base image" \
            "${BASE_DOCKERFILE}" \
            "${FULL_BASE_IMAGE}" \
            "base"

        build_and_push \
            "Building and pushing interface image" \
            "${DOCKERFILE}" \
            "${FULL_INTERFACE_IMAGE}" \
            "interface" \
            "${FULL_BASE_IMAGE}"
        ;;
    base)
        build_and_push \
            "Building and pushing base image" \
            "${BASE_DOCKERFILE}" \
            "${FULL_BASE_IMAGE}" \
            "base"
        ;;
    interface)
        build_and_push \
            "Building and pushing interface image" \
            "${DOCKERFILE}" \
            "${FULL_INTERFACE_IMAGE}" \
            "interface" \
            "${FULL_BASE_IMAGE}"
        ;;
    quick)
        require_existing_interface_for_quick_update
        build_and_push \
            "Quick-updating interface image" \
            "${QUICK_DOCKERFILE}" \
            "${FULL_INTERFACE_IMAGE}" \
            "interface-quick" \
            "${FULL_INTERFACE_IMAGE}"
        ;;
esac

print_summary
