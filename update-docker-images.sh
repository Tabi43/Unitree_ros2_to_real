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

# arm64 first: it is the architecture the robot actually runs, and the emulated
# one, so it is both the more important and the more likely to fail. Building it
# first means a failure surfaces before an hour is spent on the amd64 leg.
PLATFORMS="${PLATFORMS:-linux/arm64,linux/amd64}"
NO_CACHE=false
PURGE_BUILDER=false
DRY_RUN=false
LOCAL_ONLY=false

# Build parallelism, forwarded to the Dockerfiles as build-args. Peak concurrent
# compilers is COLCON_PARALLEL_WORKERS x MAKE_JOBS; under QEMU each one costs
# ~2 GB, so the default 2x4 fits comfortably in ~20 GB of available RAM.
# Raise on a bigger machine, lower if the arm64 leg still gets OOM-killed.
COLCON_PARALLEL_WORKERS="${COLCON_PARALLEL_WORKERS:-2}"
MAKE_JOBS="${MAKE_JOBS:-4}"

# Hard memory ceiling for the BuildKit container. Applied at builder creation
# only, so changing it requires --purge-builder.
BUILDER_MEMORY="${BUILDER_MEMORY:-22g}"

DOCKERHUB_USER="${DOCKERHUB_USER:-tabi43}"

BASE_IMAGE_NAME="${BASE_IMAGE_NAME:-unitree_ros2}"
BASE_TAG="${BASE_TAG:-base-if}"
BASE_DOCKERFILE="${BASE_DOCKERFILE:-${SCRIPT_DIR}/Docker/if-base.Dockerfile}"

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
  --local               Load the image into the local Docker daemon instead of
                        pushing it. Requires exactly one platform.
  --workers N           colcon packages built concurrently. Current: ${COLCON_PARALLEL_WORKERS}
  --jobs N              Compile jobs per package (MAKEFLAGS). Current: ${MAKE_JOBS}
  --builder-memory VAL  Memory cap for the BuildKit container, applied only when
                        the builder is created. Current: ${BUILDER_MEMORY}
  --no-cache            Disable BuildKit cache for this run.
  --purge-builder       Remove and recreate the Buildx builder and local cache.
  --dry-run             Print commands without executing them.
  -h, --help            Show this help.

Notes:
  Peak concurrent compilers is --workers x --jobs. Each one costs roughly 2 GB
  under QEMU emulation, so keep the product below (available RAM in GB) / 2.
  Platforms are always built one at a time and joined into a manifest at the end.

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
        # memory-swap == memory disables swap inside the builder, so an
        # out-of-memory compile dies fast in its own cgroup instead of dragging
        # the host through hours of swap thrash.
        info "Creating builder with memory limit ${BUILDER_MEMORY}"
        run "${DOCKER[@]}" buildx create \
            --name "${BUILDER_NAME}" \
            --driver docker-container \
            --driver-opt "memory=${BUILDER_MEMORY}" \
            --driver-opt "memory-swap=${BUILDER_MEMORY}" \
            --use >/dev/null
    else
        run "${DOCKER[@]}" buildx use "${BUILDER_NAME}" >/dev/null
    fi

    run "${DOCKER[@]}" buildx inspect "${BUILDER_NAME}" --bootstrap >/dev/null
    warn_if_builder_is_unbounded
}

# Driver options only take effect when the builder is created. A builder made
# before this script grew a memory limit keeps running unbounded, which is how a
# runaway compile takes down the whole workstation instead of just the build.
warn_if_builder_is_unbounded() {
    [[ "${DRY_RUN}" == true ]] && return 0
    local mem
    mem="$("${DOCKER[@]}" inspect "buildx_buildkit_${BUILDER_NAME}0" \
             --format '{{.HostConfig.Memory}}' 2>/dev/null)" || return 0
    if [[ "${mem}" == "0" ]]; then
        warn "Builder '${BUILDER_NAME}' has no memory limit. Re-run with --purge-builder to recreate it capped at ${BUILDER_MEMORY}."
    fi
    return 0
}

# Verify the builder can actually execute foreign binaries.
#
# The platform list from `buildx inspect` is not trustworthy: this builder
# advertises only linux/amd64 and linux/386 yet builds linux/arm64 correctly,
# because BuildKit registers its own bundled QEMU handlers inside the builder
# container. Matching against that list produces a false warning on every run,
# so probe for real instead. The probe is a two-line image against an empty
# context and is cached after the first run.
check_emulation_for_platforms() {
    [[ "${DRY_RUN}" == true ]] && return 0

    local platforms=() platform probe_ctx native
    native="$(native_platform)"
    IFS=',' read -r -a platforms <<< "${PLATFORMS}"

    probe_ctx="$(mktemp -d)"
    printf 'FROM busybox\nRUN /bin/true\n' > "${probe_ctx}/Dockerfile"

    for platform in "${platforms[@]}"; do
        platform="$(printf '%s' "${platform}" | xargs)"
        [[ "${platform}" == "${native}" ]] && continue
        info "Probing ${platform} emulation"
        if ! "${DOCKER[@]}" buildx build --builder "${BUILDER_NAME}" \
                 --platform "${platform}" "${probe_ctx}" >/dev/null 2>&1; then
            rm -rf "${probe_ctx}"
            die "Builder '${BUILDER_NAME}' cannot execute ${platform} binaries. Install the QEMU handlers with: docker run --privileged --rm tonistiigi/binfmt --install all"
        fi
    done

    rm -rf "${probe_ctx}"
    return 0
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

# build_and_push LABEL DOCKERFILE IMAGE [BASE_IMAGE]
#
# Platforms are built ONE AT A TIME, each into its own :<tag>-<arch> image, then
# joined into a multi-arch manifest with `imagetools create`. Passing both
# platforms to a single buildx invocation builds them concurrently, which doubles
# peak memory: even with the job caps that is ~16 concurrent compilers at ~2 GB
# each, well past the 29 GB on this host. Serializing also means a failure tells
# you which architecture broke instead of interleaving two logs.
#
# With --local the single requested platform is loaded into the local Docker
# daemon instead, so the image can be tested before it is published.
build_and_push() {
    local label="$1"
    local dockerfile="$2"
    local image="$3"
    local base_image="${4:-}"

    log "${label}"
    info "Dockerfile : ${dockerfile}"
    info "Image      : ${image}"
    info "Platforms  : ${PLATFORMS} (built sequentially)"
    info "Parallelism: ${COLCON_PARALLEL_WORKERS} packages x ${MAKE_JOBS} compile jobs"
    info "Output     : $([[ "${LOCAL_ONLY}" == true ]] && echo 'local --load' || echo 'push + manifest')"
    [[ -n "${base_image}" ]] && info "Base image : ${base_image}"

    local platforms=() per_arch_tags=() platform arch args
    IFS=',' read -r -a platforms <<< "${PLATFORMS}"

    for platform in "${platforms[@]}"; do
        platform="$(printf '%s' "${platform}" | xargs)"
        arch="${platform##*/}"

        args=(
            buildx build
            --builder "${BUILDER_NAME}"
            --platform "${platform}"
            --file "${dockerfile}"
            --build-arg "COLCON_PARALLEL_WORKERS=${COLCON_PARALLEL_WORKERS}"
            --build-arg "MAKE_JOBS=${MAKE_JOBS}"
        )
        [[ -n "${base_image}" ]] && args+=(--build-arg "BASE_IMAGE=${base_image}")
        [[ "${NO_CACHE}" == true ]] && args+=(--no-cache)

        if [[ "${LOCAL_ONLY}" == true ]]; then
            log "Building ${platform} -> ${image} (local only)"
            args+=(--tag "${image}" --load "${CONTEXT_DIR}")
        else
            log "Building ${platform} -> ${image}-${arch}"
            args+=(--tag "${image}-${arch}" --push "${CONTEXT_DIR}")
            per_arch_tags+=("${image}-${arch}")
        fi

        run "${DOCKER[@]}" "${args[@]}"
    done

    if [[ "${LOCAL_ONLY}" == true ]]; then
        info "Loaded into the local daemon as ${image}. Nothing pushed."
        return 0
    fi

    log "Joining per-arch images into manifest: ${image}"
    run "${DOCKER[@]}" buildx imagetools create --tag "${image}" "${per_arch_tags[@]}"
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
  ${BUILDER_NAME} (memory cap ${BUILDER_MEMORY})

Jobs:
  ${COLCON_PARALLEL_WORKERS} packages x ${MAKE_JOBS} compile jobs
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
        --local|--load)
            LOCAL_ONLY=true; shift ;;
        --workers)
            [[ $# -ge 2 ]] || die "--workers requires a value."
            COLCON_PARALLEL_WORKERS="$2"; shift 2 ;;
        --workers=*)
            COLCON_PARALLEL_WORKERS="${1#*=}"; shift ;;
        --jobs)
            [[ $# -ge 2 ]] || die "--jobs requires a value."
            MAKE_JOBS="$2"; shift 2 ;;
        --jobs=*)
            MAKE_JOBS="${1#*=}"; shift ;;
        --builder-memory)
            [[ $# -ge 2 ]] || die "--builder-memory requires a value."
            BUILDER_MEMORY="$2"; shift 2 ;;
        --builder-memory=*)
            BUILDER_MEMORY="${1#*=}"; shift ;;
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

validate_options() {
    [[ "${COLCON_PARALLEL_WORKERS}" =~ ^[1-9][0-9]*$ ]] || die "--workers must be a positive integer, got '${COLCON_PARALLEL_WORKERS}'."
    [[ "${MAKE_JOBS}" =~ ^[1-9][0-9]*$ ]] || die "--jobs must be a positive integer, got '${MAKE_JOBS}'."

    [[ "${LOCAL_ONLY}" == false ]] && return 0

    # `docker buildx --load` can only materialise a single platform.
    [[ "${PLATFORMS}" == *,* ]] && \
        die "--local builds one platform at a time. Add --native-only, --amd64-only or --arm64-only."

    # The docker-container driver resolves FROM from the registry, never from the
    # local daemon, so a locally-loaded base image would be silently ignored by
    # the interface build that follows it.
    [[ "${ACTION}" == "all" ]] && \
        die "--local cannot chain base -> interface: the interface build would pull the base from the registry. Push the base first (--update-base), then run --update-interface --local."

    [[ "${ACTION}" == "base" ]] && \
        warn "A locally-loaded base image is not visible to later buildx builds; they resolve the base from the registry."

    return 0
}

log "Docker image update"
info "Action     : ${ACTION}"
info "Base       : ${FULL_BASE_IMAGE}"
info "Interface  : ${FULL_INTERFACE_IMAGE}"
info "Platforms  : ${PLATFORMS}"
info "Context    : ${CONTEXT_DIR}"

validate_options
check_paths
select_docker_client
login_if_token_is_available
ensure_buildx
ensure_builder
check_emulation_for_platforms

case "${ACTION}" in
    all)
        build_and_push \
            "Building and pushing base image" \
            "${BASE_DOCKERFILE}" \
            "${FULL_BASE_IMAGE}"

        build_and_push \
            "Building and pushing interface image" \
            "${DOCKERFILE}" \
            "${FULL_INTERFACE_IMAGE}" \
            "${FULL_BASE_IMAGE}"
        ;;
    base)
        build_and_push \
            "Building and pushing base image" \
            "${BASE_DOCKERFILE}" \
            "${FULL_BASE_IMAGE}"
        ;;
    interface)
        build_and_push \
            "Building and pushing interface image" \
            "${DOCKERFILE}" \
            "${FULL_INTERFACE_IMAGE}" \
            "${FULL_BASE_IMAGE}"
        ;;
    quick)
        require_existing_interface_for_quick_update
        build_and_push \
            "Quick-updating interface image" \
            "${QUICK_DOCKERFILE}" \
            "${FULL_INTERFACE_IMAGE}" \
            "${FULL_INTERFACE_IMAGE}"
        ;;
esac

print_summary
