# syntax=docker/dockerfile:1

# Quick interface rebuild.
#
# This Dockerfile intentionally starts from the already-published interface image,
# not from the base image. It keeps the heavy OS/ROS/dependency layers and only
# refreshes the workspace source, optional missing rosdep dependencies, DDS config,
# entrypoint scripts, and the colcon build output.

ARG BASE_IMAGE=tabi43/unitree_ros2:if
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG COLCON_PARALLEL_WORKERS=2

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_WS=/root/ros2_ws \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# Keep the runtime defaults explicit, even if the previous interface image already
# contained them.
ENV ROS_LOCALHOST_ONLY=0 \
    ROS_DOMAIN_ID=43 \
    LDS_MODEL=none

# -----------------------------------------------------------------------------
# Sanity checks
# -----------------------------------------------------------------------------
RUN set -ex; \
    test -n "${ROS_DISTRO:-}"; \
    test -f "/opt/ros/${ROS_DISTRO}/setup.bash"; \
    command -v rosdep >/dev/null; \
    command -v colcon >/dev/null; \
    mkdir -p "${ROS_WS}/src"

WORKDIR ${ROS_WS}

# -----------------------------------------------------------------------------
# Clean only the workspace parts that must be rebuilt.
# Do not remove OS/ROS/dependency layers inherited from the previous image.
# -----------------------------------------------------------------------------
RUN set -ex; \
    rm -rf src build install log; \
    mkdir -p src

# Copy the full current workspace source. This is slightly less cache-optimal than
# copying only package.xml files first, but it is much more robust when packages
# are added, removed, or renamed.
COPY ros2_ws/src/ src/

# Refresh runtime configuration files.
COPY Docker/cyclonedds/ /opt/cyclonedds/
COPY Docker/unitree_dds_env.sh /usr/local/bin/unitree_dds_env.sh
COPY Docker/interface_entrypoint.sh /usr/local/bin/interface_entrypoint.sh

RUN set -ex; \
    chmod +x /usr/local/bin/unitree_dds_env.sh /usr/local/bin/interface_entrypoint.sh

# -----------------------------------------------------------------------------
# Install only missing dependencies declared by the current workspace.
# No BuildKit apt cache mounts here: this is intentionally boring and predictable.
# -----------------------------------------------------------------------------
RUN set -ex; \
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi; \
    rosdep update --rosdistro "${ROS_DISTRO}" || rosdep update; \
    apt-get update; \
    rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro "${ROS_DISTRO}" \
        -r \
        -y; \
    rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Rebuild the workspace from a clean state.
# Keep parallelism conservative because arm64 multi-arch builds often run through
# emulation and can fail due to memory pressure when using all cores.
# -----------------------------------------------------------------------------
RUN set -ex; \
    source "/opt/ros/${ROS_DISTRO}/setup.bash"; \
    colcon build \
        --symlink-install \
        --event-handlers console_direct+ \
        --parallel-workers "${COLCON_PARALLEL_WORKERS}" \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Give real-time priority capability where possible. Do not fail the image build
# if setcap is unavailable or unsupported in the builder environment.
RUN find "${ROS_WS}/install" -type f -perm -111 -exec setcap cap_sys_nice+ep {} \; || true

# Make interactive shells source ROS and the rebuilt workspace without appending
# duplicate lines every time the quick image is rebuilt.
RUN set -ex; \
    grep -qxF "source /opt/ros/${ROS_DISTRO}/setup.bash" /root/.bashrc || \
        echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc; \
    grep -qxF "source ${ROS_WS}/install/setup.bash" /root/.bashrc || \
        echo "source ${ROS_WS}/install/setup.bash" >> /root/.bashrc

WORKDIR ${ROS_WS}
ENTRYPOINT ["/usr/local/bin/interface_entrypoint.sh"]
