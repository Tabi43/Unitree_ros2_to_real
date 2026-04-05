# syntax=docker/dockerfile:1
# Quick incremental update: starts FROM the existing interface image (which
# already has compiled build/ and install/ artefacts), overlays fresh source,
# and runs colcon build.  CMake only recompiles translation units whose source
# files actually changed — dramatically cutting build time compared to a full
# rebuild.
ARG BASE_IMAGE=scratch
FROM ${BASE_IMAGE}

ARG TARGETARCH

SHELL ["/bin/bash", "-c"]

WORKDIR ${ROS_WS}

# Overlay fresh source on top of existing src/ — files that haven't changed
# on the host keep their original (older) timestamps, so make/ninja correctly
# skips unchanged objects.
COPY ros2_ws/src src/

# Also refresh CycloneDDS configs and entrypoint in case those changed
COPY Docker/cyclonedds/ /opt/cyclonedds/
COPY Docker/unitree_dds_env.sh /usr/local/bin/unitree_dds_env.sh
COPY Docker/interface_entrypoint.sh /usr/local/bin/interface_entrypoint.sh
RUN chmod +x /usr/local/bin/unitree_dds_env.sh /usr/local/bin/interface_entrypoint.sh

# Incremental colcon build — cmake detects what actually changed
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --parallel-workers "$(nproc)"

RUN find ${ROS_WS}/install -type f -perm -111 -exec setcap cap_sys_nice+ep {} \; || true
