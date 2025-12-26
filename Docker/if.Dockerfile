ARG BASE_IMAGE=tabi43/unitree_ros2:base
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-lc"]

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_WS=/root/ros2_ws \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# ROS 2 ENVs
ENV ROS_LOCALHOST_ONLY=0 \
    ROS_DOMAIN_ID=6 \
    LDS_MODEL="none"

# CycloneDDS: selezione RMW + config XML (solo in IF)
RUN mkdir -p /etc/cyclonedds
COPY Docker/cyclonedds.xml /etc/cyclonedds/cyclonedds.xml

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml

# Workspace
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

# Copy source code
COPY ros2_ws/src src/

RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}

RUN bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --packages-ignore cv_bridge image_geometry \
    --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN find ${ROS_WS}/install -type f -perm -111 -exec setcap cap_sys_nice+ep {} \; || true

# Entrypoint
COPY Docker/interface_entrypoint.sh /usr/local/bin/interface_entrypoint.sh
RUN chmod +x /usr/local/bin/interface_entrypoint.sh

ENV START_CMD="ros2 run unitree_ros2_interface interface_node"

RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc && \
    echo "source ${ROS_WS}/install/setup.bash" >> /root/.bashrc

WORKDIR ${ROS_WS}

ENTRYPOINT ["/usr/local/bin/interface_entrypoint.sh"]
