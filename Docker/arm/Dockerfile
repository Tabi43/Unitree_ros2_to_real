# Build: docker buildx build -f Dockerfile.arm64 -t udp_ros2_if:humble-arm64 --platform linux/arm64/v8 --build-arg ROS_DISTRO=humble .

ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    ROS_WS=/root/ros2_ws

RUN apt-get update && apt-get install -y --no-install-recommends \
#  ros-humble-desktop=0.10.0-1* \
  build-essential cmake git \
  python3-colcon-common-extensions python3-rosdep \
  libcap2-bin curl \
  iproute2 iputils-ping net-tools \
  libboost-all-dev \
  ros-${ROS_DISTRO}-rclcpp \
  ros-${ROS_DISTRO}-sensor-msgs \
  ros-${ROS_DISTRO}-std-msgs \
  ros-${ROS_DISTRO}-geometry-msgs && \
  rm -rf /var/lib/apt/lists/*


RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS

COPY . $ROS_WS/src/

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}

RUN bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN find $ROS_WS/install -type f -perm -111 -exec setcap cap_sys_nice+ep {} \; || true

COPY Docker/start_interface.sh /usr/local/bin/start_interface.sh
RUN chmod +x /usr/local/bin/start_interface.sh

ENV START_CMD="ros2 run unitree_ros2_interface interface_node"

RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc && \
    echo 'source ${ROS_WS}/install/setup.bash' >> /root/.bashrc

WORKDIR $ROS_WS
ENTRYPOINT ["/usr/local/bin/start_interface.sh"]
