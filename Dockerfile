FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /opt/ws

RUN apt-get update && \
    apt-get install -y build-essential && \
    rm -rf /var/lib/apt/lists/*

COPY . /opt/ws/src/drivers-ddboat-ros2
RUN unset CATKIN_INSTALL_INTO_PREFIX_ROOT && \
    . /opt/ros/humble/setup.sh && \
    colcon build --packages-select ros2_ddboat --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
