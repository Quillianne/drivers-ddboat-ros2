FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /opt/ws

RUN apt-get update && \
    # install the C++ serial library and asio module used by the nodes
    apt-get install -y build-essential ros-humble-serial-driver \
                       ros-humble-asio-cmake-module && \
    rm -rf /var/lib/apt/lists/*

COPY . /opt/ws/src/drivers-ddboat-ros2
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select ros2_ddboat

CMD ["bash"]
