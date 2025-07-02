FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /opt/ws

RUN apt-get update && \
    apt-get install -y build-essential && \
    rm -rf /var/lib/apt/lists/*

COPY . /opt/ws/src/drivers-ddboat-ros2
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select ros2_ddboat --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash", "-c", "source /opt/ws/install/setup.bash && exec bash"]
