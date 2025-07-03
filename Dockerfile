FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /opt/ws

# --- new: install Cyclone DDS RMW (and rosbridge, optional) -------------
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rosbridge-suite && \
    rm -rf /var/lib/apt/lists/*
# ------------------------------------------------------------------------

# Build tools for any native libs your packages need
RUN apt-get update && \
    apt-get install -y --no-install-recommends build-essential && \
    rm -rf /var/lib/apt/lists/*

# Copy workspace
COPY . /opt/ws/src/drivers-ddboat-ros2

# Build only your package
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
      --packages-select ros2_ddboat \
      --executor sequential \
      --cmake-args \
          -DCMAKE_BUILD_TYPE=MinSizeRel \
          -DBUILD_TESTING=OFF

# Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]