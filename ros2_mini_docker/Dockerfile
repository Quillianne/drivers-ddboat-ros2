##########################################
# LAYER 0: Base image for ROS 2 Humble   #
##########################################

FROM ubuntu:22.04 AS layer0

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# enable “universe” (for vcstool, colcon) and add the ROS 2 apt key & repo
RUN apt-get update && apt-get install -y --no-install-recommends \
      software-properties-common curl gnupg2 lsb-release \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         | apt-key add - \
    && echo "deb [arch=armhf] http://packages.ros.org/ros2/ubuntu \
         $(lsb_release -cs) main" \
         > /etc/apt/sources.list.d/ros2.list \
    && rm -rf /var/lib/apt/lists/*

# ── Base tools ─────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
      locales build-essential cmake git curl wget gnupg2 lsb-release \
      python3 python3-pip python3-rosdep python3-vcstool python3-rosinstall-generator \
      python3-colcon-common-extensions \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ── Initialise rosdep (Tier-3 armhf needs source builds) ──
RUN rosdep init && rosdep update


######################################
# STAGE 1: Build ROS 2 Humble        #
######################################

FROM layer0 AS layer1

WORKDIR /ros2_ws
RUN mkdir src


RUN rosinstall_generator rclcpp sensor_msgs geometry_msgs std_msgs diagnostic_msgs std_srvs \
    ros2cli ros2launch launch launch_ros ament_cmake rosbridge_suite\
    --rosdistro humble \
    --deps \
    --tar \
    > humble-ros_base.repos \
    && vcs import src < humble-ros_base.repos


# For intalling ros_base, uncomment the following lines
# RUN rosinstall_generator ros_base \
#     --rosdistro humble \
#     --deps \
#     --tar \
#     > humble-ros_base.repos \
#     && vcs import src < humble-ros_base.repos

# install dependencies
RUN apt-get update && rosdep install -y \
    --from-paths src \
    --ignore-src \
    --skip-keys " \
        fastcdr \
        rti-connext-dds-6.0.1 \
        urdfdom_headers" \
    && rm -rf /var/lib/apt/lists/*

RUN colcon build \
    --symlink-install \
    --install-base /opt/ros/humble \
    --merge-install \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=MinSizeRel \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/humble \
      -DBUILD_TESTING=OFF \
      -DBUILD_EXAMPLES=OFF

CMD ["/bin/bash"]