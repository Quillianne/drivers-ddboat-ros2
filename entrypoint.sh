#!/usr/bin/env bash
set -e

# Base ROS 2 distro that is in /opt/ros/humble/setup.bash or /opt/humble/install/setup.bash
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/humble/install/setup.bash ]; then
    source /opt/humble/install/setup.bash
fi

# Isolated workspace overlay
source /opt/ws/install/ros2_ddboat/share/ros2_ddboat/local_setup.bash

exec "$@"
