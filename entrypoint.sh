#!/usr/bin/env bash
set -e

# Base ROS 2 distro
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Isolated workspace overlay
source /opt/ws/install/ros2_ddboat/share/ros2_ddboat/local_setup.bash

exec "$@"
