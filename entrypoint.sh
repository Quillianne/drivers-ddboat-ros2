#!/bin/bash
set -e
source "/opt/ros/$ROS_DISTRO/setup.bash"
if [ -f /opt/ws/install/setup.bash ]; then
  source /opt/ws/install/setup.bash
fi
exec "$@"
