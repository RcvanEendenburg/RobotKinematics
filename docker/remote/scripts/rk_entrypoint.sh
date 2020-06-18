#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/home/dev/workspace/devel/setup.bash"
exec "$@"
