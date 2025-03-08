#!/bin/bash
set -e

#source "$ROS_WS/install/setup.bash"
source "/opt/ros/jazzy/setup.bash"
exec "$@"
