#!/bin/bash
set -e

# Setup ROS environment
source "/opt/ros/melodic/setup.bash"
exec "$@" 