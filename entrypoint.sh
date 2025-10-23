#!/bin/bash
# Basic entrypoint for ROS Docker containers

# Source ROS2
source /opt/ros/${ROS_DISTRO}/setup.bash
echo Hello, welcome ot the ROSCON25-ES Workshop on Stonefish
echo Wait a moment while colcon builds our packages
colcon build
source install/setup.bash

# Execute the command passed into this entrypoint
exec "$@"
