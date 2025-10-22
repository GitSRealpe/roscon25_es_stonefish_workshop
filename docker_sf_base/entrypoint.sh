#!/bin/bash
# Basic entrypoint for ROS Docker containers

# Source ROS2
source /opt/ros/${ROS_DISTRO}/setup.bash

# Execute the command passed into this entrypoint
exec "$@"
