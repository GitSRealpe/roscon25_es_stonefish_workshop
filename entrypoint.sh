#!/bin/bash
# Basic entrypoint for ROS Docker containers

# Source ROS2
source /opt/ros/${ROS_DISTRO}/setup.bash
echo Hello, wait a moment while colcon build our packages
colcon build

# echo 'initializing deck'
# byobu send-keys "roscd" Enter
# byobu send-keys "cd .." Enter
# byobu new-session -d -s $USER

# echo 'roscore'
# byobu rename-window -t $USER:0 'roscore'
# byobu send-keys "roscore" Enter
# sleep 1

# echo 'iauv_commander'
# byobu new-window -t $USER:1 -n 'commander'
# byobu send-keys "roslaunch iauv_commander iauv_commander.launch" Enter

# echo 'tree'
# byobu new-window -t $USER:2 -n 'tree'
# byobu send-keys "roslaunch girona_utils auv_path_controller.launch robot:=girona1000" Enter

# Execute the command passed into this entrypoint
exec "$@"
