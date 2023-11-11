#!/usr/bin/env bash

# Setup ROS for Local Development
source /opt/ros/noetic/setup.bash
source ~/minifrc/2023Zebramites/zebmite_ws/devel/setup.bash
export ROS_MASTER_URI="http://$(hostname -I | awk 'BEGIN { FS="[ ]" } ; { print $1 }'):5802"
export ROS_IP="$(hostname -I | awk 'BEGIN { FS="[ ]" } ; { print $1 }')"
export ROSLAUNCH_SSH_UNKNOWN=1

exec "$@"
