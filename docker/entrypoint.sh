#!/bin/bash

set -e 

source /opt/ros/noetic/setup.bash
#source /catkin_ws/devel/setup.bash
source /usr/share/gazebo-11/setup.bash
echo "Provided arguments: $@"

exec $@
