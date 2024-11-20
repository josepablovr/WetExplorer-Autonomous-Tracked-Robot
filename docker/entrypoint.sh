#!/bin/bash

set -e 

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "Provided arguments: $@"

exec $@
