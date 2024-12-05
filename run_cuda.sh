#!/bin/bash

# Check if a container named "cont" is already running
if sudo docker ps --filter "name=cont" --format "{{.Names}}" | grep -q "^cont$"; then
    # If the container is running, execute bash in it
    sudo docker exec -it cont bash
else
    # Otherwise, run a new container
    sudo docker run --rm --name cont -it --gpus all --user ros --network=host --ipc=host \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY \
        -v /home/pablo/humble/WetExplorer-Autonomous-Tracked-Robot/ros2_ws:/ros2_ws \
        -v /dev:/dev --device-cgroup-rule='c *:* rmw' wetexplorer /bin/bash
fi
