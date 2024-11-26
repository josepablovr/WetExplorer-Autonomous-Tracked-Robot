#!/bin/bash

# Define workspace and Docker variables
export WETEXPLORER_WS=$(pwd)
CONTAINER_NAME="cont"
IMAGE_NAME="wetexplorer"
WORKSPACE_PATH="${WETEXPLORER_WS}/ros2_ws"


# Check if a container named "cont" is already running
if sudo docker ps --filter "name=cont" --format "{{.Names}}" | grep -q "^cont$"; then
    # If the container is running, execute bash in it
    sudo docker exec -it cont bash
else
    # Otherwise, run a new container
    sudo docker run --rm --name cont -it --user ros --network=host --ipc=host \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY \
        -v "${WORKSPACE_PATH}":/ros2_ws \
        -v /dev:/dev --device-cgroup-rule='c *:* rmw' "${IMAGE_NAME}" /bin/bash
fi


