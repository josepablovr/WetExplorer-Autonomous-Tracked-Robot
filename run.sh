#!/bin/bash
export WETEXPLORER_WS=$(pwd)
# Define variables for container and image names
CONTAINER_NAME="cont"
IMAGE_NAME="wetguard"
WORKSPACE_PATH="${WETEXPLORER_WS}/catkin_ws"

# Check if the container is already running
if sudo docker ps --filter "name=${CONTAINER_NAME}" --format "{{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    # Execute bash in the running container
    sudo docker exec -it "${CONTAINER_NAME}" bash
else
    # Run a new container if it's not running
    sudo docker run --rm --name "${CONTAINER_NAME}" \
        -it \
        --user ros \
        --network=host \
        --ipc=host \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        --env DISPLAY \
        -v "${WORKSPACE_PATH}:/catkin_ws" \
        -v /dev:/dev \
        --device-cgroup-rule='c *:* rmw' \
        "${IMAGE_NAME}" /bin/bash
fi
