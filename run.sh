
#!/bin/bash

# Define workspace and Docker variables
export WETEXPLORER_WS=$(pwd)
CONTAINER_NAME="cont"
IMAGE_NAME="wetguard"
WORKSPACE_PATH="${WETEXPLORER_WS}/catkin_ws"

# Check if the container is already running
if sudo docker ps --filter "name=${CONTAINER_NAME}" --format "{{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    echo "Container '${CONTAINER_NAME}' is already running."
    # Execute bash in the running container
    sudo docker exec -it "${CONTAINER_NAME}" bash
    exit 0
fi

# Determine and define the display environment
if [ "$(echo $XDG_SESSION_TYPE)" = "x11" ]; then
    xhost +local:root
    DISPLAY_ENV="--env DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw"
    echo "Using X11 display environment."
else
    xhost +SI:localuser:root
    
    DISPLAY_ENV="-e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
    -v $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:/tmp/$WAYLAND_DISPLAY \
    -e XDG_RUNTIME_DIR=/tmp \
    -e QT_QPA_PLATFORM=wayland"
    echo "Using Wayland display environment."
fi

# Run a new container if it's not running
echo "Starting a new container with the following display environment:"
echo "$DISPLAY_ENV"

sudo docker run --rm --name "${CONTAINER_NAME}" \
    -it \
    --user ros \
    --network=host \
    --ipc=host \
    $DISPLAY_ENV \
    -v "${WORKSPACE_PATH}:/catkin_ws" \
    -v /dev:/dev \
    --device-cgroup-rule='c *:* rmw' \
    "${IMAGE_NAME}" /bin/bash
