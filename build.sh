#!/bin/bash

export WETEXPLORER_WS=$(pwd)

# Ensure the WETEXPLORER_WS variable is set
if [ -z "${WETEXPLORER_WS}" ]; then
    echo "Error: WETEXPLORER_WS environment variable is not set."
    exit 1
fi

# Navigate to the docker directory
DOCKER_DIR="${WETEXPLORER_WS}/docker"
if [ ! -d "${DOCKER_DIR}" ]; then
    echo "Error: Docker directory ${DOCKER_DIR} does not exist."
    exit 1
fi

echo "Navigating to ${DOCKER_DIR}..."
cd "${DOCKER_DIR}"

# Build the Docker image
IMAGE_NAME="wetguard"
echo "Building Docker image ${IMAGE_NAME}..."
sudo docker build -t "${IMAGE_NAME}" .

if [ $? -eq 0 ]; then
    echo "Docker image ${IMAGE_NAME} built successfully."
else
    echo "Error: Docker image build failed."
    exit 1
fi

# Navigate back and run the run.sh script
echo "Returning to the original directory and running run.sh..."
cd - > /dev/null
./run.sh
