#!/bin/bash

# Navigate to the ros2_ws directory
cd docker || exit 1

# Build the Docker image using the Dockerfile in the current directory
sudo docker build -t wetexplorer .

# Return to the original directory and run the container
cd - || exit 1
./run.sh
