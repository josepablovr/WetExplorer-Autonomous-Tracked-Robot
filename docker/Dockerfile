# Use ros2_humble-image as the base
FROM ros:humble AS deps

# Update package list
RUN apt-get update



# Install sudo
RUN apt-get install -y sudo

# Install software-properties-common
RUN apt-get install -y software-properties-common

WORKDIR /ros2_ws
SHELL ["/bin/bash", "-c"]

# Install dependencies
RUN apt-get update \
    && apt-get -y --quiet --no-install-recommends install \
    gcc \
    git \
    python3 \
    python3-pip

RUN pip3 install opencv-python>=4.8.1.78
RUN pip3 install typing-extensions>=4.4.0
RUN pip3 install ultralytics==8.3.26
RUN pip3 install super-gradients==3.7.1
RUN pip3 install lap==0.5.12    

RUN pip3 install sphinx==8.0.0 sphinx-rtd-theme==3.0.0







# Install system dependencies (libusb, usbutils, etc.)
RUN apt-get update && apt-get install -y \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    usbutils \
    build-essential \
    git \
    cmake \
    libasio-dev \
    curl \
    bluez \   
    pip\
    && apt-get clean



RUN pip3 install sphinx==8.0.0 sphinx-rtd-theme==3.0.0

# Install Python packages via pip
RUN pip3 install ds4drv scipy


# Install ROS Dependecencies:
RUN apt-get update && apt-get install -y \
    ros-humble-urdf \
    ros-humble-xacro \  
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-twist-stamper \
    ros-humble-twist-mux \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ign-ros2-control \
    ros-humble-rviz2 \   
    ros-humble-rqt \
    ros-humble-rqt-graph \
    && apt-get clean

# Update and install necessary dependencies
RUN apt-get update && apt-get install -y \
    curl \
    lsb-release \
    gnupg \
    && rm -rf /var/lib/apt/lists/*


# Add the OSRF Gazebo repository and import the key
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update package lists and install gz-harmonic
RUN apt-get update && apt-get install -y \
    gz-harmonic

# Install ROS-GZ Harmonic
RUN apt-get install -y \
    ros-humble-ros-gzharmonic \
    && rm -rf /var/lib/apt/lists/*


# Install RealSense SDK
RUN apt-get update && apt-get install -y \
    ros-humble-librealsense2*\
    && rm -rf /var/lib/apt/lists/*


# Install and initialize rosdep, then install dependencies
RUN apt-get update \
    && apt-get -y install python3-rosdep \
    && rosdep update \
    && rosdep install -i --from-paths . --ignore-src -r -y

RUN apt-get update && apt-get install -y \
    ros-humble-launch-pytest \
    python3-tqdm \
    python3-requests \
    nvidia-cuda-toolkit \
    nvidia-cuda-dev \
    && rm -rf /var/lib/apt/lists/*
    




# Update package list
RUN apt-get update

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config \
    && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Give the new user sudo privileges
RUN apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Add user to dialout group
RUN usermod -aG dialout ${USERNAME}

# Copy config files
COPY /config/.bashrc /home/ros/.bashrc
COPY /config/hosts /etc/hosts
#COPY entrypoint.sh /entrypoint.shcolco



# Set the entrypoint and default command
#ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]




