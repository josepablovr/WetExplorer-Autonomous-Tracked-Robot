# Use ros2_humble-image as the base
FROM ros:humble-perception

# Update package list
RUN apt-get update

# Install sudo
RUN apt-get install -y sudo

# Install software-properties-common
RUN apt-get install -y software-properties-common

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

# Install Python packages via pip
RUN pip install ds4drv scipy

# Install Gazebo dependencies
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y ignition-fortress \
    && apt-get clean

# Install other dependencies
RUN apt-get update && apt-get install -y \
    coinor-libipopt-dev \
    libnlopt-cxx-dev \
    && apt-get clean





# Install ROS Dependecencies:


RUN apt-get update && apt-get install -y \
    ros-humble-urdf \
    ros-humble-xacro \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim\
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
    ros-humble-ros-gz-image \
    ros-humble-rqt \
    ros-humble-rqt-graph \
    && apt-get clean

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


# Set working directory
WORKDIR /ros2_ws

