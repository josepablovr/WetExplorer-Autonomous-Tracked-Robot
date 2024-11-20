# Use ros2_humble-image as the base
FROM ros:humble-perception

# Update package list
RUN apt-get update

# Install sudo
RUN apt-get install -y sudo

# Install software-properties-common
RUN apt-get install -y software-properties-common

# Install ROS 2 Humble packages
# RUN apt-get update && apt-get install -y \
#     ros-humble-urdf \
#     ros-humble-rviz2 \
#     ros-humble-cv-bridge \
#     ros-humble-qt-gui \
#     ros-humble-mapviz \
#     ros-humble-mapviz-plugins \
#     ros-humble-tf2 \
#     ros-humble-rqt-graph \
#     ros-humble-rqt-tf-tree \
#     ros-humble-rqt-plot \
#     ros-humble-xacro \
#     ros-humble-gazebo-ros-pkgs \
#     ros-humble-ros-control \
#     ros-humble-ros-controllers \
#     ros-humble-plotjuggler-ros \
#     ros-humble-imu-transformer \
#     ros-humble-rviz-imu-plugin \
#     ros-humble-joint-state-publisher-gui \
#     ros-humble-robot-localization \
#     ros-humble-hector-gazebo-plugins \
#     ros-humble-husky-desktop \
#     ros-humble-husky-simulator \
#     ros-humble-husky-navigation \
#     ros-humble-ros-base \
#     && apt-get clean

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
COPY entrypoint.sh /entrypoint.shcolco

# Set the entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]

# Set working directory
WORKDIR /ros2_ws

