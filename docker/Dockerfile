# Use ros2_humble-image as the base
FROM ros:humble AS deps



WORKDIR /ros2_ws
SHELL ["/bin/bash", "-c"]



RUN sudo rm /etc/apt/sources.list.d/ros2-latest.list
RUN sudo rm /usr/share/keyrings/ros2-latest-archive-keyring.gpg

RUN sudo apt update && sudo apt install -y curl ca-certificates
RUN set -e \
    && ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -m1 '"tag_name"' | cut -d '"' -f4)" \
    && curl -fsSL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install -y /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb
RUN sudo apt-get update
RUN sudo apt-get install -y ros-humble-nav2-bringup

# Update package list
RUN apt-get update



# Install sudo
RUN apt-get install -y sudo \
    curl

# Install software-properties-common
RUN apt-get install -y software-properties-common

# Install ROS Dependecencies:
RUN apt-get update && apt-get install -y --fix-missing\
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
    ros-humble-rviz2 \   
    ros-humble-rqt \
    ros-humble-rqt-graph \
    && apt-get clean

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
RUN apt-get update && pip3 install ds4drv scipy



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


RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
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
    && rm -rf /var/lib/apt/lists/*
    

RUN sudo add-apt-repository ppa:deadsnakes/ppa
RUN apt update && apt install -y --no-install-recommends \
    software-properties-common \
    build-essential \
    wget \
    curl \
    libssl-dev \
    zlib1g-dev \
    libbz2-dev \
    libreadline-dev \
    libsqlite3-dev \
    libffi-dev \
    python3-pip \
    python3-venv \
    python3.8 \
    python3.8-venv \
    python3.8-dev \
    && rm -rf /var/lib/apt/lists/*



# Update package list
RUN apt-get update




# Install OverlapPredator Requirements
RUN pip3 install --no-cache-dir \   
    numpy==1.26.4 \ 
    torchaudio \
    nibabel \
    tqdm \
    easydict \
    scipy \
    coloredlog \
    PyYAML \
    scikit_learn \
    tensorboardX \
    vtk_visualizer \
    h5py \
    coloredlogs \
    gitpython

RUN pip install --no-cache-dir \
    wcwidth \
    pure-eval \
    ptyprocess \
    fastjsonschema \
    dash-table \
    dash-html-components \
    dash-core-components \
    addict \
    widgetsnbextension \
    werkzeug \
    traitlets \
    tenacity \
    retrying \
    pyquaternion \
    prompt_toolkit \
    pexpect \
    parso \
    nest-asyncio \
    jupyterlab-widgets \
    itsdangerous \
    executing \
    exceptiongroup \
    configargparse \   
    asttokens \
    stack_data \
    plotly \
    matplotlib-inline \
    jupyter-core \
    jedi \    
    comm \
    ipython \    
    Werkzeug \
    nbformat \
    ipywidgets

RUN pip3 install --no-cache-dir --no-deps \
    dash \
    Flask \
    open3d==0.18.0

RUN apt-get update


#ros-humble-imu-tools
#ros-humble-plotjuggler-ros
#ros-humble-navigation2
#ros-humble-robot-localization

RUN apt-get install -y \
    ros-humble-imu-tools \
    ros-humble-plotjuggler-ros \
    ros-humble-navigation2 \
    ros-humble-robot-localization \
    ros-humble-nav2-bringup \
    ros-humble-nav2-costmap-2d \
    && rm -rf /var/lib/apt/lists/*

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
#COPY entrypoint.sh /entrypoint.sh



# Set the entrypoint and default command
#ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]




