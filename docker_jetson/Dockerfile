# Use ros2_humble-image as the base
# Use NVIDIA PyTorch image as the base
FROM isaac_ros_dev-aarch64


# Install dependencies
RUN apt-get update \
    && apt-get -y --quiet --no-install-recommends install \
    gcc \
    git \
    python3 \
    python3-pip




# Install basic System Requirements
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
    lsb-release \
    gnupg \
    && apt-get clean


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
    && pip3 install ds4drv \
    && apt-get clean




# Install RealSense SDK
RUN apt-get update && apt-get install -y \
    ros-humble-librealsense2*\
    && rm -rf /var/lib/apt/lists/*


# Dummy installation of torch
RUN apt-get update && pip3 install --no-cache-dir \   
    torch==2.3.0  \ 
    torchaudio==2.3.0\ 
    torchvision==0.18.0
 
# YOLO Requirements

RUN apt-get update && pip3 install --no-cache-dir \   
    opencv-python>=4.8.1.78 \   
    typing-extensions>=4.4.0 \          
    lap==0.5.12 \
    ultralytics==8.3.26 --no-deps \
    && pip3 install --no-cache-dir super-gradients --no-deps 
  




# Install OverlapPredator Requirements

RUN apt-get update && pip3 install --no-cache-dir \   
    numpy==1.26.4 \     
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
    gitpython \
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
    ipywidgets \
    && pip3 install --no-cache-dir --no-deps \
    dash \
    Flask \
    open3d==0.18.0
            


# Fix Torch dependent libraries
RUN apt-get update && pip3 uninstall -y \ 
    torch \
    torchaudio \
    torchvision

# Install PyTorch
RUN apt-get update && export TORCH_INSTALL=https://pypi.jetson-ai-lab.dev/jp6/cu124/+f/5fe/ee5f5d1a75229/torch-2.3.0-cp310-cp310-linux_aarch64.whl#sha256=5feee5f5d1a75229eb5290c6fcdc534658dd8b99e55e4a49e8f7c509c313d52d \
&& python3 -m pip install --no-cache $TORCH_INSTALL

# Install Torchaudio
RUN apt-get update && export TORCH_INSTALL=https://pypi.jetson-ai-lab.dev/jp6/cu124/+f/0aa/a066463c02b4a/torchaudio-2.3.0+952ea74-cp310-cp310-linux_aarch64.whl#sha256=0aaa066463c02b4a0939dc881c48bd1d92c5b42b76d339282e2656e10aa15041 \
    && python3 -m pip install --no-cache $TORCH_INSTALL

# Install Torchvision
RUN apt-get update && export TORCH_INSTALL=https://pypi.jetson-ai-lab.dev/jp6/cu124/+f/988/cb71323efff87/torchvision-0.18.0a0+6043bc2-cp310-cp310-linux_aarch64.whl#sha256=988cb71323efff87bcf503c9c4e94a8e19a351c96146d0806f194f48da52aaa2 \
    && python3 -m pip install --no-cache $TORCH_INSTALL

# Install OpenCV
RUN apt-get update && export TORCH_INSTALL=https://pypi.jetson-ai-lab.dev/jp6/cu126/+f/bea/77ef3f9923c70/opencv_contrib_python-4.10.0+6b45caa-cp310-cp310-linux_aarch64.whl#sha256=bea77ef3f9923c708f6f38e85b615ae9b4fc775de9b765f3b42355d7ef5e8e0c \
    && python3 -m pip install --no-cache $TORCH_INSTALL





# RUN apt-get update

# # Create a non-root user
# ARG USERNAME=ros
# ARG USER_UID=1000
# ARG USER_GID=1000
# RUN groupadd --gid $USER_GID $USERNAME \
#     && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
#     && mkdir /home/$USERNAME/.config \
#     && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# # Give the new user sudo privileges
# RUN apt-get update \
#     && apt-get install -y sudo \
#     && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
#     && chmod 0440 /etc/sudoers.d/$USERNAME \
#     && rm -rf /var/lib/apt/lists/*

# # Add user to dialout group
# RUN usermod -aG dialout ${USERNAME}

# # Copy config files
# COPY /config/.bashrc /home/ros/.bashrc
# COPY /config/hosts /etc/hosts
#COPY entrypoint.sh /entrypoint.shcolco



# Set the entrypoint and default command
#ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]




