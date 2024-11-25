FROM ros:noetic-robot

# Update package list
RUN apt-get update

# Install sudo
RUN apt-get install -y sudo



# Install software-properties-common
RUN apt-get install -y software-properties-common

RUN apt-get update && apt-get install -y ros-noetic-urdf && apt-get clean

RUN apt-get update && apt-get install -y ros-noetic-rviz && apt-get clean


# Install libusb
RUN apt-get update && apt-get install -y \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    && apt-get clean

# Install libusb
RUN apt-get update && apt-get install -y \
    ros-noetic-cv-bridge\
    && apt-get clean

    
# Install usbutils for lsusb command
RUN apt-get update && apt-get install -y \
    usbutils \
    && apt-get clean



RUN apt-get update && apt-get install -y \
    ros-noetic-qt-gui\
    ros-noetic-mapviz\
    qtbase5-dev\
    && apt-get clean


RUN apt-get update && apt-get install -y \    
    ros-noetic-mapviz-plugins\
    ros-noetic-tile-map\
    ros-noetic-joint-state-publisher-gui\
    && apt-get clean


RUN apt-get update && apt-get install -y \    
    ros-noetic-joint-state-publisher-gui\
    && apt-get clean

RUN apt-get update && apt-get install -y \    
    ros-noetic-robot-localization\
    && apt-get clean


RUN apt-get update && apt-get install -y \    
    ros-noetic-imu-transformer\
    && apt-get clean    
    
RUN apt-get update && apt-get install -y \    
    ros-noetic-rviz-imu-plugin\
    && apt-get clean    

    
RUN apt-get update && apt-get install -y \    
	curl\
	&& apt-get clean
	
	
RUN apt-get update && apt-get install -y \    
    ros-noetic-rqt-graph\
    ros-noetic-tf\
    ros-noetic-rqt-tf-tree\
    ros-noetic-rqt-plot\
    ros-noetic-xacro\
    && apt-get clean   

RUN apt-get update && apt-get install -y \    
    ros-noetic-gazebo-ros-pkgs\
    && apt-get clean 
    
RUN apt-get update && apt-get install -y \    
    build-essential git cmake libasio-dev\  
    && apt-get clean   
    
   
RUN apt-get update && apt-get install -y \    
    ros-noetic-hector-gazebo-plugins\
    ros-noetic-husky-desktop\
    ros-noetic-husky-simulator\
    ros-noetic-husky-navigation\
    && apt-get clean 
    
    
RUN apt-get update && apt-get install -y \    
    pip\    
    bluez\
    && pip install ds4drv\
    && apt-get clean 
# Update package list again after adding new repository
RUN apt-get update


   
RUN apt-get update && apt-get install -y \    
    ros-noetic-ros-control\
    ros-noetic-ros-controllers\
    && apt-get clean 




RUN apt-get update && apt-get install -y \    
    ros-noetic-plotjuggler-ros\
    netcat\
    nano\
    curl\
    && pip install scipy\
    && apt-get clean 
    
ENV DEBIAN_FRONTEND=noninteractive   
RUN apt-get update
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg\
	&& echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null\
	&&apt-get update\
	&&apt-get install -y ignition-fortress\
    	&&apt-get clean
    
    
RUN apt-get update && apt-get install -y \    
    coinor-libipopt-dev\
    libnlopt-cxx-dev\
    && apt-get clean 
    

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config \
    && chown $USER_UID:$USER_GID /home/$USERNAME/.config

RUN apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*



RUN usermod -aG dialout ${USERNAME} 
COPY /config/.bashrc /home/ros/.bashrc
COPY /config/hosts /etc/hosts
COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]


 


WORKDIR /catkin_ws


