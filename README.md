# Autonomous Mobile Robot for Wetland Monitoring
 Code for the control of the WetMobile including Navigation, localization, object recognition and Sampling Tasks Execution

The project WetGuard aims to monitor the greenhouse gases emissions in wetlands. An autonomous mobile robot is developed to automize the sampling process which requires carrying two heavy sensors and placing a chamber in the ground in multiple points every 2 hours, even in difficult weather conditons.  

Everything runs in a Docker:

### Build the Image
    cd ROS_docker
    sudo docker build -t wetguard .

### Run the Container
    sudo docker run -it --user ros --network=host --ipc=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY -v /home/pablo/ROS_docker/catkin_ws:/catkin_ws -v /dev:/dev --device-cgroup-rule='c *:* rmw' -v ~/mapproxy:/mapproxy --name proto wetguard

### Open another window
    sudo docker exec -it proto bash

### Give permissions to IMU: 
    lsusb
    sudo chmod a+rw /dev/bus/usb/00X/00X
    
### Run Map Server (another terminal)
    sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
URL: http://localhost:8080/wmts/gm_layer/gm_grid/{level}/{x}/{y}.png
