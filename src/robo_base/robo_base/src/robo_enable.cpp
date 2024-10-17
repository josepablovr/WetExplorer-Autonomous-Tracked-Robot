#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include <sstream>
#include <iostream>
#include <stdio.h>
#include <string.h>

#include "../include/robo_base/RoboteqDevice.h"
#include "../include/robo_base/ErrorCodes.h"
#include "../include/robo_base/Constants.h"
#include "robo_base/robo_io.h"

string response = "";
int status;
using namespace std;



int main(int argc, char **argv)
{
  ros::init(argc, argv, "mode");
  ros::NodeHandle n;

  std::string port = "/dev/ttyACM0";
  int mode=0;
 

  n.getParam("/mode/port", port);
  n.getParam("/mode/mode", mode);


 
  //Debug
  //ROS_INFO("%d - %d - %d - %s", sys, enc, io, port.c_str());


  long long unsigned int count = 0; //Frame counter

  RoboteqDevice device;
  int status = device.Connect(port.c_str());
  
if (status == 0){
printf("\nCONFIGURATION MODE FOR ROBO_BASE - ROBODYNE\n");
printf("--------------------------_MXMD------------------\n");
if (mode==0){
printf("\nSETTING THE CONFIGURATION TO RUN ROS\n");
printf("--------------------------------------------\n");
		
		if ((status = device.SetCommand(_R, 0)) != RQ_SUCCESS)
			cout << "Change firmware configuration - failed --> " << status << endl;}

		else
			cout << "Change firmware configuration - returned --> " << status << endl; 
             
		
		if ((status = device.SetConfig(_MXMD, 0)) != RQ_SUCCESS)
			cout << "System enabled for ROS - failed --> " << status << endl;
		else
			cout << "System enabled for ROS - returned --> " << status << endl;

printf("\nROS ENABLED!\n");
printf("--------------------------------------------\n");
		exit(0);
}
else if(mode ==1) {
printf("\nSETTING THE CONFIGURATION TO RUN THE RC TRANSMITTER\n");
printf("--------------------------------------------\n");
		
		if ((status = device.SetCommand(_R, 2)) != RQ_SUCCESS)
			cout << "Change firmware configuration - failed --> " << status << endl;
		else
			cout << "Change firmware configuration - returned --> " << status << endl;
 
             
		_MXMD
		if ((status = device.SetConfig(_MXMD, 2)) != RQ_SUCCESS)
			cout << "System enabled for RC Transmitter - failed --> " << status << endl;
		else
			cout << "System enabled for Rc Transmitter - returned --> " << status << endl;

printf("\nRC TRANSMITTER ENABLED!\n");
printf("--------------------------------------------\n");
		
	exit(0);
} 
}_MXMD
else if (status == 1){

printf("\n\nPLEASE TRY TO SET ANOTHER COMMUNICATION PORT!\n\n");
	exit(0);
}

  return 0;
}
