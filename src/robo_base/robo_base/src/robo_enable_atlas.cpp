#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <signal.h>
#include <termios.h>

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

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

void mySigintHandler(int sig)
{
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mode");
  ros::NodeHandle n;
   std_msgs::String msg_sys;
  std::string port = "/dev/ttyACM0";
  int mode=0;
 

  n.getParam("/mode/port", port);
  n.getParam("/mode/mode", mode);
  
  ros::Publisher sys_pub = n.advertise<std_msgs::String>("robo_base/mission", 1);
  std::cout << "\nWAITING FOR MISSION" << std::endl;
  while (ros::ok())
{
  string c;   // call your non-blocking input function
  cin >> c;
 if (strcmp(c.c_str(), "start") == 0){
        std::cout << "\nMISSION START" << std::endl;
    msg_sys.data = "START";
    }   

if (strcmp(c.c_str(), "stop") == 0){
        std::cout << "\nMISSION STOP" << std::endl;
    msg_sys.data = "STOP";
    }

if (strcmp(c.c_str(), "quit") == 0){
        std::cout << "\nThe topic is dead!" << std::endl;
 signal(SIGINT, mySigintHandler);
 exit(0);
    }
   
 c.clear();
 sys_pub.publish(msg_sys);
}
 


  return 0;
}
