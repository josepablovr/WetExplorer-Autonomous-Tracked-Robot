#include "ros/ros.h"
#include "std_msgs/String.h"
#include "iomanip"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <sstream>
#include <iostream>
#include <string>         // std::string
#include <cstddef>        // std::size_t



using namespace std;

    void status_rtk_topic(const diagnostic_msgs::KeyValue::ConstPtr& status)
    {
   
     if (status->value[3] == '1'){
     std::cout << "1 - RTK FIX\n"; 
     }
     else if (status->value[4] == '1'){
     std::cout << "2 - RTK FLOAT\n"; 
     }
     else std::cout << "0 - NO RTK\n"; 

    }


    int main(int argc,char **argv)
    {
        ros::init(argc,argv,"check_rtk");
        ros::NodeHandle n;

        //Create a subscriber object
        ros::Subscriber sub = n.subscribe("/status", 1000, status_rtk_topic);
      
        //Let ROS take over
        ros::spin();

        return 0;
    }
