#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <stdlib.h> 
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
     ros::init(argc, argv, "robo_plan");
     ros::NodeHandle nh;
     char c;
     int i = 0;
        
            //Declares the message to be sent
            geometry_msgs::Twist msg;
     int forward0=0, forward1=0, forward2=0, forward3=0, steer0=0, steer1=0, steer2=0, steer3=0, stop0=0, stop1=0, stop2=0, stop3=0;
   
    
     nh.getParam("/robo_plan/forward0", forward0);
     nh.getParam("/robo_plan/forward1", forward1);
     nh.getParam("/robo_plan/forward2", forward2);
     nh.getParam("/robo_plan/forward3", forward3);

     nh.getParam("/robo_plan/steer0", steer0);
     nh.getParam("/robo_plan/steer1", steer1);
     nh.getParam("/robo_plan/steer2", steer2);
     nh.getParam("/robo_plan/steer3", steer3);

     nh.getParam("/robo_plan/stop0", stop0);
     nh.getParam("/robo_plan/stop1", stop1);
     nh.getParam("/robo_plan/stop2", stop2);
     nh.getParam("/robo_plan/stop3", stop3);
 


     //Creates the publisher, and tells it to publish
     //to the husky/cmd_vel topic, with a queue size of 100
     ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("robo/cmd_vel", 100);

     //Sets up the random number generator
     srand(time(0));

     //Sets the loop to publish at a rate of 10Hz
     ros::Rate rate(10);

if(stop0 != 0){

ROS_INFO("Forward 00: %d", forward0);

       while( i < forward0  ) { // prima era 380

           //Random x value between -2 and 2
           msg.linear.x=100;
           //Random y value between -3 and 3
           msg.angular.z=100;
           //Publish the message
           pub.publish(msg);
           ROS_INFO("Proccesing [FORWARD0]: %d / %d", i, forward0);
          //Delays untill it is time to send another message
          rate.sleep();
 i++;
         }

i = 0;

ROS_INFO("Steer 00: %d", steer0);

 while( i < steer0  ) {
        
           //Random x value between -2 and 2
           msg.linear.x=100;
           //Random y value between -3 and 3
           msg.angular.z=-100;
           //Publish the message
           pub.publish(msg);
           ROS_INFO("Proccesing  [STEER0]: %d / %d", i, steer0);
          //Delays untill it is time to send another message
          rate.sleep();
 i++;
         }


} else {

           msg.linear.x=0;
           msg.angular.z=0;
           pub.publish(msg);
           ROS_INFO("END");
           exit(0);
}

i = 0;

if (stop1 !=0){

ROS_INFO("Forward 01: %d", forward1);

       while( i < forward1  ) {
        
           //Random x value between -2 and 2
           msg.linear.x=100;
           //Random y value between -3 and 3
           msg.angular.z=100;
           //Publish the message
           pub.publish(msg);
	   ROS_INFO("Proccesing  [FORWARD1]: %d / %d", i, forward1);
          //Delays untill it is time to send another message
           rate.sleep();
 i++;
         }


i = 0;

ROS_INFO("Steer 01: %d", steer1);

 while( i < steer1  ) {
        
           //Random x value between -2 and 2
           msg.linear.x=100;
           //Random y value between -3 and 3
           msg.angular.z=-100;
           //Publish the message
           pub.publish(msg);
           ROS_INFO("Proccesing  [STEER1]: %d / %d", i, steer1);
          //Delays untill it is time to send another message
          rate.sleep();
 i++;
         }

} else {

           msg.linear.x=0;
           msg.angular.z=0;
           pub.publish(msg);
           ROS_INFO("END");
           exit(0);
}


i = 0;

if (stop2 != 0){

ROS_INFO("Forward 02: %d", forward2);

       while( i < forward2  ) { // prima era 380
        
           //Random x value between -2 and 2
           msg.linear.x=100;
           //Random y value between -3 and 3
           msg.angular.z=100;
           //Publish the message
           pub.publish(msg);
           ROS_INFO("Proccesing  [FORWARD2]: %d / %d", i, forward2);
          //Delays untill it is time to send another message
          rate.sleep();
 i++;
         }

 i = 0;

ROS_INFO("Steer 02: %d", steer2);

 while( i < steer2  ) {
        
           //Random x value between -2 and 2
           msg.linear.x=100;
           //Random y value between -3 and 3
           msg.angular.z=-100;
           //Publish the message
           pub.publish(msg);
           ROS_INFO("Proccesing  [STEER2]: %d / %d", i, steer2);
          //Delays untill it is time to send another message
          rate.sleep();
 i++;
         }

} else {

           msg.linear.x=0;
           msg.angular.z=0;
           pub.publish(msg);
           ROS_INFO("END");
           exit(0);
}

i = 0;


if (stop3 != 0){

ROS_INFO("Forward 03: %d", forward3);

       while( i < forward3  ) { // prima era 380
        
            //Random x value between -2 and 2
           msg.linear.x=100;
           //Random y value between -3 and 3
           msg.angular.z=100;
           //Publish the message
           pub.publish(msg);
           ROS_INFO("Proccesing  [FORWARD3]: %d / %d", i, forward3);
          //Delays untill it is time to send another message
          rate.sleep();
 i++;
         }

 i = 0;

ROS_INFO("Steer 03: %d", steer3);

 while( i < steer3  ) {
        
           //Random x value between -2 and 2
           msg.linear.x=100;
           //Random y value between -3 and 3
           msg.angular.z=-100;
           //Publish the message
           pub.publish(msg);
           ROS_INFO("Proccesing  [STEER3]: %d / %d", i, steer3);
          //Delays untill it is time to send another message
          rate.sleep();
 i++;
         }

} else {

           msg.linear.x=0;
           msg.angular.z=0;
           pub.publish(msg);
           ROS_INFO("END");
           exit(0);
}

i = 0;

           msg.linear.x=0;
           msg.angular.z=0;
           pub.publish(msg);
           ROS_INFO("END");
           exit(0);

}

