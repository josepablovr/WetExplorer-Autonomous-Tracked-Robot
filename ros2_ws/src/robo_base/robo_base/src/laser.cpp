#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "iomanip"

    void poseMessageReceived(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        ROS_INFO("position 0 =: [%f]",scan->ranges[0]);
        ROS_INFO("position 10 =: [%f]",scan->ranges[10]);
        ROS_INFO("position 180 =: [%f]",scan->ranges[180]);
    }
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
    int main(int argc,char **argv)
    {
        ros::init(argc,argv,"subscribe_to_scan");
        ros::NodeHandle n;

        //Create a subscriber object
        ros::Subscriber sub = n.subscribe("/scan",1000, poseMessageReceived);
        ros::Subscriber sub_spettrometri = n.subscribe("/spettrometro_destro",1000, chatterCallback);

        //Let ROS take over
        ros::spin();

        return 0;
    }
