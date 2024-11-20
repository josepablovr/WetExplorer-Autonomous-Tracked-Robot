#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "robo_base/robo_io.h"

class TeleopRobo
{
public:
  TeleopRobo();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_, rotation_;
  double l_scale_, a_scale_, s_rotation_;
  double l_motor, r_motor, throttle, steering;
  ros::Publisher vel_pub_, io_state;
  ros::Subscriber joy_sub_;

};


TeleopRobo::TeleopRobo():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("axis_rotation", rotation_, rotation_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  nh_.param("scale_rotation", s_rotation_, s_rotation_);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robo/cmd_vel", 1000);
  io_state = nh_.advertise<robo_base::robo_io>("/robo/io_status", 1000);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1000, &TeleopRobo::joyCallback, this);
 
}

void TeleopRobo::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  robo_base::robo_io robo;
  throttle = -a_scale_*joy->axes[linear_];
  steering = l_scale_*joy->axes[angular_];
  
  if (throttle >= 200) {
  throttle = 200;
  }
    if (throttle <= -200) {
  throttle = -200;
  }
  
  
  if (steering >= 200) {
  steering = 200;
  }
  
    if (steering <= -200) {
  steering = -200;
  }
  
  int button0 = joy->buttons[0];
  int button1 = joy->buttons[1];
  int button2 = joy->buttons[2];
  int button3 = joy->buttons[3];
    
  l_motor = throttle + steering;
  r_motor = throttle - steering;   
 
  robo.out_0 = button0;
  robo.out_1 = button1;
  robo.out_2 = button2;
  robo.out_3 = button3;
  robo.pwm_sx = throttle;
  robo.pwm_dx = steering;

  twist.linear.x = l_motor;
  twist.angular.z = r_motor;
  //twist.angular.z = s_rotation_*joy->axes[rotation_];
  vel_pub_.publish(twist);
  io_state.publish(robo);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_robo");
  TeleopRobo teleop_robo;

  ros::Rate loop_rate(10);

  ros::spin();
}
