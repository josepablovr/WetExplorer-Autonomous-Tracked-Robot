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
double r_motor, l_motor;

void RoboteqDevice::cmdCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
	std_msgs::String msg;
   	std::stringstream ssvel;
	string response2;
	   
        //Per Debug
    	cout << "Motor 1: " << (int) vel->linear.x << " and Motor 2: " << (int)vel->angular.z << "\n";
        r_motor = vel->linear.x;	
      	l_motor = vel->angular.z;

        if (vel->linear.x > 1000){
        r_motor = 1000;  
        }
        if (vel->linear.x < -1000){
        r_motor = -1000;  
        }
        if (vel->angular.z > 1000){
        l_motor = -1000;  
        }
	
	cout << "- Set Motor1";
	if ((status = this->SetCommand(_GO, 1, r_motor)) != RQ_SUCCESS)
		cout << "Setting motor command 1 failed: " << status << endl;
	else
		cout << "Setting motor command 1 succeeded: " << endl;

	cout << "- Set Motor2";
	if ((status = this->SetCommand(_GO, 2, l_motor)) != RQ_SUCCESS)
		cout << "Setting motor command 2 failed: " << status << endl;
	else
		cout << "Setting motor command 2 succeded: " << endl;

	//Funzionante OK XBOT MIXED MODE 1
	//this->MixedModeMotorMove(vel->linear.x, vel->angular.z, response2); 

}

void RoboteqDevice::cmd_ioCallback(const robo_base::robo_io::ConstPtr& robo)
{ 
    
   if (robo->out_0 == 1){
   if ((status = this->SetCommand(_D1, 1)) != RQ_SUCCESS)
		cout << "Setting D1 OUT failed: " << status << endl;
	else
		cout << "Setting D1 OUT succeded: " << endl;
		   }
   else if (robo->out_0 == 0) {
   if ((status = this->SetCommand(_D0, 1)) != RQ_SUCCESS)
		cout << "Resetting D1 OUT failed: " << status << endl;
	else
		cout << "Resetting D1 OUT succeded: " << endl;
         }

   if (robo->out_1 == 1){
   if ((status = this->SetCommand(_D1, 2)) != RQ_SUCCESS)
		cout << "Setting D2 OUT failed: " << status << endl;
	else
		cout << "Setting D2 OUT succeded: " << endl;
		   }
   else if (robo->out_2 == 0) {
   if ((status = this->SetCommand(_D0, 2)) != RQ_SUCCESS)
		cout << "Resetting D2 OUT failed: " << status << endl;
	else
		cout << "Resetting D2 OUT succeded: " << endl;
         }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robo_base");
  ros::NodeHandle n;

  std::string port = "";
  int sys=0, enc=0, io=0, velocity=0, rate=10, enc_pulse=0, pulse_rev_left=0, pulse_rev_right=0, cmd_vel=1, gear_ratio=1;
  float track=0.8, sprocket=100;

  n.getParam("/robo_base/port", port);
  n.getParam("/robo_base/sys", sys);
  n.getParam("/robo_base/enc", enc);
  n.getParam("/robo_base/io", io);
  n.getParam("/robo_base/velocity", velocity);
  n.getParam("/robo_base/rate", rate);
  n.getParam("/robo_base/enc_pulse", enc_pulse);
  n.getParam("/robo_base/pulse_rev_left", pulse_rev_left);
  n.getParam("/robo_base/pulse_rev_right", pulse_rev_right);
  n.getParam("/robo_base/sprocket", sprocket);
  n.getParam("/robo_base/track", track);
  n.getParam("/robo_base/cmd_vel", cmd_vel);
  n.getParam("/robo_base/gear_ratio", gear_ratio);


 
  //Debug
  //ROS_INFO("%d - %d - %d - %s", sys, enc, io, port.c_str());


  long long unsigned int count = 0; //Frame counter

  RoboteqDevice device;
  int status = device.Connect(port.c_str());
  
  ros::Publisher sys_pub = n.advertise<std_msgs::String>("robo/sys", 1);
  ros::Publisher enc_pub = n.advertise<std_msgs::String>("robo/enc", 1);
  ros::Publisher vel_pub = n.advertise<std_msgs::String>("robo/velocity", 1);
  ros::Publisher io_pub = n.advertise<std_msgs::String>("robo/io", 1);

  
  ros::Subscriber cmd_sub = n.subscribe("robo/cmd_vel", 1, &RoboteqDevice::cmdCallback, &device);
  
  
  ros::Subscriber cmd_io = n.subscribe("robo/io_status", 1, &RoboteqDevice::cmd_ioCallback, &device);

  double current_time =ros::Time::now().toSec();
  double last_time =ros::Time::now().toSec();

  ros::Rate loop_rate(rate);



 while (ros::ok())
  {
 
    std_msgs::String msg_sys, msg_enc, msg_io, msg_vel;
    std:string temp="", volt="", amotor="", enco="", pwm="", aio="", dio="", rpm =""; //Strings that receives data from roboteq device
    std::stringstream ss, ss2, ss3, ssvel;  //Strings used to build topics message
    // printf("\ntrack: %f\n, sprocket: %f\n, pulse_rev: %d\n, enc_pulse: %d\n", track, sprocket, pulse_rev, enc_pulse);


    //System Topic
    if (sys == 1){
      device.GetValue(_T, 0, temp);
      device.GetValue(_V, 0, volt);
      device.GetValue(_A, amotor);
      device.GetValue(_M, pwm);
      device.GetValue(_AI, 0, aio);
      device.GetValue(_S, rpm);
    
    //printf("Debug: %s -- %llu \n", temp.c_str(), count);

      if (temp != "" && volt != "" && amotor != "" && aio != ""){

        std::replace(temp.begin(), temp.end(), ':', ',');
        std::replace(volt.begin(), volt.end(), ':', ',');
        std::replace(amotor.begin(), amotor.end(), ':', ',');
        std::replace(pwm.begin(), pwm.end(), ':', ',');
        std::replace(aio.begin(), aio.end(), ':', ',');
        std::replace(rpm.begin(), rpm.end(), ':', ',');
        ros::Time time = ros::Time::now();
        ss << time << "," << pwm << "," << rpm << "," << temp << "," << volt << "," << amotor << "," << aio << "," << count;
        //msg_sys.header.stamp = ros::Time::now();
        msg_sys.data = ss.str();

        //ROS_INFO("%s", msg_sys.data.vel_max_rpmc_str());

        sys_pub.publish(msg_sys);
      }
  }


    //Encoder Topic
    if (enc == 1){
      current_time =ros::Time::now().toSec();
      device.GetValue(_C, 0, enco);
	   //enco ="15000:16000";

      if (enco != ""){

	    //Velocity 
	    if (velocity == 1){
      //Convert encoder string in long long integer
      std::string vel_enco=enco;
      std::replace(vel_enco.begin(), vel_enco.end(), ':', ' ');

      const char * c_enco = vel_enco.c_str();
      char* pEnd;
      double Encoder_sx, Encoder_dx;
      double Distance_sx, Distance_dx, Distance_prev_sx, Distance_prev_dx, Vel_sx, Vel_dx;
      float Vel_linear, w;

      Encoder_sx = std::strtod (c_enco,&pEnd);
      Encoder_dx = std::strtod (pEnd,&pEnd);
      //printf("E1 %lf, E2 %lf", Encoder_sx, Encoder_dx);

      Encoder_sx=Encoder_sx/pulse_rev_left/gear_ratio;
      Encoder_dx=Encoder_dx/pulse_rev_right/gear_ratio;
      Distance_sx = (Encoder_sx)*sprocket;
      Distance_dx = (Encoder_dx)*sprocket;

      last_time =ros::Time::now().toSec();
      Vel_sx = ((Distance_prev_sx - Distance_sx) / (last_time - current_time))/1000; // m / sec 
      Vel_dx = ((Distance_prev_dx - Distance_dx) / (last_time - current_time))/1000;  

      Distance_prev_sx = Distance_sx;
      Distance_prev_dx = Distance_dx;

      Vel_linear = (Vel_dx + Vel_sx)/2;
      w = (Vel_dx - Vel_sx)/track;

      ros::Time time2 = ros::Time::now();

      //Publish velocity topic
      ssvel << time2 << "," << l_motor << "," << r_motor << "," << Distance_sx << "," << Distance_dx << "," << Vel_sx << "," << Vel_dx << "," << Vel_linear << "," << w;
      msg_vel.data = ssvel.str();
      vel_pub.publish(msg_vel);
      }

	    ros::Time time3 = ros::Time::now();

	    std::replace(enco.begin(), enco.end(), ':', ',');
	    
	    //Publish enco topic
	    ss2 << time3 << "," << enco << "," << count;
	    msg_enc.data = ss2.str();
	    enc_pub.publish(msg_enc);
      }
    }


   //IO Topic
   if (io == 1){
      device.GetValue(_AI, 0, aio);
      device.GetValue(_DIN, 0, dio);

      if (aio != "" && dio != ""){
	    std::replace(aio.begin(), aio.end(), ':', ',');
	    std::replace(dio.begin(), dio.end(), ':', ',');

	    ros::Time time4 = ros::Time::now();
	    ss3 << time4 << "," << aio << "," << dio << "," << count;
	    //msg_enc->header.stamp = ros::Time::now();
	    msg_io.data = ss3.str();

	    //ROS_INFO("%s", msg_enc.data.c_str());

	    io_pub.publish(msg_io);
      }
    }

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}