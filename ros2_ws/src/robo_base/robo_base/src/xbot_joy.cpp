#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/QuaternionStamped.h>

namespace controllo_joypad {

  ros::Subscriber joyInput;
  ros::Publisher motionCommandOutput;
  //ros::Publisher cameraCommandOutput;
  geometry_msgs::Twist twistCommand;

  int axis_speed;
  int axis_steer;
  double axis_speed_dead_zone;
  double axis_steer_dead_zone;

  void calcola_vel(float speed_lineare, float vel_angolare, ros::NodeHandle* n)
  {
    float speed = speed_lineare;
    float angRate = vel_angolare;

    if (std::abs(speed) < axis_speed_dead_zone){
     speed = 0.0f;
    }

    if(std::abs(angRate) < axis_steer_dead_zone){
      angRate = 0.0f;
    }
    
    float val_left = speed - angRate;
    float val_right = speed + angRate;
    float val_max = std::max(val_left, val_right);

    if (val_max > 1.0f){
      val_left = val_left / val_max;
      val_right = val_right / val_max;
    }

  //  inserire moltiplicazione per range 0 - 1000 motore sinistro
  //  inserire moltiplicazione per range 0 - 1000 motore destro

   printf("Left: %f, Right: %f\n", val_left, val_right);
   motionCommandOutput = n->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

   twistCommand.linear.x = val_left;
   twistCommand.linear.x = 0;
   twistCommand.linear.x = 0;

   twistCommand.angular.z = val_right;
   twistCommand.angular.y = 0;
   twistCommand.angular.x = 0;

   motionCommandOutput.publish(twistCommand);
   
  // ************** PUBBLICARE VALORI LEFT e RIGHT in CMD_VEL ************* //

  }
  
 
  void joyCallback(const sensor_msgs::JoyConstPtr joystick) {

    float speed = joystick->axes[axis_speed];
    float angRate = joystick->axes[axis_steer];
    
    //calcola_vel(speed, angRate, &n);

 //   float speed = speed_lineare;
 //   float angRate = vel_angolare;

    if (std::abs(speed) < axis_speed_dead_zone){
     speed = 0.0f;
    }

    if(std::abs(angRate) < axis_steer_dead_zone){
      angRate = 0.0f;
    }
    
    float val_left = speed - angRate;
    float val_right = speed + angRate;
    float val_max = std::max(val_left, val_right);

    if (val_max > 1.0f){
      val_left = val_left / val_max;
      val_right = val_right / val_max;
    }

  //  inserire moltiplicazione per range 0 - 1000 motore sinistro
  //  inserire moltiplicazione per range 0 - 1000 motore destro

   printf("Left: %f, Right: %f\n", val_left, val_right);
   motionCommandOutput = n.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

   twistCommand.linear.x = val_left;
   twistCommand.linear.x = 0;
   twistCommand.linear.x = 0;

   twistCommand.angular.z = val_right;
   twistCommand.angular.y = 0;
   twistCommand.angular.x = 0;

   motionCommandOutput.publish(twistCommand);



  }
  
};

// la stessa funzione può essere utilizzata per ricevere comandi sotto forma di velocità lineare e angolare e pubblicare su cmd_vel i valori separati per ciascun motore


using namespace controllo_joypad;

int main(int argc, char **argv) {

  ros::init(argc, argv, "xbot_joy");
  ros::NodeHandle n;

  joyInput = n.subscribe("joy", 10, &controllo_joypad::joyCallback, &n);

  ros::param::param("~axis_speed", controllo_joypad::axis_speed, 0);
  ros::param::param("~axis_steer", controllo_joypad::axis_steer, 1);
  ros::param::param("~axis_speed_dead_zone", controllo_joypad::axis_speed_dead_zone, 0.1);
  ros::param::param("~axis_steer_dead_zone", controllo_joypad::axis_steer_dead_zone, 0.1);
 

  ros::Rate rate(10.0);

  while(ros::ok()) {
  
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

