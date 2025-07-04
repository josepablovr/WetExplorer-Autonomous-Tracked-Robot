#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <sstream>
#include <iostream>
#include <stdio.h>
#include <string.h>

#include "robo_base/RoboteqDevice.h"
#include "robo_base/ErrorCodes.h"
#include "robo_base/Constants.h"
#include "robo_io/msg/robo_io.hpp" // Note: robo_io::msg::RoboIO

using namespace std;
using namespace std::chrono_literals;

string response = "";
int status;
double r_motor, l_motor;

void RoboteqDevice::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr vel)
{
    std_msgs::msg::String msg;
    std::stringstream ssvel;
    string response2;

    // Debug
    cout << "Motor 1: " << (int)vel->linear.x << " and Motor 2: " << (int)vel->angular.z << "\n";
    r_motor = vel->linear.x;
    l_motor = vel->angular.z;

    if (vel->linear.x > 1000)
        r_motor = 1000;

    if (vel->linear.x < -1000)
        r_motor = -1000;

    if (vel->angular.z > 1000)
        l_motor = -1000;

    cout << "- Set Motor1";
    if ((status = this->SetCommand(_GO, 1, r_motor)) != RQ_SUCCESS)
        cout << "Setting motor command 1 failed: " << status << endl;
    else
        cout << "Setting motor command 1 succeeded." << endl;

    cout << "- Set Motor2";
    if ((status = this->SetCommand(_GO, 2, l_motor)) != RQ_SUCCESS)
        cout << "Setting motor command 2 failed: " << status << endl;
    else
        cout << "Setting motor command 2 succeeded." << endl;
}

void RoboteqDevice::cmd_ioCallback(const robo_io::msg::RoboIO::SharedPtr robo)
{
    if (robo->out_0)
    {
        if ((status = this->SetCommand(_D1, 1)) != RQ_SUCCESS)
            cout << "Setting D1 OUT failed: " << status << endl;
        else
            cout << "Setting D1 OUT succeeded." << endl;
    }
    else
    {
        if ((status = this->SetCommand(_D0, 1)) != RQ_SUCCESS)
            cout << "Resetting D1 OUT failed: " << status << endl;
        else
            cout << "Resetting D1 OUT succeeded." << endl;
    }

    if (robo->out_1)
    {
        if ((status = this->SetCommand(_D1, 2)) != RQ_SUCCESS)
            cout << "Setting D2 OUT failed: " << status << endl;
        else
            cout << "Setting D2 OUT succeeded." << endl;
    }
    else
    {
        if ((status = this->SetCommand(_D0, 2)) != RQ_SUCCESS)
            cout << "Resetting D2 OUT failed: " << status << endl;
        else
            cout << "Resetting D2 OUT succeeded." << endl;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robo_base");

    std::string port;
    int sys = 0, enc = 0, io = 0, velocity = 0, rate = 10, enc_pulse = 0, pulse_rev_left = 0, pulse_rev_right = 0, cmd_vel = 1, gear_ratio = 1;
    float track = 0.8, sprocket = 100;

    node->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    node->declare_parameter<int>("sys", 1);
    node->declare_parameter<int>("enc", 0);
    node->declare_parameter<int>("io", 0);
    node->declare_parameter<int>("velocity", 0);
    node->declare_parameter<int>("rate", 20);
    node->declare_parameter<int>("enc_pulse", 512);
    node->declare_parameter<int>("pulse_rev_left", 4096);
    node->declare_parameter<int>("pulse_rev_right", 4096);
    node->declare_parameter<float>("sprocket", 100.0);
    node->declare_parameter<float>("track", 0.8);
    node->declare_parameter<int>("cmd_vel", 1);
    node->declare_parameter<int>("gear_ratio", 1);

    node->get_parameter("port", port);
    node->get_parameter("sys", sys);
    node->get_parameter("enc", enc);
    node->get_parameter("io", io);
    node->get_parameter("velocity", velocity);
    node->get_parameter("rate", rate);
    node->get_parameter("enc_pulse", enc_pulse);
    node->get_parameter("pulse_rev_left", pulse_rev_left);
    node->get_parameter("pulse_rev_right", pulse_rev_right);
    node->get_parameter("sprocket", sprocket);
    node->get_parameter("track", track);
    node->get_parameter("cmd_vel", cmd_vel);
    node->get_parameter("gear_ratio", gear_ratio);



     long long unsigned int count = 0; //Frame counter

    RoboteqDevice device;
    int status = device.Connect(port);
    if (status != 0)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to connect to Roboteq device.");
        return 1;
    }

    auto sys_pub = node->create_publisher<std_msgs::msg::String>("robo/sys", 10);
    auto enc_pub = node->create_publisher<std_msgs::msg::String>("robo/enc", 10);
    auto vel_pub = node->create_publisher<std_msgs::msg::String>("robo/velocity", 10);
    auto io_pub = node->create_publisher<std_msgs::msg::String>("robo/io", 10);

    auto cmd_sub = node->create_subscription<geometry_msgs::msg::Twist>("robo/cmd_vel", 1, std::bind(&RoboteqDevice::cmdCallback, &device, std::placeholders::_1));
    auto cmd_io_sub = node->create_subscription<robo_io::msg::RoboIO>("robo/io_status", 10, std::bind(&RoboteqDevice::cmd_ioCallback, &device, std::placeholders::_1));

    rclcpp::Rate loop_rate(rate);

    double current_time = node->now().seconds();
    double last_time = node->now().seconds();
    RCLCPP_INFO(rclcpp::get_logger("robo_base"), "Starting Communication");

    while (rclcpp::ok())
    {
        

        std_msgs::msg::String msg_sys, msg_enc, msg_io, msg_vel;
        std:string temp="", volt="", amotor="", enco="", pwm="", aio="", dio="", rpm =""; //Strings that receives data from roboteq device
        std::stringstream ss, ss2, ss3, ssvel;  //Strings used to build topics message
        // printf("\ntrack: %f\n, sprocket: %f\n, pulse_rev: %d\n, enc_pulse: %d\n", track, sprocket, pulse_rev, enc_pulse);


        //System Topic
        if (sys == 1){
            //RCLCPP_INFO(rclcpp::get_logger("robo_base"), "Processing Sys Message");
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
                auto time = node->now();
               
                std::string data = std::to_string(time.seconds()) + "," +
                   pwm + "," +
                   rpm + "," +
                   temp + "," +
                   volt + "," +
                   amotor + "," +
                   aio + "," +
                   std::to_string(count);
                ss << data;

                msg_sys.data = ss.str();
                //RCLCPP_INFO(rclcpp::get_logger("robo_base"), "ss: %s", ss.str().c_str());


                sys_pub->publish(msg_sys);
                }
            }

        // Encoder Topic
        if (enc == 1)
        {
            current_time = node->now().seconds();
            device.GetValue(_C, 0, enco);

            if (!enco.empty())
            {
                // Velocity
                if (velocity == 1)
                {
                    // Convert encoder string into long long integer
                    std::string vel_enco = enco;
                    std::replace(vel_enco.begin(), vel_enco.end(), ':', ' ');

                    const char *c_enco = vel_enco.c_str();
                    char *pEnd;
                    double Encoder_sx, Encoder_dx;
                    double Distance_sx, Distance_dx, Distance_prev_sx, Distance_prev_dx, Vel_sx, Vel_dx;
                    float Vel_linear, w;

                    Encoder_sx = std::strtod(c_enco, &pEnd);
                    Encoder_dx = std::strtod(pEnd, &pEnd);

                    Encoder_sx = Encoder_sx / pulse_rev_left / gear_ratio;
                    Encoder_dx = Encoder_dx / pulse_rev_right / gear_ratio;
                    Distance_sx = (Encoder_sx)*sprocket;
                    Distance_dx = (Encoder_dx)*sprocket;

                    last_time = node->now().seconds();
                    Vel_sx = ((Distance_prev_sx - Distance_sx) / (last_time - current_time)) / 1000; // m / sec
                    Vel_dx = ((Distance_prev_dx - Distance_dx) / (last_time - current_time)) / 1000;

                    Distance_prev_sx = Distance_sx;
                    Distance_prev_dx = Distance_dx;

                    Vel_linear = (Vel_dx + Vel_sx) / 2;
                    w = (Vel_dx - Vel_sx) / track;

                    auto time2 = node->now();

                    // Publish velocity topic
                    std::string data3 = std::to_string(time2.seconds()) + "," + std::to_string(l_motor) + "," + std::to_string(r_motor) + "," +
                    std::to_string(Distance_sx) + "," + std::to_string(Distance_dx) + "," +
                    std::to_string(Vel_sx) + "," + std::to_string(Vel_dx) + "," +
                    std::to_string(Vel_linear) + "," + std::to_string(w);
                    ssvel << data3;

                    msg_vel.data = ssvel.str();
                    vel_pub->publish(msg_vel);
                }

                auto time3 = node->now();

                std::replace(enco.begin(), enco.end(), ':', ',');

                // Publish encoder topic
                std::string data1 = std::to_string(time3.seconds()) + "," + enco + "," + std::to_string(count);
                ss2 << data1;

                msg_enc.data = ss2.str();
                enc_pub->publish(msg_enc);
            }
        }

        // IO Topic
        if (io == 1)
        {
            device.GetValue(_AI, 0, aio);
            device.GetValue(_DIN, 0, dio);

            if (!aio.empty() && !dio.empty())
            {
                std::replace(aio.begin(), aio.end(), ':', ',');
                std::replace(dio.begin(), dio.end(), ':', ',');

                auto time4 = node->now();
                std::string data2 = std::to_string(time4.seconds()) + "," + aio + "," + dio + "," + std::to_string(count);
                ss3 << data2;
                msg_io.data = ss3.str();

                io_pub->publish(msg_io);
            }
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
        ++count;
    }


    rclcpp::shutdown();
    return 0;
}
