#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "robo_base/RoboteqDevice.h"
#include "robo_base/ErrorCodes.h"
#include "robo_base/Constants.h"

#include <iostream>
#include <string>

using namespace std;

class RoboBaseModeNode : public rclcpp::Node {
public:
    RoboBaseModeNode() : Node("mode") {
        // Declare and get parameters
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("mode", 0);

        this->get_parameter("port", port_);
        this->get_parameter("mode", mode_);

        RCLCPP_INFO(this->get_logger(), "Port: %s, Mode: %d", port_.c_str(), mode_);

        // Connect to the Roboteq device
        status_ = device_.Connect(port_.c_str());
        if (status_ == 0) {
            configureDevice();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to device. Please try a different port.");
        }
    }

private:
    void configureDevice() {
        RCLCPP_INFO(this->get_logger(), "CONFIGURATION MODE FOR ROBO_BASE - ROBODYNE");

        if (mode_ == 0) {
            RCLCPP_INFO(this->get_logger(), "SETTING THE CONFIGURATION TO RUN ROS");

            if (device_.SetCommand(_R, 0) != RQ_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to change firmware configuration for ROS.");
            } else {
                RCLCPP_INFO(this->get_logger(), "Firmware configuration changed for ROS.");
            }

            if (device_.SetConfig(_MXMD, 0) != RQ_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to enable system for ROS.");
            } else {
                RCLCPP_INFO(this->get_logger(), "System enabled for ROS.");
            }

            RCLCPP_INFO(this->get_logger(), "ROS ENABLED!");
        } else if (mode_ == 1) {
            RCLCPP_INFO(this->get_logger(), "SETTING THE CONFIGURATION TO RUN THE RC TRANSMITTER");

            if (device_.SetCommand(_R, 2) != RQ_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to change firmware configuration for RC Transmitter.");
            } else {
                RCLCPP_INFO(this->get_logger(), "Firmware configuration changed for RC Transmitter.");
            }

            if (device_.SetConfig(_MXMD, 2) != RQ_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to enable system for RC Transmitter.");
            } else {
                RCLCPP_INFO(this->get_logger(), "System enabled for RC Transmitter.");
            }

            RCLCPP_INFO(this->get_logger(), "RC TRANSMITTER ENABLED!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid mode specified. Use 0 for ROS or 1 for RC Transmitter.");
        }
    }

    std::string port_;
    int mode_;
    int status_;
    RoboteqDevice device_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoboBaseModeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
