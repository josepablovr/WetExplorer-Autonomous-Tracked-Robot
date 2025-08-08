#ifndef __RoboteqDevice_H_
#define __RoboteqDevice_H_

#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "robo_io/msg/robo_io.hpp"

using namespace std;

string ReplaceString(string source, string find, string replacement);
void sleepms(int milliseconds);

class RoboteqDevice
{
private:
    int device_fd;
	int fd0;
	int handle;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<robo_io::msg::RoboIO>::SharedPtr cmd_io_sub_;

protected:
    void InitPort();

	int Write(string str);
	int ReadAll(string &str);

	int IssueCommand(string commandType, string command, string args, int waitms, string &response, bool isplusminus = false);
	int IssueCommand(string commandType, string command, int waitms, string &response, bool isplusminus = false);

public:
    bool IsConnected();
    int Connect(string port);
    void Disconnect();

    int SetConfig(int configItem, int index, int value);
    int SetConfig(int configItem, int value);

    int SetCommand(int commandItem, int index, int value);
    int SetCommand(int commandItem, int value);
    int SetCommand(int commandItem);

    int GetConfig(int configItem, int index, int &result);
    int GetConfig(int configItem, int &result);

    int GetValue(int operatingItem, int index, std::string &result);
    int GetValue(int operatingItem, std::string &result);

    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr vel);
    void cmd_ioCallback(const robo_io::msg::RoboIO::SharedPtr robo);
    int MixedModeMotorMove(float throttle, float steering, string &response);

    RoboteqDevice();
    ~RoboteqDevice();
};

#endif
