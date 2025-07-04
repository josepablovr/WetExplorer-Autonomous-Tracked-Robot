#include <iostream>
#include <string>
#include <sstream>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <cstring>
#include "rclcpp/rclcpp.hpp"

#include "robo_base/RoboteqDevice.h"
#include "robo_base/ErrorCodes.h"
#include "robo_base/Constants.h"

using namespace std;

#define BUFFER_SIZE 1024
#define MISSING_VALUE -1024

RoboteqDevice::RoboteqDevice()
{
    handle = RQ_INVALID_HANDLE;
}

RoboteqDevice::~RoboteqDevice()
{
    Disconnect();
}

bool RoboteqDevice::IsConnected()
{
    return handle != RQ_INVALID_HANDLE;
}

int RoboteqDevice::Connect(string port)
{
    if (IsConnected())
    {
        RCLCPP_WARN(rclcpp::get_logger("RoboteqDevice"), "Device is connected, attempting to disconnect.");
        Disconnect();
    }

    RCLCPP_INFO(rclcpp::get_logger("RoboteqDevice"), "Opening port: '%s'...", port.c_str());
    handle = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (handle == RQ_INVALID_HANDLE)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RoboteqDevice"), "Failed to open port.");
        return RQ_ERR_OPEN_PORT;
    }

    RCLCPP_INFO(rclcpp::get_logger("RoboteqDevice"), "Port opened successfully.");
    fcntl(handle, F_SETFL, O_APPEND | O_NONBLOCK & ~FNDELAY);

    RCLCPP_INFO(rclcpp::get_logger("RoboteqDevice"), "Initializing port...");
    InitPort();
    RCLCPP_INFO(rclcpp::get_logger("RoboteqDevice"), "Port initialized.");

    int status;
    string response;
    RCLCPP_INFO(rclcpp::get_logger("RoboteqDevice"), "Detecting device version...");
    for (int z = 0; z < 5; z++)
    {
        status = IssueCommand("?", "FID", 50, response);
        if (status == RQ_SUCCESS)
            break;
    }

    if (status != RQ_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RoboteqDevice"), "Failed to detect device version (status: %d).", status);
        Disconnect();
        return RQ_UNRECOGNIZED_DEVICE;
    }

    RCLCPP_INFO(rclcpp::get_logger("RoboteqDevice"), "Device version detected: %s.", response.substr(8, 4).c_str());
    return RQ_SUCCESS;
}

void RoboteqDevice::Disconnect()
{
    if (IsConnected())
        close(handle);

    handle = RQ_INVALID_HANDLE;
}

void RoboteqDevice::InitPort()
{
    if (!IsConnected())
        return;

    int BAUDRATE = B115200;
    struct termios newtio;
    tcgetattr(handle, &newtio);

    cfsetospeed(&newtio, (speed_t)BAUDRATE);
    cfsetispeed(&newtio, (speed_t)BAUDRATE);

    newtio.c_iflag = IGNBRK;
    newtio.c_lflag = 0;
    newtio.c_oflag = 0;
    newtio.c_cflag |= (CLOCAL | CREAD);
    newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY);
    newtio.c_cflag &= ~CSIZE;
    newtio.c_cflag |= CS8;
    newtio.c_cflag &= ~PARENB;
    newtio.c_cflag &= ~PARODD;
    newtio.c_cflag &= ~CSTOPB;

    newtio.c_cc[VMIN] = 0;
    newtio.c_cc[VTIME] = 100;

    tcflush(handle, TCIFLUSH);
    tcsetattr(handle, TCSANOW, &newtio);
}

int RoboteqDevice::Write(string str)
{
    if (!IsConnected())
        return RQ_ERR_NOT_CONNECTED;

    int countSent = write(handle, str.c_str(), str.length());
    if (countSent < 0)
        return RQ_ERR_TRANSMIT_FAILED;

    return RQ_SUCCESS;
}

int RoboteqDevice::ReadAll(string &str)
{
    if (!IsConnected())
        return RQ_ERR_NOT_CONNECTED;

    char buf[BUFFER_SIZE + 1] = "";
    str.clear();

    int countRcv;
    while ((countRcv = read(handle, buf, BUFFER_SIZE)) > 0)
    {
        str.append(buf, countRcv);
        if (countRcv < BUFFER_SIZE)
            break;
    }

    if (countRcv < 0)
    {
        if (errno == EAGAIN)
            return RQ_ERR_SERIAL_IO;
        else
            return RQ_ERR_SERIAL_RECEIVE;
    }

    return RQ_SUCCESS;
}

int RoboteqDevice::MixedModeMotorMove(float throttle, float steering, string &response)
{
    string args = to_string(static_cast<int>(throttle)) + " " + to_string(static_cast<int>(steering));
    RCLCPP_INFO(rclcpp::get_logger("RoboteqDevice"), "Sending Mixed Mode Motor Move command: %s", args.c_str());

    int status = Write("!M " + args + "\r");
    if (status != RQ_SUCCESS)
        return status;

    usleep(7500);

    string read;
    status = ReadAll(read);
    if (status != RQ_SUCCESS)
        return status;

    if (read.length() < 2)
        return RQ_INVALID_RESPONSE;

    response = read.substr(read.length() - 2, 1);
    return RQ_SUCCESS;
}



int RoboteqDevice::IssueCommand(string commandType, string command, string args, int waitms, string &response, bool isplusminus)
{
	int status;
	string read;
	response = "";

	//DEBUG

	/*std_msgs::String msgDebug2;
   	std::stringstream sdebug2;
    	sdebug2 << args;
	msgDebug2.data = sdebug2.str();
	ROS_INFO("ISSUECOMMAND: [%s]", msgDebug2.data.c_str());*/

	if(args == "")
		status = Write(commandType + command + "\r");
	else
		status = Write(commandType + command + " " + args + "\r");

	if(status != RQ_SUCCESS)
		return status;

	usleep(waitms * 1000l);

	status = ReadAll(read);
	if(status != RQ_SUCCESS)
		return status;

	if(isplusminus)
	{
		if(read.length() < 2)
			return RQ_INVALID_RESPONSE;

		response = read.substr(read.length() - 2, 1);
		return RQ_SUCCESS;
	}


	string::size_type pos = read.rfind(command + "=");
	if(pos == string::npos)
		return RQ_INVALID_RESPONSE;

	pos += command.length() + 1;

	string::size_type carriage = read.find("\r", pos);
	if(carriage == string::npos)
		return RQ_INVALID_RESPONSE;

	response = read.substr(pos, carriage - pos);

	return RQ_SUCCESS;
}
int RoboteqDevice::IssueCommand(string commandType, string command, int waitms, string &response, bool isplusminus)
{
	return IssueCommand(commandType, command, "", waitms, response, isplusminus);
}


int RoboteqDevice::SetConfig(int configItem, int index, int value)
{
    std::string response;
    char command[10];
    char args[50];

    if (configItem < 0 || configItem > 255)
        return RQ_INVALID_CONFIG_ITEM;

    sprintf(command, "$%02X", configItem);
    sprintf(args, "%i %i", index, value);
    if (index == MISSING_VALUE)
    {
        sprintf(args, "%i", value);
        index = 0;
    }

    if (index < 0)
        return RQ_INDEX_OUT_RANGE;

    int status = IssueCommand("^", command, args, 7.5, response, true);
    if (status != RQ_SUCCESS)
        return status;
    if (response != "+")
        return RQ_SET_CONFIG_FAILED;

    return RQ_SUCCESS;
}

int RoboteqDevice::SetConfig(int configItem, int value)
{
    return SetConfig(configItem, MISSING_VALUE, value);
}

int RoboteqDevice::SetCommand(int commandItem, int index, int value)
{
    std::string response;
    char command[10];
    char args[50];

    if (commandItem < 0 || commandItem > 255)
        return RQ_INVALID_COMMAND_ITEM;

    sprintf(command, "$%02X", commandItem);
    sprintf(args, "%i %i", index, value);

    // Debug logging
    std::stringstream sdebug;
    sdebug << args;
    RCLCPP_INFO(rclcpp::get_logger("RoboteqDevice"), "motor[%d]: [%s]", index, sdebug.str().c_str());

    if (index == MISSING_VALUE)
    {
        if (value != MISSING_VALUE)
            sprintf(args, "%i", value);
        index = 0;
    }

    if (index < 0)
        return RQ_INDEX_OUT_RANGE;

    int status = IssueCommand("!", command, args, 8, response, true);
    if (status != RQ_SUCCESS)
        return status;
    if (response != "+")
        return RQ_SET_COMMAND_FAILED;

    return RQ_SUCCESS;
}

int RoboteqDevice::SetCommand(int commandItem, int value)
{
    return SetCommand(commandItem, MISSING_VALUE, value);
}

int RoboteqDevice::SetCommand(int commandItem)
{
    return SetCommand(commandItem, MISSING_VALUE, MISSING_VALUE);
}

int RoboteqDevice::GetConfig(int configItem, int index, int &result)
{
    std::string response;
    char command[10];
    char args[50];

    if (configItem < 0 || configItem > 255)
        return RQ_INVALID_CONFIG_ITEM;

    if (index < 0)
        return RQ_INDEX_OUT_RANGE;

    sprintf(command, "$%02X", configItem);
    sprintf(args, "%i", index);

    int status = IssueCommand("~", command, args, 8, response);
    if (status != RQ_SUCCESS)
        return status;

    std::istringstream iss(response);
    iss >> result;

    if (iss.fail())
        return RQ_GET_CONFIG_FAILED;

    return RQ_SUCCESS;
}

int RoboteqDevice::GetConfig(int configItem, int &result)
{
    return GetConfig(configItem, 0, result);
}

int RoboteqDevice::GetValue(int operatingItem, int index, std::string &result)
{
    std::string response;
    char command[10];
    char args[50];

    if (operatingItem < 0 || operatingItem > 255)
        return RQ_INVALID_OPER_ITEM;

    if (index < 0)
        return RQ_INDEX_OUT_RANGE;

    sprintf(command, "$%02X", operatingItem);
    sprintf(args, "%i", index);

    int status = IssueCommand("?", command, args, 3, response);
    if (status != RQ_SUCCESS)
        return status;

    std::istringstream iss(response);
    iss >> result;

    if (iss.fail())
        return RQ_GET_VALUE_FAILED;

    return RQ_SUCCESS;
}

int RoboteqDevice::GetValue(int operatingItem, std::string &result)
{
    return GetValue(operatingItem, 0, result);
}

std::string ReplaceString(std::string source, std::string find, std::string replacement)
{
    std::string::size_type pos = 0;
    while ((pos = source.find(find, pos)) != std::string::npos)
    {
        source.replace(pos, find.size(), replacement);
        pos++;
    }

    return source;
}

void sleepms(int milliseconds)
{
    usleep(milliseconds * 1000);
}