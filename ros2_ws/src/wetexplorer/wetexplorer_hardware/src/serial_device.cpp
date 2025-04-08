#include "wetexplorer_hardware/serial_device.h"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <linux/serial.h>
#include <sys/ioctl.h> 

#define SERIAL_INVALID_HANDLE -1
#define SERIAL_SUCCESS 0
#define SERIAL_ERROR -1

SerialDevice::SerialDevice() : handle_(SERIAL_INVALID_HANDLE) {}

SerialDevice::~SerialDevice() {
  disconnect();
}

bool SerialDevice::isConnected() const {
  return handle_ != SERIAL_INVALID_HANDLE;
}

int SerialDevice::connect(const std::string& port, int baudrate) {
  if (isConnected()) {
    RCLCPP_WARN(rclcpp::get_logger("SerialDevice"), "Already connected, disconnecting first.");
    disconnect();
  }

  handle_ = open(port.c_str(), O_RDWR | O_NOCTTY);
  if (handle_ == SERIAL_INVALID_HANDLE) {
    RCLCPP_ERROR(rclcpp::get_logger("SerialDevice"), "Failed to open serial port.");
    return SERIAL_ERROR;
  } 
  int modem_bits = 0;
  ioctl(handle_, TIOCMGET, &modem_bits);
  modem_bits &= ~TIOCM_DTR;
  ioctl(handle_, TIOCMSET, &modem_bits);
  usleep(10000);
  modem_bits |= TIOCM_DTR;
  ioctl(handle_, TIOCMSET, &modem_bits);
  usleep(10000);

  RCLCPP_INFO(rclcpp::get_logger("SerialDevice"), "Opened port '%s' successfully.", port.c_str());

  termios tty{};
  tcgetattr(handle_, &tty);

  cfsetospeed(&tty, baudrate);
  cfsetispeed(&tty, baudrate);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  tcflush(handle_, TCIFLUSH);
  tcsetattr(handle_, TCSANOW, &tty);

  return SERIAL_SUCCESS;
}

void SerialDevice::disconnect() {
  if (isConnected()) {
    close(handle_);
    handle_ = SERIAL_INVALID_HANDLE;
  }
}

int SerialDevice::writeLine(const std::string& data) {
  if (!isConnected()) return SERIAL_ERROR;
  return write(handle_, data.c_str(), data.length());
}


std::string SerialDevice::readLine(int timeout_ms) {
  if (!isConnected())
    RCLCPP_ERROR(rclcpp::get_logger("SerialDevice"), "Failed to open serial port.");
  if (!isConnected()) return "";

  std::string result;
  char c;
  auto start = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(timeout_ms)) {
    int n = read(handle_, &c, 1);
    if (n > 0) {
      if (c == '\r') break;
      if (c != '\n')
        result += c;
    }
    

    
      
  }

  return result;
}
