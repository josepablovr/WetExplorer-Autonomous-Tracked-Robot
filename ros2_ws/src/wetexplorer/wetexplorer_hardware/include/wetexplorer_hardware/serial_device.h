#pragma once

#include <string>

class SerialDevice {
public:
  SerialDevice();
  ~SerialDevice();

  bool isConnected() const;
  int connect(const std::string& port, int baudrate);
  void disconnect();

  int writeLine(const std::string& data);
  std::string readLine(int timeout_ms = 50);
  int getHandle() const { return handle_; }


private:
  int handle_;
};
