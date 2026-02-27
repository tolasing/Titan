#include "battery_status/comms.h"
//#include <rclcpp/rclcpp.hpp>  
#include <sstream>
#include <cstdlib>
//#include <sensor_msgs/msg/imu.h>
//#include <sensor_msgs/msg/imu.hpp>
//#include <serial/serial.h>
#include <libserial/SerialPort.h>
#include <cstring>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>

using namespace std;
LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}                                                

void Comms::setup(const std::string &serial_device,int32_t baud_rate,int32_t timeout_ms)
{
    serialDriver.Open(serial_device);
    serialDriver.SetBaudRate(convert_baud_rate(baud_rate));
    //serial::Timeout to=serial::Timeout::simpleTimeout(timeout_ms);
    //serialDriver.setTimeout(to);
    //serialDriver.open();
    if(serialDriver.IsOpen())
    {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        RCLCPP_INFO(rclcpp::get_logger("imu_node"), "Serial driver is open!");
    }
}


void Comms::read_battery(float& voltage)
{
    serialDriver.FlushIOBuffers();
 // Requesting the IMU data from the Arduino
serialDriver.Write("e\r");
std::string response = "";
serialDriver.ReadLine(response, '\n', timeout_ms);
voltage = std::atof(response.c_str());
    
    }
/*
int main()
{
    Comms arduino_;
    arduino_.setup("/dev/ttyUSB0", 9600, 10000);
    float  voltage=0.0;

    // Assuming the Arduino is already connected and configured

    // Continuously read and display the YPR values
    while (true)
    {
        arduino_.read_battery(voltage);
        std::cout << "voltage: " << voltage <<std::endl;
        // Add delay if needed
    }

    return 0;
}

*/
