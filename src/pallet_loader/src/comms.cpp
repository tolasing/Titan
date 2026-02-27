#include "pallet_loader/comms.h"
//#include <rclcpp/rclcpp.hpp>  
#include <sstream>
#include <cstdlib>
//#include <sensor_msgs/msg/imu.h>
//#include <sensor_msgs/msg/imu.hpp>
#include <serial/serial.h>
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
    serialDriver.Open("/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_55838323835351110262-if00");
    serialDriver.SetBaudRate(convert_baud_rate(baud_rate));
    //serial::Timeout to=serial::Timeout::simpleTimeout(timeout_ms);
    //serialDriver.setTimeout(to);
    //serialDriver.open();
    if(serialDriver.IsOpen())
    {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        RCLCPP_INFO(rclcpp::get_logger("Pallet_loader"), "Serial driver is open!");
    }
}

void Comms::move_pallet(double val_1)
{
    serialDriver.FlushIOBuffers();
       switch (static_cast<int>(val_1))
    {
        case 1:
            serialDriver.Write("u\r");
            break;
        case 0:
            serialDriver.Write("d\r");
            break;
        case 3:
            serialDriver.Write("c\r");
            break;
        default:
            // Handle any other unexpected values if necessary
            break;
    }
}

void Comms::read_actuator(double& hw_joint)
{
  serialDriver.FlushIOBuffers();
  serialDriver.Write("s\r");
  serialDriver.SetRTS(true);
  std::string response = "";
  serialDriver.ReadLine(response,'\n',10000);
  //serialDriver.Read(response,5,1000);
  hw_joint=std::atof(response.c_str());
    
}
/*
void Comms::read_imu_values(double& accel_x,double& accel_y,double& accel_z, double& gyro_x)
{
    serialDriver.FlushIOBuffers();
 // Requesting the IMU data from the Arduino
serialDriver.Write("e\r");
std::string response = "";
serialDriver.ReadLine(response, '\n', timeout_ms);

    size_t delimiterPos = response.find(" ");
    std::string axtoken = response.substr(0, delimiterPos);
    response.erase(0, delimiterPos + 1);

    delimiterPos = response.find(" ");
    std::string aytoken = response.substr(0, delimiterPos);
    response.erase(0, delimiterPos + 1);

    delimiterPos = response.find(" ");
    std::string aztoken = response.substr(0, delimiterPos);
    response.erase(0, delimiterPos + 1);

    delimiterPos = response.find(" ");
    std::string gxtoken = response;


    // Convert the tokens to double values
    accel_x = std::atof(axtoken.c_str());
    accel_y = std::atof(aytoken.c_str());
    accel_z = std::atof(aztoken.c_str());
    gyro_x = std::atof(gxtoken.c_str());
    
    }*/