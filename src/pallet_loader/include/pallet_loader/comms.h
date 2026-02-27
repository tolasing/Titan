#ifndef PALLET_LOADER_H
#define PALLET_LOADER_H
//#include "imutest/comms.h"
//#include <rclcpp/rclcpp.hpp>  
#include <sstream>
#include <cstdlib>
//#include <sensor_msgs/msg/imu.h>
//#include <sensor_msgs/msg/imu.hpp>
//#include <serial/serial.h>
#include <libserial/SerialPort.h>
#include <cstring>
#include<iostream>

using namespace std;

class Comms
{
    public:

    Comms()=default;
        void setup(const string &serial_device,int32_t baud_rate,int32_t timeout_ms);
        void move_pallet(double hw_command);
        void read_actuator(double& hw_joint);

        bool connected()const{ return serialDriver.IsOpen();}

        private:
        LibSerial::SerialPort serialDriver;
        int timeout_ms=1000;
       

};

#endif
