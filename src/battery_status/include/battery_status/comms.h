#ifndef IMUTEST_COMMS_H
#define IMUTEST_COMMS_H
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
        //void sendEmpytMsg();
        //void readEncoderValues(int &val_1,int &val_2);
        void read_battery(float &voltage);
        bool connected()const{ return serialDriver.IsOpen();}

        //std::string sendMsg(const std::string &msg_to_send,bool print_output=false);
        float voltage=0.0;
        private:

        LibSerial::SerialPort serialDriver;
        int timeout_ms=10000;
       

};

#endif
