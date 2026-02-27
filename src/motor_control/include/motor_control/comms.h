#ifndef MOTOR_CONTROL_COMMS_H
#define MOTOR_CONTROL_COMMS_H

#include <serial/serial.h>
#include <cstring>

using namespace std;

class Comms
{
    public:
    Comms(){}
    
    Comms(const string &serial_device,int32_t baud_rate,int32_t timeout_ms)

    :serialDriver(serial_device,baud_rate,serial::Timeout::simpleTimeout(timeout_ms))
    {}
        void setup(const string &serial_device,int32_t baud_rate,int32_t timeout_ms);
        void sendEmpytMsg();
        void readEncoderValues(int &val_1,int &val_2);
        void setMotorValues(double val_1,double val_2);
        void setPidValues(float k_p,float k_d,float k_i,float k_o); 

        bool connected()const{ return serialDriver.isOpen();}

        std::string sendMsg(const std::string &msg_to_send,bool print_output=false);

        private:

        serial::Serial serialDriver;

};

#endif