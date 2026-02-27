#include "motor_control/comms.h"
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>

void Comms::setup(const std::string &serial_device,int32_t baud_rate,int32_t timeout_ms)
{
    serialDriver.setPort(serial_device);
    serialDriver.setBaudrate(baud_rate);
    serial::Timeout to=serial::Timeout::simpleTimeout(timeout_ms);
    serialDriver.setTimeout(to);
    serialDriver.open();

   /* try
    {
        serialDriver.open();
    catch(serial::IOException &e)
    {
        RCLCPP_INFO(rclcpp::get_logger("serialDriver"),"the error is %s",e.what());
        throw e;
    }*/

}

void Comms::sendEmpytMsg()
{
    std::string response=sendMsg("\r");
}

void Comms::readEncoderValues(int &val_1,int &val_2 )
{
    static int write_count=0;
    std::string response=sendMsg("e\r");
    std::string delimiter=" ";
    size_t del_pos=response.find(delimiter);
    std::string token_1=response.substr(0,del_pos);
    std::string token_2=response.substr(del_pos+delimiter.length());
    val_1=std::atoi(token_1.c_str());
    val_2=std::atoi(token_2.c_str());
    /*
     if(write_count %10==0)
       {
       RCLCPP_INFO(rclcpp::get_logger("encoder values command"),"%d %d",val_1,val_2);
       }
       write_count++; 
    */


}

void Comms::setMotorValues(double val_1,double val_2)
{
    //static int counter=0;
    std::stringstream ss;
   // int v1=static_cast<int>(val_1);
    //int v2=static_cast<int>(val_2);
    ss<<"m "<<val_1<<" "<<val_2<<"\r";
    sendMsg(ss.str(),false);

    //if(counter %200==0)
    //{
   // RCLCPP_INFO(rclcpp::get_logger("set motor values"),"%f,%f",val_1,val_2);
    //}
   // std::string line=serialDriver.readline();
   // RCLCPP_INFO(rclcpp::get_logger("response"),"%s",line.c_str());
    //counter++;
}

void Comms::setPidValues(float k_p,float k_d,float k_i,float k_o)
{
    std::stringstream ss;
    ss<<"u "<<k_p<<":"<<k_d<<":"<<k_i<<":"<<k_o<<"\r";
    sendMsg(ss.str());
}

std::string Comms::sendMsg(const std::string &msg_to_send,bool print_output)
{
     //RCLCPP_INFO(rclcpp::get_logger("send message"), "msg_to_send=%s", msg_to_send.c_str());
    serialDriver.write(msg_to_send);
    std::string response=serialDriver.readline();

    if(print_output)
    {
      
    }
    return response;
}
