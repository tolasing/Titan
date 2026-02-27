#include "motor_control/motor_interface.hpp"
#include <chrono>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace motor_control
{
    MotorControl::MotorControl(): log_(rclcpp::get_logger("motor_control"))
    {}
    CallbackReturn MotorControl::on_init(const hardware_interface::HardwareInfo &info)
    {
        if(hardware_interface::SystemInterface::on_init(info) !=CallbackReturn::SUCCESS)
        {return CallbackReturn::ERROR;}
        RCLCPP_INFO(rclcpp::get_logger("on init method"),": initialising..");


        //configure the wheels and arduino.
        cfg_.back_left_wheel_name=info_.hardware_parameters["back_left_wheel_name"];
        cfg_.back_right_wheel_name=info_.hardware_parameters["back_right_wheel_name"];
        cfg_.loop_rate=stof(info_.hardware_parameters["loop_rate"]);
        cfg_.baud_rate=stoi(info_.hardware_parameters["baud_rate"]);
        cfg_.device=info_.hardware_parameters["device"];
        cfg_.timeout=stoi(info_.hardware_parameters["timeout"]);
        cfg_.enc_counts_per_rev=stoi(info_.hardware_parameters["enc_counts_per_rev"]);
        //setup the wheels
        l_wheel_back.setup(cfg_.back_left_wheel_name,cfg_.enc_counts_per_rev);
        r_wheel_back.setup(cfg_.back_right_wheel_name,cfg_.enc_counts_per_rev);
       
        //setup the arduino
        arduino_.setup(cfg_.device,cfg_.baud_rate,cfg_.timeout);
        RCLCPP_INFO(rclcpp::get_logger("arduino"),"%s,%d",cfg_.device.c_str(),cfg_.baud_rate);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn MotorControl::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("motor_control"),"configuring....");
        return CallbackReturn::SUCCESS;}
     
    std::vector<hardware_interface::StateInterface> MotorControl::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        //state interface for the left wheel;
        state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_back.name,hardware_interface::HW_IF_VELOCITY,&l_wheel_back.vel));

        state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_back.name,hardware_interface::HW_IF_POSITION,&l_wheel_back.pos));

        //state interface for the right wheel
        state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_back.name,hardware_interface::HW_IF_VELOCITY,&r_wheel_back.vel));

        state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_back.name,hardware_interface::HW_IF_POSITION,&r_wheel_back.pos));

         return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> MotorControl::export_command_interfaces()
    {
        //static int command_count=0;
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(hardware_interface::CommandInterface(l_wheel_back.name,hardware_interface::HW_IF_VELOCITY,&l_wheel_back.cmd));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(r_wheel_back.name,hardware_interface::HW_IF_VELOCITY,&r_wheel_back.cmd));
        /*
        if(command_count%200==0)
        {
        RCLCPP_INFO(rclcpp::get_logger("command interface"),"%f",l_wheel_.cmd);
        }
        command_count++;
        */
        return command_interfaces;


    }

    CallbackReturn MotorControl::on_activate(const rclcpp_lifecycle::State &/*previous_state*/)
    {
      RCLCPP_INFO(rclcpp::get_logger("motor_control"),"activating this now ...");
       arduino_.sendEmpytMsg();
      //arduino_.setPidValues(9,7,0,100);
      //arduino.setPidValues(14,7,0,100);
      //arduino_.setPidValues(50,40,0,100);

      return CallbackReturn::SUCCESS;
    }

    CallbackReturn MotorControl::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
    
        RCLCPP_INFO(rclcpp::get_logger("motor_Control"),"successfully deactivated");
        return CallbackReturn::SUCCESS;

    }

    hardware_interface::return_type MotorControl::read(const rclcpp::Time &/*time*/,const rclcpp::Duration &/*period*/)
    {
        auto new_time=std::chrono::system_clock::now();
        std::chrono::duration<double> diff=new_time-time_;
        double deltaSeconds=diff.count();
        time_=new_time;

        if(!arduino_.connected())
        {return hardware_interface::return_type::ERROR;}

        arduino_.readEncoderValues(l_wheel_back.enc,r_wheel_back.enc);

        double prev_pos=l_wheel_back.pos;
        l_wheel_back.pos=l_wheel_back.calcEncAngle();
        l_wheel_back.vel=(l_wheel_back.pos-prev_pos)/deltaSeconds;

        prev_pos=r_wheel_back.pos;
        r_wheel_back.pos=r_wheel_back.calcEncAngle();
        r_wheel_back.vel=(r_wheel_back.pos-prev_pos)/deltaSeconds;

        return hardware_interface::return_type::OK;
    }
    
    hardware_interface::return_type MotorControl::write(const rclcpp::Time & /*time*/,const rclcpp::Duration &/*period*/)
    {
        static int write_count=0;
      if(!arduino_.connected())
      {return hardware_interface::return_type::ERROR;}
        
       arduino_.setMotorValues(l_wheel_back.cmd / l_wheel_back.rads_per_count / cfg_.loop_rate,
       r_wheel_back.cmd / r_wheel_back.rads_per_count / cfg_.loop_rate);
    /*
          if(write_count %10==0)
       {
       RCLCPP_INFO(rclcpp::get_logger("motor values"),"%f %f",l_wheel_back.cmd / l_wheel_back.rads_per_count / cfg_.loop_rate,r_wheel_back.cmd / r_wheel_back.rads_per_count / cfg_.loop_rate);
       }
       write_count++; 
    */
       // arduino_.setMotorValues(l_wheel_front.cmd,r_wheel_front.cmd);
      return hardware_interface::return_type::OK;

    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(motor_control::MotorControl,hardware_interface::SystemInterface)
