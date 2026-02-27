#ifndef MOTOR_CONTROL_MOTOR_INTERFACE_H
#define MOTOR_CONTROL_MOTOR_INTERFACE_H

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "motor_control/config.h"
#include "motor_control/comms.h"
#include "motor_control/wheel.h"
#include "rclcpp/rclcpp.hpp"


using CallbackReturn=rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
namespace motor_control
{
class MotorControl: public hardware_interface::SystemInterface
{
    public : 
    RCLCPP_SHARED_PTR_DEFINITIONS(MotorControl)
    MotorControl();
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State  & previous_state) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time &time,const rclcpp::Duration &period)override;
    hardware_interface::return_type write(const rclcpp::Time &time,const rclcpp::Duration &period) override;

    private:
    Config cfg_;
    Comms arduino_;
    Wheel l_wheel_back;
    Wheel r_wheel_back;
    rclcpp::Logger log_;
    //Parameters for the robot
    double hw_start_sec_;
    double hw_stop_sec_;
    //Store the commands for the robot
    std::vector<double> hw_commands_;
    std::vector<double> hw_states_;
    std::chrono::time_point<std::chrono::system_clock> time_;
    
    
};
}//namespace for the robot

#endif
