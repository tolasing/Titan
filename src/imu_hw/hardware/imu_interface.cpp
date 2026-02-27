// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Authors: Subhas Das, Denis Stogl
//

#include "imu_interface/imu_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <libserial/SerialPort.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace imu_interface
{
hardware_interface::CallbackReturn Imu_Sensor::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SensorInterface::on_init(info) !=hardware_interface::CallbackReturn::SUCCESS)
  {return hardware_interface::CallbackReturn::ERROR;}

  cfg_.baud_rate=stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.device=info_.hardware_parameters["device"];
  cfg_.timeout_ms=stoi(info_.hardware_parameters["timeout_ms"]);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Imu_Sensor::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // export sensor state interface
state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[0].name, &imu_.orientation_x));
state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[1].name, &imu_.orientation_y));
state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[2].name, &imu_.orientation_z));
state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[3].name, &imu_.orientation_w));
state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[4].name, &imu_.angular_velocity_x));
state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[5].name, &imu_.angular_velocity_y));
state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[6].name, &imu_.angular_velocity_z));
state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[7].name, &imu_.linear_acceleration_x));
state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[8].name, &imu_.linear_acceleration_y));
state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[9].name, &imu_.linear_acceleration_z));

  return state_interfaces;
}

hardware_interface::CallbackReturn Imu_Sensor::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Imu_sensor"), "Activating ...please wait...");
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms); 

  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  } 
  
  RCLCPP_INFO(rclcpp::get_logger("Imu_Sensor"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Imu_Sensor::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("Imu_Sensor"), "Deactivating ...please wait...");

  RCLCPP_INFO(
    rclcpp::get_logger("Imu_Sensor"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Imu_Sensor::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Imu_Sensor"), "Reading...");

  comms_.read_imu_values(imu_.linear_acceleration_x,
  imu_.angular_velocity_z);
   RCLCPP_INFO(
    rclcpp::get_logger("Imu_Sensor"), "%f %f",imu_.linear_acceleration_x,
  imu_.angular_velocity_z);



  return hardware_interface::return_type::OK;
}

}  // namespace imu_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  imu_interface::Imu_Sensor,
  hardware_interface::SensorInterface)
