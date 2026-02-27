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

#include "pallet_loader/pallet_loader.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"

namespace pallet_loader
{
hardware_interface::CallbackReturn PalletLoader::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::ActuatorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.device= info_.hardware_parameters["device"];
  cfg_.baud_rate= stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout = stoi(info_.hardware_parameters["timeout"]);
  cfg_.pallet_joint=info_.hardware_parameters["pallet_joint"];
  arduino_.setup(cfg_.device,cfg_.baud_rate,cfg_.timeout);
   

  RCLCPP_INFO(rclcpp::get_logger("arduino"),"%s,%i",cfg_.device.c_str(),cfg_.baud_rate);


  const hardware_interface::ComponentInfo & joint = info_.joints[0];

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> PalletLoader::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    cfg_.pallet_joint, hardware_interface::HW_IF_POSITION, &hw_joint_state_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> PalletLoader::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    cfg_.pallet_joint, hardware_interface::HW_IF_POSITION, &hw_joint_command_));

  return command_interfaces;
}

hardware_interface::CallbackReturn PalletLoader::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("PalletLoader"), "Activating ...please wait...");


  RCLCPP_INFO(rclcpp::get_logger("PalletLoader"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PalletLoader::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("PalletLoader"), "Deactivating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("PalletLoader"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type PalletLoader::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
 arduino_.read_actuator(hw_joint_state_);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type pallet_loader::PalletLoader::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  arduino_.move_pallet(hw_joint_command_);
  //hw_joint_command_=3.0; //reseting the hw_joint_command so that it doesnt continuously send command to the arduino to rotate the motor.

  return hardware_interface::return_type::OK;
}

}  // namespace pallet_loader

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  pallet_loader::PalletLoader, hardware_interface::ActuatorInterface)
