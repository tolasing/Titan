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

#ifndef IMU_INTERFACE_IMU_INTERFACE_HPP_
#define IMU_INTERFACE_IMU_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "imu_interface/visibility_control.h"
#include "imu_interface/arduino_comms.hpp"
#include "imu_interface/imu_arduino.hpp"

namespace imu_interface
{
class Imu_Sensor : public hardware_interface::SensorInterface
{

struct Config
{
  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Imu_Sensor);

  IMU_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  IMU_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  IMU_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  IMU_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  IMU_INTERFACE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Store the sensor states for the simulated robot


private:
  ArduinoComms comms_;
  Config cfg_;
  Imu_Arduino imu_;


};

}  // namespace imu_interface

#endif  // IMU_INTERFACE_IMU_INTERFACE_HPP_
