#ifndef IMU_INTERFACE_IMU_ARDUINO_HPP
#define IMU_INTERFACE_IMU_ARDUINO_HPP

#include <string>
#include <cmath>


class Imu_Arduino
{
    public:

  double orientation_x=0.0;
  double orientation_y=0.0;
  double orientation_z=0.0;
  double orientation_w=0.0;
  double angular_velocity_x=0.0;
  double angular_velocity_y=0.0;
  double angular_velocity_z=0.0;
  double linear_acceleration_x=0.0;
  double linear_acceleration_y=0.0;
  double linear_acceleration_z=0.0;

    Imu_Arduino() = default;

};


#endif // IMU_INTERFACE_IMU_ARDUINO_HPP