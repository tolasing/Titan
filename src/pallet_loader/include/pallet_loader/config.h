#ifndef PALLET_LOADER_CONFIG_H
#define PALLET_LOADER_CONFIG_H

#include <string>

namespace pallet_loader
{

struct Config
{
 double loop_rate=30;
 std::string device="/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_55838323835351110262-if00";
 std::string pallet_joint="pallet_loader_joint";
 int baud_rate=115200;
 int timeout= 1000;

};

}

#endif