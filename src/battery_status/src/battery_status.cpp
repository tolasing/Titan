#include "battery_status/comms.h"
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/battery_state.hpp"



class BatteryStatusNode : public rclcpp::Node
{
public:
  BatteryStatusNode() : Node("battery_status")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/battery_status", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(10), std::bind(&BatteryStatusNode::publishBatteryStatus, this));
    comms_.setup("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", 9600, 10000);
    
  }

private:
  void publishBatteryStatus()
  {
    comms_.read_battery(comms_.voltage);
     //RCLCPP_INFO(rclcpp::get_logger("imu_node"), "Serial driver is open!");
    sensor_msgs::msg::BatteryState battery;
    battery.header.frame_id = "battery";
    battery.header.stamp = rclcpp::Clock().now();
    battery.voltage=comms_.voltage;
    
    publisher_->publish(battery);
  }

  Comms comms_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryStatusNode>());
  rclcpp::shutdown();
  return 0;
}