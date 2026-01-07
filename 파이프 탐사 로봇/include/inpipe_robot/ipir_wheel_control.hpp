/**
 * Author: Geonwoo Kho, Heeseung Yoos
 */

#ifndef IPIR_WHEEL_CONTROL_HPP_
#define IPIR_WHEEL_CONTROL_HPP_

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

#include "inpipe_robot/ipir_wheel_control.hpp"


class IpirWheelControlNode : public rclcpp::Node
{
public:
  explicit IpirWheelControlNode();
  virtual ~IpirWheelControlNode();

  void sendVelocity(int id, int16_t velocity);

private:
  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  dynamixel::PortHandler* portHandler_;
  dynamixel::PacketHandler* packetHandler_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;

  std::map<int, std::chrono::steady_clock::time_point> last_log_time;

};

#endif  // IPIR_WHEEL_CONTROL_HPP_