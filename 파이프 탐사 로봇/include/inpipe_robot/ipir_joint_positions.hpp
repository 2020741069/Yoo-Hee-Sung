/**
 * Author: Geonwoo Kho
 */

#ifndef IPIR_JOINT_POSITIONS_HPP_
#define IPIR_JOINT_POSITIONS_HPP_

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

#include "inpipe_robot_interfaces/msg/set_multi_positions.hpp"
#include "inpipe_robot_interfaces/srv/get_position.hpp"

#include "rcutils/cmdline_parser.h"


using SetMultiPositions = inpipe_robot_interfaces::msg::SetMultiPositions;
using GetPosition = inpipe_robot_interfaces::srv::GetPosition;

class IpirJointPositions : public rclcpp::Node
{
public:
  int dxl_comm_result = COMM_TX_FAIL;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  IpirJointPositions();
  virtual ~IpirJointPositions();

  void setupDynamixel(uint8_t dxl_id);

private:
  void setMultiPositionsCallback(const SetMultiPositions::SharedPtr msg);
  void getPositionCallback(const std::shared_ptr<GetPosition::Request> request,
                           std::shared_ptr<GetPosition::Response> response);


  rclcpp::Subscription<SetMultiPositions>::SharedPtr set_multi_positions_sub_;
  rclcpp::Service<GetPosition>::SharedPtr get_position_srv_;

  uint8_t dxl_error = 0;
  uint32_t goal_position = 0;
  int32_t present_position = 0;
  int16_t present_current = 0;

};

#endif  // IPIR_JOINT_POSITIONS_HPP_