// Copyright 2021 ROBOTIS CO., LTD.
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

#ifndef READ_WRITE_NODE_HPP_
#define READ_WRITE_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include "geometry_msgs/msg/twist.hpp"
#include "inpipe_robot_interfaces/srv/get_position.hpp"
#include "inpipe_robot_interfaces/msg/set_multi_positions.hpp"


class ReadWriteNode : public rclcpp::Node
{
public:
  using GetPosition = inpipe_robot_interfaces::srv::GetPosition;
  using SetMultiPosition = inpipe_robot_interfaces::msg::SetMultiPositions;

  ReadWriteNode();
  virtual ~ReadWriteNode();

private:

  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;
  rclcpp::Subscription<SetMultiPosition>::SharedPtr set_multi_position_subscriber_;

  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  int present_position;

};

#endif  // READ_WRITE_NODE_HPP_
