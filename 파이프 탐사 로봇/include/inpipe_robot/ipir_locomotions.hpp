/**
 * Author: Geonwoo Kho, Chanwoo Seon
 */

#ifndef IPIR_LOCOMOTIONS_HPP_
#define IPIR_LOCOMOTIONS_HPP_

#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include "geometry_msgs/msg/twist.hpp"
#include "inpipe_robot_interfaces/action/set_joint_positions.hpp"
#include "inpipe_robot_interfaces/action/corner_motion.hpp"


using namespace std::chrono_literals;

// Uncomment to enable continuous negative rotation of motors 5,6,7
//#define ENABLE_CONTINUOUS_NEGATIVE_ROTATION


class IpirLocomotionNode : public rclcpp::Node
{
public:
  IpirLocomotionNode();
  virtual ~IpirLocomotionNode();

private:
  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  std::unique_ptr<dynamixel::GroupSyncWrite> groupSyncWritePos_;
  std::unique_ptr<dynamixel::GroupSyncWrite> groupSyncWriteVel_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr wheel_velocity_sub_;

  std::thread control_thread_;
  std::thread continuous_rotation_thread_;

  std::vector<int> position_ids_;
  std::vector<int> velocity_ids_;
  std::vector<double> in_pipe_positions_;
  std::vector<double> target_positions_;
  std::vector<int> move_times_;

  bool stop_ = false;

  int dxl_comm_result = COMM_TX_FAIL;
  uint32_t goal_position = 0;
  int32_t present_position = 0;
  int16_t present_current = 0;

  void sendVelocity(int id, int16_t velocity);
  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  void setOperatingMode(uint8_t id, int mode);
  void enableTorque(uint8_t id);

  // Set goal position for motor in degrees
  void setGoalPosition(uint8_t id, double degree);

  void syncWritePosition(
    int id,
    uint32_t profile_velocity,
    double goal_positions);

  void syncWriteVelocities(
    const std::vector<int>& ids,
    int16_t velocity);

  // Control loop: move motors 0~4 sequentially to target and back
  void controlLoop();
  void syncControlLoop();

  // Continuously rotate motors 5,6,7 in negative direction
  // void continuousNegativeRotationLoop() {}


  // Convert degrees to raw Dynamixel position (0~4095)
  int degreeToRaw(double degree) {
    // Clamp degree between -180 and 180 if needed
    if (degree < -180.0) degree = -180.0;
    if (degree > 180.0) degree = 180.0;
    return static_cast<int>((degree + 180.0) * (4095.0 / 360.0));
  }

};


#endif  // IPIR_LOCOMOTIONS_HPP_