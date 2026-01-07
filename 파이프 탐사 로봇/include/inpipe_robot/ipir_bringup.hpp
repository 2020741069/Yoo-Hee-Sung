/**
 * Author: Geonwoo Kho
 */
#ifndef IPIR_BRINGUP_HPP_
#define IPIR_BRINGUP_HPP_

// #include <mutex>

#include <iostream>
#include <utility>
#include <iomanip>
#include <functional>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

#include "geometry_msgs/msg/twist.hpp"
#include "inpipe_robot_interfaces/msg/set_multi_positions.hpp"
#include "inpipe_robot_interfaces/msg/motor_states.hpp"
#include "inpipe_robot_interfaces/msg/pulse_width_modular.hpp"

#include "std_srvs/srv/empty.hpp"
#include "inpipe_robot_interfaces/srv/get_position.hpp"

#include "inpipe_robot_interfaces/action/set_joint_positions.hpp"
#include "inpipe_robot_interfaces/action/inpipe_motion.hpp"
#include "inpipe_robot_interfaces/action/corner_motion.hpp"
#include "inpipe_robot_interfaces/action/mapping_config.hpp"


class IpirBringupNode : public rclcpp::Node
{
public:
  IpirBringupNode();
  ~IpirBringupNode();

private:
  std::mutex dxl_mutex_;
  dynamixel::PortHandler* portHandler_;
  dynamixel::PacketHandler* packetHandler_;
  std::unique_ptr<dynamixel::GroupSyncWrite> groupSyncWritePos_;
  std::unique_ptr<dynamixel::GroupSyncWrite> groupSyncWriteVel_;

  rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::Publisher<inpipe_robot_interfaces::msg::MotorStates>::SharedPtr motor_states_pub_;
  rclcpp::Publisher<inpipe_robot_interfaces::msg::PulseWidthModular>::SharedPtr pwm_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr wheel_velocity_sub_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::Service<inpipe_robot_interfaces::srv::GetPosition>::SharedPtr get_position_srv_;

  rclcpp_action::Server<inpipe_robot_interfaces::action::SetJointPositions>::SharedPtr set_joint_positions_server_;
  rclcpp_action::Server<inpipe_robot_interfaces::action::InpipeMotion>::SharedPtr inpipe_motion_server_;
  rclcpp_action::Server<inpipe_robot_interfaces::action::CornerMotion>::SharedPtr corner_motion_server_;
  rclcpp_action::Server<inpipe_robot_interfaces::action::MappingConfig>::SharedPtr mapping_config_server_;

  // DXL Utilities
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;
  uint32_t goal_position = 0;
  int32_t present_position = 0;
  int16_t present_current = 0;
  uint32_t raw_center_position_ = 2048;  // default: 2048
  double corner_angles_[5] = {
    -39.54, // 0
    -65.18, // 1
    45.28,  // 2  23.63 // 2
    -65.18, // 3
    43.71   // 4
  };

  // Kinematic constants
  double D_p_;
  double R_w_;
  double W_R_;
  double H_w_;
  double R_H_o_, R_H_i_;
  double H_shaft_;
  double R_p_;
  double x_vel_;
  double dt_;
  std::vector<double> L_;

  // Kinematic utility functions
  std::pair<double, double> c_func(double x);
  std::pair<double, double> o_func(double x);
  std::pair<double, double> i_func(double x);
  std::pair<double, double> r_func(double x);

  std::pair<double, double> find_next_point(
    std::pair<double, double> prev,
    std::function<std::pair<double, double>(double)> func,
    double dist, double step = 0.001);

  std::pair<double, double> half_c_func(double x);
  std::pair<double, double> half_o_func(double x);
  std::pair<double, double> half_i_func(double x);
  std::pair<double, double> half_r_func(double x);

  std::pair<double, double> half_find_next_point(
    std::pair<double, double> prev,
    std::function<std::pair<double, double>(double)> func,
    double dist, double step = 0.001);

  double calc_theta(
    std::pair<double, double> p0,
    std::pair<double, double> p1,
    std::pair<double, double> p2);

  double calc_signed_theta(
    std::pair<double, double> p0,
    std::pair<double, double> p1,
    std::pair<double, double> p2);
  //

  std::map<int, std::chrono::steady_clock::time_point> last_log_time;

  // Utilities
  void calcJointPositions(const double raw_goal_position, std::vector<double>& goal_positions);
  void calcJointPositionsWeight(const double raw_goal_position, std::vector<double>& goal_positions, std::vector<double>& weights);
  void calcNewJointPositionsWeight(const double raw_goal_position, std::vector<double>& goal_positions, std::vector<double>& weights);
  void sendVelocity(int id, int16_t velocity);
  void syncWritePosition(int id, uint32_t profile_velocity, double goal_positions);
  void syncWriteVelocities(const std::vector<int>& ids, int16_t velocity);
  int degreeToRaw(double degree) {
    // Clamp degree between -180 and 180 if needed
    if (degree < -180.0) degree = -180.0;
    if (degree > 180.0) degree = 180.0;
    return static_cast<int>((degree + 180.0) * (4095.0 / 360.0));
  }

  // TOPIC MSG CALLBACKS
  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void publishMotorStates();

  // SERVICES
  void handleDxlReset(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);

  void getPositionCallback(const std::shared_ptr<inpipe_robot_interfaces::srv::GetPosition::Request> request,
    std::shared_ptr<inpipe_robot_interfaces::srv::GetPosition::Response> response);

  // ACTIONS
      // Action: Inpipe Motion
  rclcpp_action::GoalResponse inpipeMotionHandleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const inpipe_robot_interfaces::action::InpipeMotion::Goal> goal);

  rclcpp_action::CancelResponse inpipeMotionHandleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<inpipe_robot_interfaces::action::InpipeMotion>> goal_handle);

  void inpipeMotionHandleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<inpipe_robot_interfaces::action::InpipeMotion>> goal_handle);

    // Action: Corner Motion
  rclcpp_action::GoalResponse cornerMotionHandleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const inpipe_robot_interfaces::action::CornerMotion::Goal> goal);

  rclcpp_action::CancelResponse cornerMotionHandleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<inpipe_robot_interfaces::action::CornerMotion>> goal_handle);

  void cornerMotionHandleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<inpipe_robot_interfaces::action::CornerMotion>> goal_handle);

  // Action: Set Joint Positions
  rclcpp_action::GoalResponse setJointPositionsHandleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const inpipe_robot_interfaces::action::SetJointPositions::Goal> goal);

  rclcpp_action::CancelResponse setJointPositionsHandleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<inpipe_robot_interfaces::action::SetJointPositions>> goal_handle);

  void setJointPositionsHandleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<inpipe_robot_interfaces::action::SetJointPositions>> goal_handle);

};

#endif  // IPIR_BRINGUP_HPP_