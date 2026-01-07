#ifndef IPIR_ODOMETRY_HPP_
#define IPIR_ODOMETRY_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "inpipe_robot_interfaces/msg/motor_states.hpp"

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


class IpirOdometryNode : public rclcpp::Node {
public:
  IpirOdometryNode();
  virtual ~IpirOdometryNode();

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Subscription<inpipe_robot_interfaces::msg::MotorStates>::SharedPtr motor_state_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

  double x_, y_, theta_, roll_;

  // Roll (Robot's)
  double imu_yaw_;
  // Pitch (Robot's)
  double imu_roll_;
  double imu_pitch_;

  rclcpp::Time last_time_;

  double v_front_, v_mid_, v_rear_, v_roll_;
  bool got_motor_, got_imu_;

  void motorCallback(const inpipe_robot_interfaces::msg::MotorStates::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  void tryComputeOdom();

};

#endif  // IPIR_ODOMETRY_HPP_