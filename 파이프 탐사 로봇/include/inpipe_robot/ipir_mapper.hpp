#ifndef IPIR_MAPPER_HPP_
#define IPIR_MAPPER_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl_conversions/pcl_conversions.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "inpipe_robot_interfaces/msg/motor_states.hpp"

#include "inpipe_robot_interfaces/action/mapping_config.hpp"


class IpirMapperNode : public rclcpp::Node
{
public:
  IpirMapperNode();

  void addCircle(double diameter);

private:
  double d_p_;
  double x_, y_, z_;
  double roll_, pitch_, yaw_;
  bool got_odom_;
  bool got_d_p_;

  std::atomic<bool> mapping_active_;
  std::mutex goal_mutex_;

  std::vector<geometry_msgs::msg::Point> all_points_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  // rclcpp::Subscription<inpipe_robot_interfaces::msg::MotorStates>::SharedPtr diameter_sub_;

  rclcpp_action::Server<inpipe_robot_interfaces::action::MappingConfig>::SharedPtr mapping_config_server_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<inpipe_robot_interfaces::action::MappingConfig>> current_mapping_handle_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  // void recievedDP(const inpipe_robot_interfaces::msg::MotorStates::SharedPtr msg);
  void mapper(double diameter, std::vector<geometry_msgs::msg::Point>& circle_pts);
  void publishCloud();

  rclcpp_action::GoalResponse handleMapperGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const inpipe_robot_interfaces::action::MappingConfig::Goal> goal);
  rclcpp_action::CancelResponse handleMapperCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<inpipe_robot_interfaces::action::MappingConfig>> goal);
  void handleMapperAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<inpipe_robot_interfaces::action::MappingConfig>> goal);

  void saveMap(const std::string& filename);
};

#endif  // IPIR_MAPPER_HPP_
