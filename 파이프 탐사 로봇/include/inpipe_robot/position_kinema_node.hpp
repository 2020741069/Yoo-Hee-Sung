/**
 * Author: Chanwoo Seon, Geonwoo Kho
 */

#ifndef POSITION_KINEMA_NODE_HPP_
#define POSITION_KINEMA_NODE_HPP_

#include <iostream>
#include <cmath>
#include <functional>
#include <vector>
#include <utility>
#include <iomanip>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"


class PositionKinema : public rclcpp::Node
{
public:
  PositionKinema();

private:
  double D_p_;
  double R_w_;
  double W_R_;
  double H_w_;
  double R_H_o_, R_H_i_;
  double H_shaft_;
  double R_p_;
  double x_vel_;
  std::vector<double> L_;

  std::vector<std::pair<double, double>> prev_points_;
  std::vector<double> prev_theta_;


  double deg(double rad) { return rad * 180.0 / M_PI; }

  std::pair<double, double> new_o_func(double x);
  std::pair<double, double> new_i_func(double x);
  std::pair<double, double> new_c_func(double x);
  std::pair<double, double> new_r_func(double x);

  std::pair<double, double> new_find_next_point(
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

  void newLoopControl();

  void drawPose(
    const std::vector<std::pair<double, double>>& points,
    const std::vector<double>& thetas);

  void drawPoseOnCanvas(cv::Mat& canvas,
    const std::vector<std::pair<double, double>>& points,
    const std::vector<double>& thetas);

  void animateMotion();

};

#endif  // POSITION_KINEMA_NODE_HPP_
