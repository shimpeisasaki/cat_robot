#pragma once

#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>
#include "ddsm115_controller/differential_drive.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ddsm115_controller
{
// Geometry and wheel integration are independent of serial transport and motor mode.
class WheelOdometry
{
public:
  explicit WheelOdometry(rclcpp::Node & node) : node_(node)
  {
    base_ = node.declare_parameter("wheel_base", 0.208);
    radius_ = node.declare_parameter("R_wheel", 0.05065);
    left_direction_ = node.declare_parameter("left_direction", -1);
    right_direction_ = node.declare_parameter("right_direction", 1);
    offset_ = node.declare_parameter("rpm_feedback_offset", 0.5);
    frame_ = node.declare_parameter("odom_frame", std::string("odom"));
    child_ = node.declare_parameter("base_frame", std::string("base_link"));
    pose_xy_ = node.declare_parameter("pose_xy_covariance", 0.01);
    pose_yaw_ = node.declare_parameter("pose_yaw_covariance", 0.02);
    twist_linear_ = node.declare_parameter("twist_linear_covariance", 0.01);
    twist_angular_ = node.declare_parameter("twist_angular_covariance", 0.02);
    if (!std::isfinite(base_) || !std::isfinite(radius_) || base_ <= 0 || radius_ <= 0 ||
      !std::isfinite(offset_) || std::abs(offset_) > 5 ||
      std::abs(left_direction_) != 1 || std::abs(right_direction_) != 1 ||
      frame_.empty() || child_.empty() || frame_ == child_)
    {
      throw std::invalid_argument("Invalid wheel geometry, direction, offset or frames");
    }
    for (double v : {pose_xy_, pose_yaw_, twist_linear_, twist_angular_}) {
      if (!std::isfinite(v) || v < 0) {throw std::invalid_argument("Invalid odometry covariance");}
    }
    publisher_ = node.create_publisher<nav_msgs::msg::Odometry>("/wheel/odom", 10);
  }

  std::optional<std::pair<int16_t, int16_t>> command(const geometry_msgs::msg::Twist & msg) const
  {
    for (double v : {msg.linear.x, msg.linear.y, msg.linear.z,
      msg.angular.x, msg.angular.y, msg.angular.z})
    {
      if (!std::isfinite(v)) {return std::nullopt;}
    }
    const auto rpm = twist_to_wheel_rpm(msg.linear.x, msg.angular.z, base_, radius_);
    return std::pair<int16_t, int16_t>{
      std::lround(rpm.first) * left_direction_, std::lround(rpm.second) * right_direction_};
  }

  void update(int left, int right, bool healthy, std::chrono::steady_clock::time_point t)
  {
    const double dt = std::chrono::duration<double>(t - last_).count();
    last_ = t;
    // Do not turn missing feedback into a fresh zero-velocity measurement.
    if (!healthy) {previous_valid_ = false; return;}
    const double factor = 2 * M_PI * radius_ / 60.;
    const double vl = correct_rpm_feedback(left, offset_) * left_direction_ * factor;
    const double vr = correct_rpm_feedback(right, offset_) * right_direction_ * factor;
    const double linear = (vl + vr) / 2, angular = (vr - vl) / base_;
    if (previous_valid_ && dt > 0 && dt < .2) {
      pose_ = integrate_odometry(pose_, linear, angular, dt);
    }
    previous_valid_ = true;
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = node_.now();
    msg.header.frame_id = frame_;
    msg.child_frame_id = child_;
    msg.pose.pose.position.x = pose_.x;
    msg.pose.pose.position.y = pose_.y;
    msg.pose.pose.orientation.z = std::sin(pose_.yaw / 2);
    msg.pose.pose.orientation.w = std::cos(pose_.yaw / 2);
    msg.twist.twist.linear.x = linear;
    msg.twist.twist.angular.z = angular;
    for (int i : {7, 14, 21, 28}) {msg.twist.covariance[i] = 1e6;}
    for (int i : {14, 21, 28}) {msg.pose.covariance[i] = 1e6;}
    msg.pose.covariance[0] = msg.pose.covariance[7] = pose_xy_;
    msg.pose.covariance[35] = pose_yaw_;
    msg.twist.covariance[0] = twist_linear_;
    msg.twist.covariance[35] = twist_angular_;
    publisher_->publish(msg);
  }

private:
  rclcpp::Node & node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  double base_, radius_, offset_, pose_xy_, pose_yaw_, twist_linear_, twist_angular_;
  int left_direction_, right_direction_;
  std::string frame_, child_;
  Pose2D pose_;
  bool previous_valid_{false};
  std::chrono::steady_clock::time_point last_{std::chrono::steady_clock::now()};
};
}  // namespace ddsm115_controller
