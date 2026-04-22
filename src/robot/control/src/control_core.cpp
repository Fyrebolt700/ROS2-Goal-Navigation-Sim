#include "control_core.hpp"

#include <cmath>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger), has_path_(false), has_odom_(false) {}

void ControlCore::updatePath(const nav_msgs::msg::Path::SharedPtr msg)
{
  path_ = *msg;
  has_path_ = true;
}

void ControlCore::updateOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_ = *msg;
  has_odom_ = true;
}

bool ControlCore::canControl()
{
  return has_path_ && has_odom_ && !path_.poses.empty();
}

double ControlCore::getYaw(double x, double y, double z, double w)
{
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Twist ControlCore::computeCommand()
{
  geometry_msgs::msg::Twist cmd;

  auto robot_pos = odom_.pose.pose.position;
  auto robot_ori = odom_.pose.pose.orientation;

  double robot_yaw = getYaw(
    robot_ori.x, robot_ori.y, robot_ori.z, robot_ori.w);

  auto goal_pos = path_.poses.back().pose.position;

  double dx = goal_pos.x - robot_pos.x;
  double dy = goal_pos.y - robot_pos.y;

  double target_angle = std::atan2(dy, dx);
  double angle_error = target_angle - robot_yaw;

  while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
  while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

  double distance = std::sqrt(dx * dx + dy * dy);

  if (distance > 0.2) {
    cmd.linear.x = 0.5;
    cmd.angular.z = angle_error;
  } else {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
  }

  return cmd;
}

}