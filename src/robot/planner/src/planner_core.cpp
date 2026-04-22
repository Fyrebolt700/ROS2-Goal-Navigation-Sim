#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger),
  has_map_(false),
  has_goal_(false),
  has_odom_(false)
{}

void PlannerCore::updateMap(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_ = *msg;
  has_map_ = true;
}

void PlannerCore::updateGoal(
  const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;
  has_goal_ = true;
}

void PlannerCore::updateOdom(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_pose_ = msg->pose.pose;
  has_odom_ = true;
}

bool PlannerCore::canPlan()
{
  return has_map_ && has_goal_ && has_odom_;
}

nav_msgs::msg::Path PlannerCore::planSimplePath()
{
  nav_msgs::msg::Path path;

  path.header.stamp = rclcpp::Clock().now();
  path.header.frame_id = "odom";

  geometry_msgs::msg::PoseStamped start;
  start.pose = robot_pose_;

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.pose.position.x = goal_.point.x;
  goal_pose.pose.position.y = goal_.point.y;
  goal_pose.pose.orientation.w = 1.0;

  path.poses.push_back(start);
  path.poses.push_back(goal_pose);

  return path;
}

}