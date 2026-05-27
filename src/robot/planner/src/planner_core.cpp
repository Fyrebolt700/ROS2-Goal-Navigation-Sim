#include "planner_core.hpp"
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

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
  path.header.frame_id = "sim_world";

  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);
  const double resolution = map_.info.resolution;
  const double origin_x = map_.info.origin.position.x;
  const double origin_y = map_.info.origin.position.y;

  if (width <= 0 || height <= 0 || resolution <= 0.0) {
    RCLCPP_WARN(logger_, "Invalid map metadata. Cannot plan path.");
    return path;
  }

  auto worldToGrid = [&](double world_x, double world_y, int& grid_x, int& grid_y) -> bool {
    grid_x = static_cast<int>(std::floor((world_x - origin_x) / resolution));
    grid_y = static_cast<int>(std::floor((world_y - origin_y) / resolution));

    return grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height;
  };

  int start_x;
  int start_y;
  int goal_x;
  int goal_y;

  if (!worldToGrid(robot_pose_.position.x, robot_pose_.position.y, start_x, start_y)) {
    RCLCPP_WARN(logger_, "Robot pose is outside the map.");
    return path;
  }

  if (!worldToGrid(goal_.point.x, goal_.point.y, goal_x, goal_y)) {
    RCLCPP_WARN(logger_, "Goal is outside the map.");
    return path;
  }

  RCLCPP_INFO(
    logger_,
    "Planning from grid (%d, %d) to (%d, %d)",
    start_x,
    start_y,
    goal_x,
    goal_y
  );

  auto index = [&](int grid_x, int grid_y) -> int {
  return grid_y * width + grid_x;
  };

  auto heuristic = [&](int grid_x, int grid_y) -> double {
    const double dx = static_cast<double>(grid_x - goal_x);
    const double dy = static_cast<double>(grid_y - goal_y);
    return std::sqrt(dx * dx + dy * dy);
  };

  struct QueueNode {
    int index;
    double priority;
  };

  struct CompareQueueNode {
    bool operator()(const QueueNode& a, const QueueNode& b) const {
      return a.priority > b.priority;
    }
  };

  const int start_index = index(start_x, start_y);
  const int goal_index = index(goal_x, goal_y);

  std::priority_queue<QueueNode, std::vector<QueueNode>, CompareQueueNode> open_set;
  std::vector<double> cost_so_far(width * height, std::numeric_limits<double>::infinity());
  std::vector<int> parent(width * height, -1);

  open_set.push({start_index, 0.0});
  cost_so_far[start_index] = 0.0;
  parent[start_index] = start_index;

  geometry_msgs::msg::PoseStamped start;
  start.header = path.header;
  start.pose = robot_pose_;

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header = path.header;
  goal_pose.pose.position.x = goal_.point.x;
  goal_pose.pose.position.y = goal_.point.y;
  goal_pose.pose.position.z = 0.0;
  goal_pose.pose.orientation.w = 1.0;

  path.poses.push_back(start);
  path.poses.push_back(goal_pose);

  return path;
}

}