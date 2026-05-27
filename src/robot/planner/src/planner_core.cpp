#include "planner_core.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
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

  auto gridToWorld = [&](int grid_x, int grid_y, double& world_x, double& world_y) {
    world_x = origin_x + (static_cast<double>(grid_x) + 0.5) * resolution;
    world_y = origin_y + (static_cast<double>(grid_y) + 0.5) * resolution;
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

  auto isOccupied = [&](int grid_x, int grid_y) -> bool {
  if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height) {
    return true;
  }

  const int cell_value = map_.data[index(grid_x, grid_y)];
  return cell_value > 50;
  };

  if (isOccupied(start_x, start_y)) {
    RCLCPP_WARN(logger_, "Start cell is occupied.");
    return path;
  }

  if (isOccupied(goal_x, goal_y)) {
    RCLCPP_WARN(logger_, "Goal cell is occupied.");
    return path;
  }

  open_set.push({start_index, 0.0});
  cost_so_far[start_index] = 0.0;
  parent[start_index] = start_index;

  const std::vector<std::pair<int, int>> directions = {
  {1, 0}, {-1, 0}, {0, 1}, {0, -1},
  {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  };

  bool found_path = false;

  while (!open_set.empty()) {
    const QueueNode current_node = open_set.top();
    open_set.pop();

    const int current_index = current_node.index;

    if (current_index == goal_index) {
      found_path = true;
      break;
    }

    const int current_x = current_index % width;
    const int current_y = current_index / width;

    for (const auto& direction : directions) {
      const int next_x = current_x + direction.first;
      const int next_y = current_y + direction.second;

      if (next_x < 0 || next_x >= width || next_y < 0 || next_y >= height) {
        continue;
      }

      if (isOccupied(next_x, next_y)) {
        continue;
      }

      const int next_index = index(next_x, next_y);

      const double step_cost =
        (direction.first != 0 && direction.second != 0) ? 1.414 : 1.0;

      const double new_cost = cost_so_far[current_index] + step_cost;

      if (new_cost < cost_so_far[next_index]) {
        cost_so_far[next_index] = new_cost;
        parent[next_index] = current_index;

        const double priority = new_cost + heuristic(next_x, next_y);
        open_set.push({next_index, priority});
      }
    }
  }

  if (!found_path) {
    RCLCPP_WARN(logger_, "No obstacle-free path found.");
    return path;
  }

  std::vector<int> path_indices;
  int current = goal_index;

  while (current != start_index) {
    path_indices.push_back(current);
    current = parent[current];

    if (current < 0) {
      RCLCPP_WARN(logger_, "Path reconstruction failed.");
      return path;
    }
  }

  path_indices.push_back(start_index);
  std::reverse(path_indices.begin(), path_indices.end());

  for (int cell_index : path_indices) {
    const int grid_x = cell_index % width;
    const int grid_y = cell_index / width;

    double world_x;
    double world_y;
    gridToWorld(grid_x, grid_y, world_x, world_y);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

return path;
}

}