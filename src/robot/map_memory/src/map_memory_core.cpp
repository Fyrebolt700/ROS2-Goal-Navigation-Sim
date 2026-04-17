#include "map_memory_core.hpp"

#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
: logger_(logger),
  has_costmap_(false),
  has_odom_(false),
  global_map_initialized_(false),
  last_update_x_(0.0),
  last_update_y_(0.0),
  distance_threshold_(1.5)
{
}

void MapMemoryCore::updateLatestCostmap(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_costmap_ = *msg;
  has_costmap_ = true;

  if (!global_map_initialized_) {
    initializeGlobalMap();
  }
}

void MapMemoryCore::updateLatestOdom(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = *msg;
  has_odom_ = true;

  if (!global_map_initialized_) {
    return;
  }
}

void MapMemoryCore::initializeGlobalMap()
{
  global_map_.header.frame_id = "odom";

  global_map_.info.resolution = 0.1;
  global_map_.info.width = 1000;
  global_map_.info.height = 1000;
  global_map_.info.origin.position.x = -50.0;
  global_map_.info.origin.position.y = -50.0;
  global_map_.info.origin.position.z = 0.0;
  global_map_.info.origin.orientation.x = 0.0;
  global_map_.info.origin.orientation.y = 0.0;
  global_map_.info.origin.orientation.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;

  global_map_.data.assign(global_map_.info.width * global_map_.info.height, -1);

  global_map_initialized_ = true;
}

double MapMemoryCore::getYawFromQuaternion(
  double x, double y, double z, double w) const
{
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

bool MapMemoryCore::readyToUpdateMap()
{
  if (!has_costmap_ || !has_odom_ || !global_map_initialized_) {
    return false;
  }

  double current_x = latest_odom_.pose.pose.position.x;
  double current_y = latest_odom_.pose.pose.position.y;

  double dx = current_x - last_update_x_;
  double dy = current_y - last_update_y_;
  double distance = std::sqrt(dx * dx + dy * dy);

  return distance >= distance_threshold_;
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::updateMap()
{
  double robot_x = latest_odom_.pose.pose.position.x;
  double robot_y = latest_odom_.pose.pose.position.y;

  double qx = latest_odom_.pose.pose.orientation.x;
  double qy = latest_odom_.pose.pose.orientation.y;
  double qz = latest_odom_.pose.pose.orientation.z;
  double qw = latest_odom_.pose.pose.orientation.w;

  double yaw = getYawFromQuaternion(qx, qy, qz, qw);

  const double local_resolution = latest_costmap_.info.resolution;
  const int local_width = static_cast<int>(latest_costmap_.info.width);
  const int local_height = static_cast<int>(latest_costmap_.info.height);
  const double local_origin_x = latest_costmap_.info.origin.position.x;
  const double local_origin_y = latest_costmap_.info.origin.position.y;

  const double global_resolution = global_map_.info.resolution;
  const int global_width = static_cast<int>(global_map_.info.width);
  const int global_height = static_cast<int>(global_map_.info.height);
  const double global_origin_x = global_map_.info.origin.position.x;
  const double global_origin_y = global_map_.info.origin.position.y;

  for (int local_y_index = 0; local_y_index < local_height; ++local_y_index) {
    for (int local_x_index = 0; local_x_index < local_width; ++local_x_index) {
      int local_index = local_y_index * local_width + local_x_index;
      int8_t cell_value = latest_costmap_.data[local_index];

      // Only fuse occupied cells from your current costmap
      if (cell_value != 100) {
        continue;
      }

      double local_x = local_origin_x + (local_x_index + 0.5) * local_resolution;
      double local_y = local_origin_y + (local_y_index + 0.5) * local_resolution;

      double world_x = robot_x + (local_x * std::cos(yaw) - local_y * std::sin(yaw));
      double world_y = robot_y + (local_x * std::sin(yaw) + local_y * std::cos(yaw));

      int global_x_index = static_cast<int>((world_x - global_origin_x) / global_resolution);
      int global_y_index = static_cast<int>((world_y - global_origin_y) / global_resolution);

      if (global_x_index < 0 || global_x_index >= global_width ||
          global_y_index < 0 || global_y_index >= global_height) {
        continue;
      }

      int global_index = global_y_index * global_width + global_x_index;
      global_map_.data[global_index] = 100;
    }
  }

  global_map_.header.stamp = rclcpp::Clock().now();

  last_update_x_ = robot_x;
  last_update_y_ = robot_y;

  return global_map_;
}

}