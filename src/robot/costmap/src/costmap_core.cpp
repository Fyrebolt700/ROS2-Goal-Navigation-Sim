#include "costmap_core.hpp"

#include <cmath>
#include <vector>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

nav_msgs::msg::OccupancyGrid CostmapCore::createCostmap(
  const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  nav_msgs::msg::OccupancyGrid costmap_msg;

  const int width = 400;
  const int height = 400;
  const double resolution = 0.1;
  const double origin_x = -20.0;
  const double origin_y = -20.0;

  costmap_msg.header.stamp = rclcpp::Clock().now();
  costmap_msg.header.frame_id = scan->header.frame_id;

  costmap_msg.info.resolution = resolution;
  costmap_msg.info.width = width;
  costmap_msg.info.height = height;
  costmap_msg.info.origin.position.x = origin_x;
  costmap_msg.info.origin.position.y = origin_y;
  costmap_msg.info.origin.position.z = 0.0;
  costmap_msg.info.origin.orientation.w = 1.0;

  costmap_msg.data.assign(width * height, 0);

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];

    if (!std::isfinite(range)) {
      continue;
    }

    double angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;

    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    int grid_x = static_cast<int>((x - origin_x) / resolution);
    int grid_y = static_cast<int>((y - origin_y) / resolution);

    if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
      int index = grid_y * width + grid_x;
      costmap_msg.data[index] = 100;
    }
  }

  return costmap_msg;
}

}