#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    nav_msgs::msg::OccupancyGrid createCostmap(
      const sensor_msgs::msg::LaserScan::SharedPtr scan);

  private:
    rclcpp::Logger logger_;
};

}

#endif