#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void updateLatestCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void updateLatestOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

    bool readyToUpdateMap();
    nav_msgs::msg::OccupancyGrid updateMap();

  private:
    void initializeGlobalMap();
    double getYawFromQuaternion(double x, double y, double z, double w) const;

    rclcpp::Logger logger_;

    nav_msgs::msg::OccupancyGrid latest_costmap_;
    nav_msgs::msg::Odometry latest_odom_;
    nav_msgs::msg::OccupancyGrid global_map_;

    bool has_costmap_;
    bool has_odom_;
    bool global_map_initialized_;

    double last_update_x_;
    double last_update_y_;
    double distance_threshold_;
};

}

#endif