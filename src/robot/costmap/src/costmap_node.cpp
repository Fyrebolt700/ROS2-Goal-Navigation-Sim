#include <memory>
#include <functional>

#include "costmap_node.hpp"

CostmapNode::CostmapNode()
: Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar",
    10,
    std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1)
  );

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  nav_msgs::msg::OccupancyGrid costmap_msg = costmap_.createCostmap(scan);
  costmap_pub_->publish(costmap_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}