#include "map_memory_node.hpp"

#include <memory>
#include <functional>
#include <chrono>

using namespace std::chrono_literals;

MapMemoryNode::MapMemoryNode()
: Node("map_memory"),
  map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap",
    10,
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1)
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1)
  );

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(
    1s,
    std::bind(&MapMemoryNode::timerCallback, this)
  );
}

void MapMemoryNode::costmapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_memory_.updateLatestCostmap(msg);
}

void MapMemoryNode::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  map_memory_.updateLatestOdom(msg);
}

void MapMemoryNode::timerCallback()
{
  if (map_memory_.readyToUpdateMap()) {
    nav_msgs::msg::OccupancyGrid map_msg = map_memory_.updateMap();
    map_pub_->publish(map_msg);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}