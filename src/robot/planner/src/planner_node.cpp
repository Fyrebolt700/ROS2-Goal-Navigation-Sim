#include "planner_node.hpp"

#include <memory>
#include <functional>
#include <chrono>

using namespace std::chrono_literals;

PlannerNode::PlannerNode()
: Node("planner"),
  planner_(robot::PlannerCore(this->get_logger()))
{
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    10,
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1)
  );

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point",
    10,
    std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1)
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    10,
    std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1)
  );

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
    500ms,
    std::bind(&PlannerNode::timerCallback, this)
  );
}

void PlannerNode::mapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  planner_.updateMap(msg);
}

void PlannerNode::goalCallback(
  const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  planner_.updateGoal(msg);
}

void PlannerNode::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  planner_.updateOdom(msg);
}

void PlannerNode::timerCallback()
{
  if (planner_.canPlan()) {
    nav_msgs::msg::Path path = planner_.planSimplePath();
    path_pub_->publish(path);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}