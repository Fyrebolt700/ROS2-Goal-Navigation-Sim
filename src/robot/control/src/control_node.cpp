#include "control_node.hpp"

#include <memory>
#include <functional>
#include <chrono>

using namespace std::chrono_literals;

ControlNode::ControlNode()
: Node("control"), control_(robot::ControlCore(this->get_logger()))
{
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path",
    10,
    std::bind(&ControlNode::pathCallback, this, std::placeholders::_1)
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    10,
    std::bind(&ControlNode::odomCallback, this, std::placeholders::_1)
  );

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ = this->create_wall_timer(
    100ms,
    std::bind(&ControlNode::timerCallback, this)
  );
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  control_.updatePath(msg);
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  control_.updateOdom(msg);
}

void ControlNode::timerCallback()
{
  if (control_.canControl()) {
    geometry_msgs::msg::Twist cmd = control_.computeCommand();
    cmd_pub_->publish(cmd);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}