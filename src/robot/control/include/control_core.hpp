#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

class ControlCore {
  public:
    ControlCore(const rclcpp::Logger& logger);

    void updatePath(const nav_msgs::msg::Path::SharedPtr msg);
    void updateOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

    bool canControl();
    geometry_msgs::msg::Twist computeCommand();

  private:
    double getYaw(double x, double y, double z, double w);

    rclcpp::Logger logger_;
    nav_msgs::msg::Path path_;
    nav_msgs::msg::Odometry odom_;
    bool has_path_;
    bool has_odom_;
};

}

#endif