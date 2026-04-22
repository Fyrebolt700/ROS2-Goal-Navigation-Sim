#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot
{

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    void updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void updateGoal(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void updateOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

    bool canPlan();
    nav_msgs::msg::Path planSimplePath();

  private:
    rclcpp::Logger logger_;

    nav_msgs::msg::OccupancyGrid map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose robot_pose_;

    bool has_map_;
    bool has_goal_;
    bool has_odom_;
};

}

#endif