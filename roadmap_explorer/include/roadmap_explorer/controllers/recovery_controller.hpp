#ifndef RECOVERY_CONTROLLER_HPP
#define RECOVERY_CONTROLLER_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <vector>
#include <cmath>
#include "roadmap_explorer/Helpers.hpp"
#include "roadmap_explorer/util/RosVisualizer.hpp"

class RecoveryController
{
public:
  // Constructor
  RecoveryController(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
    rclcpp::Node::SharedPtr node_ptr);

  // Function to compute the velocity command based on free space
  bool computeVelocityCommand(bool backward_only);

  bool alignWithPose(geometry_msgs::msg::Pose & poseToAlignWith);

private:
  // Member variables
  nav2_costmap_2d::Costmap2D * costmap_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
  geometry_msgs::msg::PoseStamped robot_pose_;
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;

  // Private function to evaluate free space in a specific direction
  double evaluateFreeSpaceInDirection(double direction);
};

#endif // RECOVERY_CONTROLLER_HPP
