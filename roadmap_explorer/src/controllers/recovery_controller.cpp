#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

#include "roadmap_explorer/controllers/recovery_controller.hpp"

RecoveryController::RecoveryController(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
  rclcpp::Node::SharedPtr node_ptr)
: explore_costmap_ros_(explore_costmap_ros)
{
  node_ptr_ = node_ptr;
  vel_publisher_ = node_ptr_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_nav", 10);
  costmap_ = explore_costmap_ros_->getCostmap();
}

RecoveryController::~RecoveryController()
{
  LOG_WARN("RecoveryController::~RecoveryController()");
  delete costmap_;
  vel_publisher_.reset();
}

bool RecoveryController::computeVelocityCommand(bool backward_only)
{
  // Define possible directions (angles in radians) to check (e.g., forward, left, right, etc.)
  // const std::vector<double> directions = {0.0, M_PI_2, -M_PI_2, M_PI}; // forward, left, right, backward
  std::vector<double> directions = {0.0, M_PI};
  if (backward_only) {
    directions = {M_PI};
  }
  std::vector<double> free_space_in_directions;

  // Evaluate free space in each direction
  for (double direction : directions) {
    auto value = evaluateFreeSpaceInDirection(direction);
    free_space_in_directions.push_back(value);
    LOG_TRACE("Got " << value << " for " << direction);
  }

  // Find the index of the direction with the most free space
  auto max_free_space_iter = std::min_element(
    free_space_in_directions.begin(), free_space_in_directions.end());
  int best_direction_idx = std::distance(free_space_in_directions.begin(), max_free_space_iter);

  // Compute corresponding command velocities for the best direction
  geometry_msgs::msg::Twist cmd_vel;
  double best_direction = directions[best_direction_idx];
  cmd_vel.linear.x = std::cos(best_direction) * 0.6;   // Move in the direction
  cmd_vel.linear.y = 0.0;   // Only for holonomic robots
  cmd_vel.angular.z = 0.0;                       // No rotation for simplicity
  for (auto i = 0; i < 140; i++) {
    vel_publisher_->publish(cmd_vel);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  geometry_msgs::msg::Twist cmd_vel_stopped;
  vel_publisher_->publish(cmd_vel_stopped);
  vel_publisher_->publish(cmd_vel_stopped);
  vel_publisher_->publish(cmd_vel_stopped);
  vel_publisher_->publish(cmd_vel_stopped);
  vel_publisher_->publish(cmd_vel_stopped);
  return true;
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) {angle -= 2 * M_PI;}
  while (angle < -M_PI) {angle += 2 * M_PI;}
  return angle;
}

bool RecoveryController::alignWithPose(geometry_msgs::msg::Pose & poseToAlignWith)
{
  {
    explore_costmap_ros_->getRobotPose(robot_pose_);
    auto pose_yaw = roadmap_explorer::quatToEuler(poseToAlignWith.orientation)[2];
    auto robot_yaw = roadmap_explorer::quatToEuler(robot_pose_.pose.orientation)[2];
    double deltaYaw = normalizeAngle(pose_yaw - robot_yaw);
    LOG_TRACE("DeltaYaw: " << deltaYaw);
    if (deltaYaw > 0 && fabs(deltaYaw) > 0.2) {
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.angular.z = 0.2;                           // No rotation for simplicity
      for (auto i = 0; i < 100; i++) {
        vel_publisher_->publish(cmd_vel);
        rclcpp::sleep_for(std::chrono::milliseconds(10));
      }
      geometry_msgs::msg::Twist cmd_vel_stopped;
      vel_publisher_->publish(cmd_vel_stopped);
      vel_publisher_->publish(cmd_vel_stopped);
      vel_publisher_->publish(cmd_vel_stopped);
      vel_publisher_->publish(cmd_vel_stopped);
      vel_publisher_->publish(cmd_vel_stopped);
    } else if (deltaYaw < 0 && fabs(deltaYaw) > 0.2) {
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.angular.z = -0.2;                           // No rotation for simplicity
      for (auto i = 0; i < 100; i++) {
        vel_publisher_->publish(cmd_vel);
        rclcpp::sleep_for(std::chrono::milliseconds(10));
      }
      geometry_msgs::msg::Twist cmd_vel_stopped;
      vel_publisher_->publish(cmd_vel_stopped);
      vel_publisher_->publish(cmd_vel_stopped);
      vel_publisher_->publish(cmd_vel_stopped);
      vel_publisher_->publish(cmd_vel_stopped);
      vel_publisher_->publish(cmd_vel_stopped);
    } else {

    }
  }
  explore_costmap_ros_->getRobotPose(robot_pose_);
  auto pose_yaw = roadmap_explorer::quatToEuler(poseToAlignWith.orientation)[2];
  auto robot_yaw = roadmap_explorer::quatToEuler(robot_pose_.pose.orientation)[2];
  double deltaYaw = normalizeAngle(pose_yaw - robot_yaw);
  LOG_TRACE("DeltaYaw2: " << deltaYaw);
  if (fabs(deltaYaw) > 0.2) {
    return false;
  }
  return true;
}

double RecoveryController::evaluateFreeSpaceInDirection(double direction)
{

  std::vector<nav2_costmap_2d::MapLocation> traced_cells;
  // treats cells 240 to 254 as obstacles and returns 255 in the traced cells.
  roadmap_explorer::RayTracedCells cell_gatherer(costmap_, traced_cells, 256, 256, 0, 255);
  explore_costmap_ros_->getRobotPose(robot_pose_);
  double sx = robot_pose_.pose.position.x;
  double sy = robot_pose_.pose.position.y;
  auto yaw = roadmap_explorer::quatToEuler(robot_pose_.pose.orientation)[2];
  double wx = sx + (2.5 * cos(yaw + direction));
  double wy = sy + (2.5 * sin(yaw + direction));
  if (!roadmap_explorer::getTracedCells(
      sx, sy, wx, wy, cell_gatherer,
      4.0 / costmap_->getResolution(), costmap_))
  {
    return -std::numeric_limits<double>::max();
  }
  auto cells = cell_gatherer.getCells();
  int free_space = 0;
  for (auto & cell : cells) {
    auto costmapcost = static_cast<int>(costmap_->getCost(cell.x, cell.y));
    free_space += costmapcost;
  }
  RosVisualizer::getInstance().observableCellsViz(cells);

  return free_space;
}
