#ifndef ROS_VISUALIZER_HPP_
#define ROS_VISUALIZER_HPP_

#include <vector>
#include <string>
#include <fstream>
#include <iomanip>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_costmap_2d/layer.hpp>

#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/Frontier.hpp"
#include "roadmap_explorer/util/GeometryUtils.hpp"

class RosVisualizer
{

public:
  ~RosVisualizer();

  static RosVisualizer & getInstance()
  {
    std::lock_guard<std::mutex> lock(instanceMutex_);
    if (RosVisualizerPtr == nullptr) {
      throw std::runtime_error("Cannot de-reference a null RosVisualizer! :(");
    }
    return *RosVisualizerPtr;
  }

  void cleanupInstance()
  {
    LOG_INFO("RosVisualizer::cleanupInstance()");
    observable_cells_publisher_.reset();
    connecting_cells_publisher_.reset();
    spatial_hashmap_pub_.reset();
    frontier_cloud_pub_.reset();
    all_frontier_cloud_pub_.reset();
    frontier_plan_pub_.reset();
    frontier_marker_array_publisher_.reset();
    full_path_plan_pub_.reset();
    trailing_robot_poses_publisher_.reset();
  }

  static void createInstance(
    std::shared_ptr<nav2_util::LifecycleNode> node,
    nav2_costmap_2d::Costmap2D * costmap)
  {
    std::cout << "Creating ros visualizer instance" << std::endl;
    std::lock_guard<std::mutex> lock(instanceMutex_);
    if (RosVisualizerPtr == nullptr) {
      RosVisualizerPtr.reset(new RosVisualizer(node, costmap));
    }
  }

  void observableCellsViz(std::vector<geometry_msgs::msg::Point> & points);
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr
    observable_cells_publisher_;

  void observableCellsViz(std::vector<nav2_costmap_2d::MapLocation> & points);
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr
    connecting_cells_publisher_;

  void visualizeSpatialHashMap(
    const std::vector<FrontierPtr> & frontier_list,
    std::string globalFrameID);
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    spatial_hashmap_pub_;

  void visualizeFrontier(
    const std::vector<FrontierPtr> & frontier_list,
    const std::vector<std::vector<double>> & every_frontier,
    std::string globalFrameID);
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr frontier_cloud_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    all_frontier_cloud_pub_;

  void visualizeFrontierMarker(
    const std::vector<FrontierPtr> & frontier_list,
    std::string globalFrameID);
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    frontier_marker_array_publisher_;

  void frontierPlanViz(nav_msgs::msg::Path & path);
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr frontier_plan_pub_;

  void fullPathPlanViz(nav_msgs::msg::Path & path);
  size_t getNumSubscribersFullPathPlan();
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr full_path_plan_pub_;

  void visualizeTrailingPoses(std::deque<geometry_msgs::msg::Pose> robot_queue);
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr
    trailing_robot_poses_publisher_;

  void visualizeBlacklistedFrontiers(
    const std::vector<FrontierPtr> & blacklisted_frontiers,
    std::string globalFrameID);
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    blacklisted_frontiers_publisher_;

private:
  // Delete copy constructor and assignment operator to prevent copying
  RosVisualizer(const RosVisualizer &) = delete;
  RosVisualizer & operator=(const RosVisualizer &) = delete;
  RosVisualizer(
    std::shared_ptr<nav2_util::LifecycleNode> node,
    nav2_costmap_2d::Costmap2D * costmap);

  static std::unique_ptr<RosVisualizer> RosVisualizerPtr;
  static std::mutex instanceMutex_;

  std::shared_ptr<nav2_util::LifecycleNode> node_;
  nav2_costmap_2d::Costmap2D * costmap_;
};

// using RosVisualizerInstance = RosVisualizer::getInstance;
#endif // ROS_VISUALIZER_HPP
