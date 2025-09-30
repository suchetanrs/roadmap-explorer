#ifndef ROS_VISUALIZER_HPP_
#define ROS_VISUALIZER_HPP_

#include <vector>
#include <string>
#include <unordered_map>
#include <memory>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_util/lifecycle_node.hpp>

#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/Frontier.hpp"
#include "roadmap_explorer/util/GeometryUtils.hpp"

class RosVisualizer
{
  // Type aliases for different publisher types
  using PointCloud2Publisher =
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr;
  using MarkerPublisher =
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr;
  using MarkerArrayPublisher =
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr;
  using PathPublisher = rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr;
  using PoseArrayPublisher =
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr;

public:
  ~RosVisualizer();

  static void createInstance(
    std::shared_ptr<nav2_util::LifecycleNode> node,
    nav2_costmap_2d::Costmap2D * costmap)
  {
    LOG_INFO("Creating ros visualizer instance");
    std::lock_guard<std::mutex> lock(instanceMutex_);
    if (RosVisualizerPtr == nullptr) {
      RosVisualizerPtr.reset(new RosVisualizer(node, costmap));
    } else {
      throw RoadmapExplorerException("RosVisualizer instance already exists!");
    }
  }

  static RosVisualizer & getInstance()
  {
    std::lock_guard<std::mutex> lock(instanceMutex_);
    if (RosVisualizerPtr == nullptr) {
      throw RoadmapExplorerException("Cannot de-reference a null RosVisualizer! :(");
    }
    return *RosVisualizerPtr;
  }

  static void destroyInstance()
  {
    LOG_INFO("RosVisualizer::destroyInstance()");
    RosVisualizerPtr.reset();
  }

  // New interface: topic-based visualization methods
  void visualizePointCloud(
    const std::string & topic_name,
    const std::vector<FrontierPtr> & frontier_list,
    const std::string & frame_id = "map",
    float intensity = 50.0f);

  void visualizePointCloud(
    const std::string & topic_name,
    const std::vector<std::vector<double>> & points,
    const std::string & frame_id = "map",
    float intensity = 500.0f);

  void visualizeMarkers(
    const std::string & topic_name,
    const std::vector<geometry_msgs::msg::Point> & points,
    const std::string & frame_id = "map");

  void visualizeMarkers(
    const std::string & topic_name,
    const std::vector<nav2_costmap_2d::MapLocation> & points,
    const std::string & frame_id = "map");

  void visualizeFrontierMarkers(
    const std::string & topic_name,
    const std::vector<FrontierPtr> & frontier_list,
    const std::string & frame_id = "map");

  void visualizePath(
    const std::string & topic_name,
    nav_msgs::msg::Path & path);

  void visualizePoseArray(
    const std::string & topic_name,
    const std::deque<geometry_msgs::msg::Pose> & poses,
    const std::string & frame_id = "map");

  void visualizeBlacklistedFrontierMarkers(
    const std::string & topic_name,
    const std::vector<FrontierPtr> & blacklisted_frontiers,
    const std::string & frame_id = "map");

  // Utility methods
  size_t getNumSubscribers(const std::string & topic_name);

private:
  // Delete copy constructor and assignment operator to prevent copying
  RosVisualizer(const RosVisualizer &) = delete;
  RosVisualizer & operator=(const RosVisualizer &) = delete;
  RosVisualizer(
    std::shared_ptr<nav2_util::LifecycleNode> node,
    nav2_costmap_2d::Costmap2D * costmap);

  // Template methods for publisher management
  template<typename MessageType>
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MessageType>> getOrCreatePublisher(
    const std::string & topic_name);

  // Helper methods for point cloud creation
  pcl::PointCloud<pcl::PointXYZI> createPointCloudFromFrontiers(
    const std::vector<FrontierPtr> & frontier_list, float intensity = 50.0f);

  pcl::PointCloud<pcl::PointXYZI> createPointCloudFromPoints(
    const std::vector<std::vector<double>> & points, float intensity = 500.0f);

  sensor_msgs::msg::PointCloud2 convertToRosPointCloud(
    const pcl::PointCloud<pcl::PointXYZI> & pcl_cloud,
    const std::string & frame_id);

  static std::unique_ptr<RosVisualizer> RosVisualizerPtr;
  static std::mutex instanceMutex_;

  std::shared_ptr<nav2_util::LifecycleNode> node_;
  nav2_costmap_2d::Costmap2D * costmap_;

  // Publisher maps for different message types
  std::unordered_map<std::string, PointCloud2Publisher> pointcloud_publishers_;
  std::unordered_map<std::string, MarkerPublisher> marker_publishers_;
  std::unordered_map<std::string, MarkerArrayPublisher> marker_array_publishers_;
  std::unordered_map<std::string, PathPublisher> path_publishers_;
  std::unordered_map<std::string, PoseArrayPublisher> pose_array_publishers_;

  // Mutex for thread-safe publisher access
  std::mutex publisher_mutex_;
};

#define rosVisualizerInstance (RosVisualizer::getInstance())
#endif // ROS_VISUALIZER_HPP
