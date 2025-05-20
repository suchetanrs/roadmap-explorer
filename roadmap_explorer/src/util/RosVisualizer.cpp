#include "roadmap_explorer/util/RosVisualizer.hpp"
std::unique_ptr<RosVisualizer> RosVisualizer::RosVisualizerPtr = nullptr;
std::mutex RosVisualizer::instanceMutex_;

RosVisualizer::RosVisualizer(rclcpp::Node::SharedPtr node, nav2_costmap_2d::Costmap2D * costmap)
{
  node_ = node;
  // Creating publishers with custom QoS settings
  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  frontier_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "frontiers",
    custom_qos);
  spatial_hashmap_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "spatial_hashmap_points", custom_qos);

  all_frontier_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "all_frontiers",
    custom_qos);

  frontier_marker_array_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "frontier_cell_markers", 10);

  observable_cells_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>(
    "observable_cells", 10);
  connecting_cells_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>(
    "connecting_cells", 10);
  frontier_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("frontier_plan", 10);
  full_path_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("full_path", 10);
  trailing_robot_poses_publisher_ = node->create_publisher<geometry_msgs::msg::PoseArray>(
    "trailing_robot_poses", 10);
  costmap_ = costmap;
}

void RosVisualizer::observableCellsViz(std::vector<geometry_msgs::msg::Point> & points)
{
  if (costmap_ == nullptr) {
    throw std::runtime_error("You called the wrong constructor. Costmap is a nullptr");
  }
  visualization_msgs::msg::Marker marker_msg_;
  // Initialize the Marker message
  marker_msg_.header.frame_id = "map";   // Set the frame ID
  marker_msg_.type = visualization_msgs::msg::Marker::POINTS;
  marker_msg_.action = visualization_msgs::msg::Marker::ADD;

  // Set the scale of the points
  marker_msg_.scale.x = costmap_->getResolution();   // Point size
  marker_msg_.scale.y = costmap_->getResolution();

  // Set the color (green in RGBA format)
  marker_msg_.color.r = 0.0;
  marker_msg_.color.g = 1.0;
  marker_msg_.color.b = 0.0;
  marker_msg_.color.a = 1.0;
  for (auto point : points) {
    marker_msg_.points.push_back(point);
  }
  observable_cells_publisher_->publish(marker_msg_);
}

void RosVisualizer::observableCellsViz(std::vector<nav2_costmap_2d::MapLocation> & points)
{
  if (costmap_ == nullptr) {
    throw std::runtime_error("You called the wrong constructor. Costmap is a nullptr");
  }
  visualization_msgs::msg::Marker marker_msg_;
  // Initialize the Marker message
  marker_msg_.header.frame_id = "map";   // Set the frame ID
  marker_msg_.type = visualization_msgs::msg::Marker::POINTS;
  marker_msg_.action = visualization_msgs::msg::Marker::ADD;

  // Set the scale of the points
  marker_msg_.scale.x = costmap_->getResolution();   // Point size
  marker_msg_.scale.y = costmap_->getResolution();

  // Set the color (green in RGBA format)
  marker_msg_.color.r = 0.0;
  marker_msg_.color.g = 1.0;
  marker_msg_.color.b = 0.0;
  marker_msg_.color.a = 1.0;
  for (auto point : points) {
    geometry_msgs::msg::Point point1;
    costmap_->mapToWorld(point.x, point.y, point1.x, point1.y);
    marker_msg_.points.push_back(point1);
  }
  connecting_cells_publisher_->publish(marker_msg_);
}

void RosVisualizer::visualizeSpatialHashMap(
  const std::vector<FrontierPtr> & frontier_list,
  std::string globalFrameID)
{
  pcl::PointCloud<pcl::PointXYZI> spatial_hashmap_viz;
  pcl::PointXYZI frontier_point_viz(50);
  for (const auto & frontier : frontier_list) {
    // load frontier into visualization poitncloud
    frontier_point_viz.x = frontier->getGoalPoint().x;
    frontier_point_viz.y = frontier->getGoalPoint().y;
    spatial_hashmap_viz.push_back(frontier_point_viz);
  }

  // publish visualization point cloud
  sensor_msgs::msg::PointCloud2 frontier_viz_output;
  pcl::toROSMsg(spatial_hashmap_viz, frontier_viz_output);
  frontier_viz_output.header.frame_id = globalFrameID;
  frontier_viz_output.header.stamp = rclcpp::Clock().now();
  spatial_hashmap_pub_->publish(frontier_viz_output);
}

void RosVisualizer::visualizeFrontier(
  const std::vector<FrontierPtr> & frontier_list,
  const std::vector<std::vector<double>> & every_frontier,
  std::string globalFrameID)
{
  // pointcloud for visualization purposes
  pcl::PointCloud<pcl::PointXYZI> frontier_cloud_viz;
  pcl::PointXYZI frontier_point_viz(50);

  // pointcloud for visualization purposes
  pcl::PointCloud<pcl::PointXYZI> all_frontier_cloud_viz;
  pcl::PointXYZI all_frontier_point_viz(500);

  for (const auto & frontier : frontier_list) {
    // load frontier into visualization poitncloud
    frontier_point_viz.x = frontier->getGoalPoint().x;
    frontier_point_viz.y = frontier->getGoalPoint().y;
    frontier_cloud_viz.push_back(frontier_point_viz);
  }

  for (const auto & frontier : every_frontier) {
    // load frontier into visualization poitncloud
    all_frontier_point_viz.x = frontier[0];
    all_frontier_point_viz.y = frontier[1];
    all_frontier_cloud_viz.push_back(all_frontier_point_viz);
  }

  // publish visualization point cloud
  sensor_msgs::msg::PointCloud2 frontier_viz_output;
  pcl::toROSMsg(frontier_cloud_viz, frontier_viz_output);
  frontier_viz_output.header.frame_id = globalFrameID;
  frontier_viz_output.header.stamp = rclcpp::Clock().now();
  frontier_cloud_pub_->publish(frontier_viz_output);

  // publish visualization point cloud (all frontiers)
  sensor_msgs::msg::PointCloud2 all_frontier_viz_output;
  pcl::toROSMsg(all_frontier_cloud_viz, all_frontier_viz_output);
  all_frontier_viz_output.header.frame_id = globalFrameID;
  all_frontier_viz_output.header.stamp = rclcpp::Clock().now();
  all_frontier_cloud_pub_->publish(all_frontier_viz_output);
}

void RosVisualizer::visualizeFrontierMarker(
  const std::vector<FrontierPtr> & frontier_list,
  const std::vector<std::vector<double>> & every_frontier,
  std::string globalFrameID)
{
  visualization_msgs::msg::MarkerArray markers;
  int id = 0;

  for (const auto & frontier : frontier_list) {
    // Create a marker for each frontier
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";     // Replace with your desired frame_id
    marker.header.stamp = node_->now();
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;     // Text marker type

    // Set marker pose (position)
    marker.pose.position.x = frontier->getGoalPoint().x;
    marker.pose.position.y = frontier->getGoalPoint().y;
    marker.pose.position.z = 0.0;     // Assuming 2D visualization

    // Set marker orientation (optional)
    marker.pose.orientation.w = 1.0;

    // Set marker scale
    marker.scale.z = 0.15;     // Text height

    // Set marker color
    marker.color.a = 1.0;     // Fully opaque
    marker.color.r = 1.0;     // Red
    marker.color.g = 1.0;     // Green
    marker.color.b = 1.0;     // Blue

    // Set marker text
    std::stringstream ss;
    ss << "weighted_cost:" << frontier->getWeightedCost() << "\n dist_ut:" << frontier->getCost(
      "distance_utility")
       << "\n path_len:" << frontier->getPathLength() << ", " << frontier->getPathLengthInM() <<
      "\n path_heading:" << frontier->getPathHeading() * 180 / M_PI
       << "\n arrival_ut:" << frontier->getCost("arrival_gain_utility") << "\n arrival_info:" <<
      frontier->getArrivalInformation()
       << "\n achievability:" << frontier->isAchievable();     // Example: Display weighted cost
    marker.text = ss.str();

    markers.markers.push_back(marker);

    // Create a sphere marker for visualizing point location
    visualization_msgs::msg::Marker arrow_marker;
    arrow_marker.header.frame_id = "map";     // Replace with your desired frame_id
    arrow_marker.header.stamp = node_->now();
    arrow_marker.id = id++;
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;     // Sphere marker type

    // Set marker pose (position)
    arrow_marker.pose.position.x = frontier->getGoalPoint().x;
    arrow_marker.pose.position.y = frontier->getGoalPoint().y;
    arrow_marker.pose.position.z = 0.0;     // On the ground (adjust as needed)

    arrow_marker.pose.orientation = frontier->getGoalOrientation();

    // Set marker scale (sphere diameter)
    arrow_marker.scale.x = 0.25;     // Arrow length
    arrow_marker.scale.y = 0.1;      // Arrow width
    arrow_marker.scale.z = 0.1;      // Arrow height

    // Set marker color
    arrow_marker.color.a = 0.5;     // Fully opaque
    arrow_marker.color.r = 1.0;     // Red
    arrow_marker.color.g = 0.0;     // Green
    arrow_marker.color.b = 0.0;     // Blue

    markers.markers.push_back(arrow_marker);
  }

  // Publish the marker array
  frontier_marker_array_publisher_->publish(markers);
}

void RosVisualizer::frontierPlanViz(nav_msgs::msg::Path & path)
{
  frontier_plan_pub_->publish(path);
}

void RosVisualizer::fullPathPlanViz(nav_msgs::msg::Path & path)
{
  full_path_plan_pub_->publish(path);
}

size_t RosVisualizer::getNumSubscribersFullPathPlan()
{
  return full_path_plan_pub_->get_subscription_count();
}

void RosVisualizer::visualizeTrailingPoses(std::deque<geometry_msgs::msg::Pose> robot_queue)
{
  geometry_msgs::msg::PoseArray pose_array_msg;
  pose_array_msg.header.stamp = node_->get_clock()->now();
  pose_array_msg.header.frame_id = "map";   // Use the appropriate frame ID for your setup

  // Add poses from deque to PoseArray
  for (const auto & pose : robot_queue) {
    pose_array_msg.poses.push_back(pose);
  }

  // Publish PoseArray
  trailing_robot_poses_publisher_->publish(pose_array_msg);
}
