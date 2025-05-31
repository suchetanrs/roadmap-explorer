#include "roadmap_explorer/util/RosVisualizer.hpp"
std::unique_ptr<RosVisualizer> RosVisualizer::RosVisualizerPtr = nullptr;
std::mutex RosVisualizer::instanceMutex_;

RosVisualizer::RosVisualizer(
  std::shared_ptr<nav2_util::LifecycleNode> node,
  nav2_costmap_2d::Costmap2D * costmap)
{
  node_ = node;
  frontier_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "frontiers",
    10);
  spatial_hashmap_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "spatial_hashmap_points", 10);

  all_frontier_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "all_frontiers",
    10);

  frontier_marker_array_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "frontier_cell_markers", 10);

  observable_cells_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>(
    "observable_cells", 10);
  connecting_cells_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>(
    "connecting_cells", 10);
  frontier_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("grid_based_frontier_plan", 10);
  full_path_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("full_path", 10);
  trailing_robot_poses_publisher_ = node->create_publisher<geometry_msgs::msg::PoseArray>(
    "trailing_robot_poses", 10);

  blacklisted_frontiers_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "blacklisted_frontiers", 10);

  frontier_cloud_pub_->on_activate();
  spatial_hashmap_pub_->on_activate();
  all_frontier_cloud_pub_->on_activate();
  frontier_marker_array_publisher_->on_activate();
  observable_cells_publisher_->on_activate();
  connecting_cells_publisher_->on_activate();
  frontier_plan_pub_->on_activate();
  full_path_plan_pub_->on_activate();
  trailing_robot_poses_publisher_->on_activate();
  blacklisted_frontiers_publisher_->on_activate();

  costmap_ = costmap;
}

RosVisualizer::~RosVisualizer()
{
  LOG_INFO("RosVisualizer::~RosVisualizer()");

  frontier_cloud_pub_->on_deactivate();
  spatial_hashmap_pub_->on_deactivate();
  all_frontier_cloud_pub_->on_deactivate();
  frontier_marker_array_publisher_->on_deactivate();
  observable_cells_publisher_->on_deactivate();
  connecting_cells_publisher_->on_deactivate();
  frontier_plan_pub_->on_deactivate();
  full_path_plan_pub_->on_deactivate();
  trailing_robot_poses_publisher_->on_deactivate();
  blacklisted_frontiers_publisher_->on_deactivate();

  // Reset all publishers
  frontier_cloud_pub_.reset();
  spatial_hashmap_pub_.reset();
  all_frontier_cloud_pub_.reset();
  frontier_marker_array_publisher_.reset();
  observable_cells_publisher_.reset();
  connecting_cells_publisher_.reset();
  frontier_plan_pub_.reset();
  full_path_plan_pub_.reset();
  trailing_robot_poses_publisher_.reset();
  blacklisted_frontiers_publisher_.reset();
}

void RosVisualizer::observableCellsViz(std::vector<geometry_msgs::msg::Point> & points)
{
  if (costmap_ == nullptr) {
    throw RoadmapExplorerException("You called the wrong constructor. Costmap is a nullptr");
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
    throw RoadmapExplorerException("You called the wrong constructor. Costmap is a nullptr");
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
  const std::vector<FrontierPtr> & frontier_list)
{
  visualization_msgs::msg::MarkerArray markers;
  int id = 0;

  if (frontier_marker_array_publisher_->get_subscription_count() == 0) {
    return;
  }

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
    marker.lifetime.sec = 1;
    marker.lifetime.nanosec = 1;

    if (!frontier->isAchievable()) {
      continue;
    }
    // Set marker text
    std::stringstream ss;
    ss << "arrival_cost:" << frontier->getArrivalInformation() << "\n";
    ss << "size:" << frontier->getSize() << "\n";
    ss << "path_length:" << frontier->getPathLength() << "\n";
    ss << "path_length_m:" << frontier->getPathLengthInM() << "\n";
    // ss << " achievability:" << frontier->isAchievable();
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
    arrow_marker.lifetime.sec = 1;
    arrow_marker.lifetime.nanosec = 1;

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

void RosVisualizer::visualizeBlacklistedFrontiers(
  const std::vector<FrontierPtr> & blacklisted_frontiers)
{
  if (blacklisted_frontiers_publisher_->get_subscription_count() == 0) {
    return;
  }

  LOG_TRACE("Visualizing blacklisted frontiers, count: " << blacklisted_frontiers.size());

  visualization_msgs::msg::MarkerArray markers;
  int id = 0;

  for (const auto & frontier : blacklisted_frontiers) {
    // Create a marker for each blacklisted frontier
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";     // Replace with your desired frame_id
    marker.header.stamp = node_->now();
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;       // Change to CYLINDER
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime.sec = 1;
    marker.lifetime.nanosec = 1;

    // Set marker pose (position)
    marker.pose.position.x = frontier->getGoalPoint().x;
    marker.pose.position.y = frontier->getGoalPoint().y;
    marker.pose.position.z = 0.0;     // Assuming 2D visualization

    // Set marker orientation (optional)
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.15 * 2;       // Ensure this is suitable for a cylinder
    marker.scale.y = 0.15 * 2;
    marker.scale.z = 0.2;
    marker.color.a = 0.7;       // Semi-transparent
    marker.color.r = 0.5;          // Red color
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    markers.markers.push_back(marker);
  }

  // Publish the blacklisted frontiers markers
  blacklisted_frontiers_publisher_->publish(markers);
}
