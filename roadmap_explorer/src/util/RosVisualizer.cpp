#include "roadmap_explorer/util/RosVisualizer.hpp"

std::unique_ptr<RosVisualizer> RosVisualizer::RosVisualizerPtr = nullptr;
std::mutex RosVisualizer::instanceMutex_;

// ################################## Template specializations for creating publishers of different message types ##################################
template<>
std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>>
RosVisualizer::getOrCreatePublisher<sensor_msgs::msg::PointCloud2>(const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(publisher_mutex_);

  auto it = pointcloud_publishers_.find(topic_name);
  if (it != pointcloud_publishers_.end()) {
    return it->second;
  }

  // Create new publisher
  auto publisher = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 10);
  publisher->on_activate();
  pointcloud_publishers_[topic_name] = publisher;

  LOG_DEBUG("Created new PointCloud2 publisher for topic: " << topic_name);
  return publisher;
}

template<>
std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>>
RosVisualizer::getOrCreatePublisher<visualization_msgs::msg::Marker>(const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(publisher_mutex_);

  auto it = marker_publishers_.find(topic_name);
  if (it != marker_publishers_.end()) {
    return it->second;
  }

  // Create new publisher
  auto publisher = node_->create_publisher<visualization_msgs::msg::Marker>(topic_name, 10);
  publisher->on_activate();
  marker_publishers_[topic_name] = publisher;

  LOG_DEBUG("Created new Marker publisher for topic: " << topic_name);
  return publisher;
}

template<>
std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
RosVisualizer::getOrCreatePublisher<visualization_msgs::msg::MarkerArray>(
  const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(publisher_mutex_);

  auto it = marker_array_publishers_.find(topic_name);
  if (it != marker_array_publishers_.end()) {
    return it->second;
  }

  // Create new publisher
  auto publisher = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name, 10);
  publisher->on_activate();
  marker_array_publishers_[topic_name] = publisher;

  LOG_DEBUG("Created new MarkerArray publisher for topic: " << topic_name);
  return publisher;
}

template<>
std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>>
RosVisualizer::getOrCreatePublisher<nav_msgs::msg::Path>(const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(publisher_mutex_);

  auto it = path_publishers_.find(topic_name);
  if (it != path_publishers_.end()) {
    return it->second;
  }

  // Create new publisher
  auto publisher = node_->create_publisher<nav_msgs::msg::Path>(topic_name, 10);
  publisher->on_activate();
  path_publishers_[topic_name] = publisher;

  LOG_DEBUG("Created new Path publisher for topic: " << topic_name);
  return publisher;
}

template<>
std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>>
RosVisualizer::getOrCreatePublisher<geometry_msgs::msg::PoseArray>(const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(publisher_mutex_);

  auto it = pose_array_publishers_.find(topic_name);
  if (it != pose_array_publishers_.end()) {
    return it->second;
  }

  // Create new publisher
  auto publisher = node_->create_publisher<geometry_msgs::msg::PoseArray>(topic_name, 10);
  publisher->on_activate();
  pose_array_publishers_[topic_name] = publisher;

  LOG_DEBUG("Created new PoseArray publisher for topic: " << topic_name);
  return publisher;
}


// ################################## End of template specializations ##################################

// ################################### RosVisualizer class ##################################

RosVisualizer::RosVisualizer(
  std::shared_ptr<nav2_util::LifecycleNode> node,
  nav2_costmap_2d::Costmap2D * costmap)
: node_(node), costmap_(costmap)
{
  LOG_INFO("RosVisualizer constructor - publisher system initialized");

  // Pre-create all publishers used by legacy methods so they show up in RViz
  LOG_INFO("Pre-creating publishers for all legacy topic names");

  // Marker publishers
  getOrCreatePublisher<visualization_msgs::msg::Marker>("observable_cells");
  getOrCreatePublisher<visualization_msgs::msg::Marker>("connecting_cells");

  // PointCloud2 publishers
  getOrCreatePublisher<sensor_msgs::msg::PointCloud2>("spatial_hashmap_points");
  getOrCreatePublisher<sensor_msgs::msg::PointCloud2>("frontiers");
  getOrCreatePublisher<sensor_msgs::msg::PointCloud2>("all_frontiers");

  // MarkerArray publishers
  getOrCreatePublisher<visualization_msgs::msg::MarkerArray>("frontier_cell_markers");
  getOrCreatePublisher<visualization_msgs::msg::MarkerArray>("blacklisted_frontiers");

  // Path publishers
  getOrCreatePublisher<nav_msgs::msg::Path>("grid_based_frontier_plan");
  getOrCreatePublisher<nav_msgs::msg::Path>("full_path");
  getOrCreatePublisher<nav_msgs::msg::Path>("global_repositioning_path");

  // PoseArray publishers
  getOrCreatePublisher<geometry_msgs::msg::PoseArray>("trailing_robot_poses");

  LOG_INFO("All legacy publishers pre-created and activated");
}

RosVisualizer::~RosVisualizer()
{
  LOG_INFO("RosVisualizer::~RosVisualizer()");

  std::lock_guard<std::mutex> lock(publisher_mutex_);

  // Deactivate and reset all publishers
  for (auto & [topic, publisher] : pointcloud_publishers_) {
    if (publisher) {
      publisher->on_deactivate();
      publisher.reset();
    }
  }

  for (auto & [topic, publisher] : marker_publishers_) {
    if (publisher) {
      publisher->on_deactivate();
      publisher.reset();
    }
  }

  for (auto & [topic, publisher] : marker_array_publishers_) {
    if (publisher) {
      publisher->on_deactivate();
      publisher.reset();
    }
  }

  for (auto & [topic, publisher] : path_publishers_) {
    if (publisher) {
      publisher->on_deactivate();
      publisher.reset();
    }
  }

  for (auto & [topic, publisher] : pose_array_publishers_) {
    if (publisher) {
      publisher->on_deactivate();
      publisher.reset();
    }
  }

  // Clear all maps
  pointcloud_publishers_.clear();
  marker_publishers_.clear();
  marker_array_publishers_.clear();
  path_publishers_.clear();
  pose_array_publishers_.clear();
}

// ######################################## Helper methods for point cloud creation ########################################
pcl::PointCloud<pcl::PointXYZI> RosVisualizer::createPointCloudFromFrontiers(
  const std::vector<FrontierPtr> & frontier_list, float intensity)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI point(intensity);

  for (const auto & frontier : frontier_list) {
    point.x = frontier->getGoalPoint().x;
    point.y = frontier->getGoalPoint().y;
    point.z = 0.0f;
    cloud.push_back(point);
  }

  return cloud;
}

pcl::PointCloud<pcl::PointXYZI> RosVisualizer::createPointCloudFromPoints(
  const std::vector<std::vector<double>> & points, float intensity)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI point(intensity);

  for (const auto & pt : points) {
    if (pt.size() >= 2) {
      point.x = pt[0];
      point.y = pt[1];
      point.z = (pt.size() > 2) ? pt[2] : 0.0;
      cloud.push_back(point);
    }
  }

  return cloud;
}

sensor_msgs::msg::PointCloud2 RosVisualizer::convertToRosPointCloud(
  const pcl::PointCloud<pcl::PointXYZI> & pcl_cloud,
  const std::string & frame_id)
{
  sensor_msgs::msg::PointCloud2 ros_cloud;
  pcl::toROSMsg(pcl_cloud, ros_cloud);
  ros_cloud.header.frame_id = frame_id;
  ros_cloud.header.stamp = rclcpp::Clock().now();
  return ros_cloud;
}

// ######################################## New interface implementation ########################################
void RosVisualizer::visualizePointCloud(
  const std::string & topic_name,
  const std::vector<FrontierPtr> & frontier_list,
  const std::string & frame_id,
  float intensity)
{
  auto publisher = getOrCreatePublisher<sensor_msgs::msg::PointCloud2>(topic_name);
  auto pcl_cloud = createPointCloudFromFrontiers(frontier_list, intensity);
  auto ros_cloud = convertToRosPointCloud(pcl_cloud, frame_id);
  publisher->publish(ros_cloud);
}

void RosVisualizer::visualizePointCloud(
  const std::string & topic_name,
  const std::vector<std::vector<double>> & points,
  const std::string & frame_id,
  float intensity)
{
  auto publisher = getOrCreatePublisher<sensor_msgs::msg::PointCloud2>(topic_name);
  auto pcl_cloud = createPointCloudFromPoints(points, intensity);
  auto ros_cloud = convertToRosPointCloud(pcl_cloud, frame_id);
  publisher->publish(ros_cloud);
}

void RosVisualizer::visualizeMarkers(
  const std::string & topic_name,
  const std::vector<geometry_msgs::msg::Point> & points,
  const std::string & frame_id)
{
  if (costmap_ == nullptr) {
    throw RoadmapExplorerException("You called the wrong constructor. Costmap is a nullptr");
  }

  auto publisher = getOrCreatePublisher<visualization_msgs::msg::Marker>(topic_name);

  visualization_msgs::msg::Marker marker_msg;
  marker_msg.header.frame_id = frame_id;
  marker_msg.header.stamp = node_->now();
  marker_msg.type = visualization_msgs::msg::Marker::POINTS;
  marker_msg.action = visualization_msgs::msg::Marker::ADD;

  // Set the scale of the points
  marker_msg.scale.x = costmap_->getResolution();
  marker_msg.scale.y = costmap_->getResolution();

  // Set the color (green in RGBA format)
  marker_msg.color.r = 0.0;
  marker_msg.color.g = 1.0;
  marker_msg.color.b = 0.0;
  marker_msg.color.a = 1.0;

  for (const auto & point : points) {
    marker_msg.points.push_back(point);
  }

  publisher->publish(marker_msg);
}

void RosVisualizer::visualizeMarkers(
  const std::string & topic_name,
  const std::vector<nav2_costmap_2d::MapLocation> & points,
  const std::string & frame_id)
{
  if (costmap_ == nullptr) {
    throw RoadmapExplorerException("You called the wrong constructor. Costmap is a nullptr");
  }

  auto publisher = getOrCreatePublisher<visualization_msgs::msg::Marker>(topic_name);

  visualization_msgs::msg::Marker marker_msg;
  marker_msg.header.frame_id = frame_id;
  marker_msg.header.stamp = node_->now();
  marker_msg.type = visualization_msgs::msg::Marker::POINTS;
  marker_msg.action = visualization_msgs::msg::Marker::ADD;

  // Set the scale of the points
  marker_msg.scale.x = costmap_->getResolution();
  marker_msg.scale.y = costmap_->getResolution();

  // Set the color (green in RGBA format)
  marker_msg.color.r = 0.0;
  marker_msg.color.g = 1.0;
  marker_msg.color.b = 0.0;
  marker_msg.color.a = 1.0;

  for (const auto & point : points) {
    geometry_msgs::msg::Point world_point;
    costmap_->mapToWorld(point.x, point.y, world_point.x, world_point.y);
    marker_msg.points.push_back(world_point);
  }

  publisher->publish(marker_msg);
}

void RosVisualizer::visualizeFrontierMarkers(
  const std::string & topic_name,
  const std::vector<FrontierPtr> & frontier_list,
  const std::string & frame_id)
{
  auto publisher = getOrCreatePublisher<visualization_msgs::msg::MarkerArray>(topic_name);

  if (publisher->get_subscription_count() == 0) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;
  int id = 0;

  for (const auto & frontier : frontier_list) {
    if (!frontier->isAchievable()) {
      continue;
    }

    // Create a text marker for each frontier
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

    // Set marker pose (position)
    marker.pose.position.x = frontier->getGoalPoint().x;
    marker.pose.position.y = frontier->getGoalPoint().y;
    marker.pose.position.z = 0.0;

    // Set marker orientation
    marker.pose.orientation.w = 1.0;

    // Set marker scale
    marker.scale.z = 0.15; // Text height

    // Set marker color
    marker.color.a = 1.0; // Fully opaque
    marker.color.r = 1.0; // Red
    marker.color.g = 1.0; // Green
    marker.color.b = 1.0; // Blue
    marker.lifetime.sec = 1;
    marker.lifetime.nanosec = 1;

    // Set marker text
    std::stringstream ss;
    ss << "arrival_cost:" << frontier->getArrivalInformation() << "\n";
    ss << "size:" << frontier->getSize() << "\n";
    ss << "path_length:" << frontier->getPathLength() << "\n";
    ss << "path_length_m:" << frontier->getPathLengthInM() << "\n";
    marker.text = ss.str();

    markers.markers.push_back(marker);

    // Create an arrow marker for visualizing point location
    visualization_msgs::msg::Marker arrow_marker;
    arrow_marker.header.frame_id = frame_id;
    arrow_marker.header.stamp = node_->now();
    arrow_marker.id = id++;
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;

    // Set marker pose (position)
    arrow_marker.pose.position.x = frontier->getGoalPoint().x;
    arrow_marker.pose.position.y = frontier->getGoalPoint().y;
    arrow_marker.pose.position.z = 0.0;

    arrow_marker.pose.orientation = frontier->getGoalOrientation();
    arrow_marker.lifetime.sec = 1;
    arrow_marker.lifetime.nanosec = 1;

    // Set marker scale (arrow dimensions)
    arrow_marker.scale.x = 0.25; // Arrow length
    arrow_marker.scale.y = 0.1;  // Arrow width
    arrow_marker.scale.z = 0.1;  // Arrow height

    // Set marker color
    arrow_marker.color.a = 0.5; // Semi-transparent
    arrow_marker.color.r = 1.0; // Red
    arrow_marker.color.g = 0.0; // Green
    arrow_marker.color.b = 0.0; // Blue

    markers.markers.push_back(arrow_marker);
  }

  publisher->publish(markers);
}

void RosVisualizer::visualizePath(
  const std::string & topic_name,
  nav_msgs::msg::Path & path)
{
  auto publisher = getOrCreatePublisher<nav_msgs::msg::Path>(topic_name);
  publisher->publish(path);
}

void RosVisualizer::visualizePoseArray(
  const std::string & topic_name,
  const std::deque<geometry_msgs::msg::Pose> & poses,
  const std::string & frame_id)
{
  auto publisher = getOrCreatePublisher<geometry_msgs::msg::PoseArray>(topic_name);

  geometry_msgs::msg::PoseArray pose_array_msg;
  pose_array_msg.header.stamp = node_->get_clock()->now();
  pose_array_msg.header.frame_id = frame_id;

  // Add poses from deque to PoseArray
  for (const auto & pose : poses) {
    pose_array_msg.poses.push_back(pose);
  }

  publisher->publish(pose_array_msg);
}

void RosVisualizer::visualizeBlacklistedFrontierMarkers(
  const std::string & topic_name,
  const std::vector<FrontierPtr> & blacklisted_frontiers,
  const std::string & frame_id)
{
  auto publisher = getOrCreatePublisher<visualization_msgs::msg::MarkerArray>(topic_name);

  if (publisher->get_subscription_count() == 0) {
    return;
  }

  LOG_TRACE("Visualizing blacklisted frontiers, count: " << blacklisted_frontiers.size());

  visualization_msgs::msg::MarkerArray markers;
  int id = 0;

  for (const auto & frontier : blacklisted_frontiers) {
    // Create a cylinder marker for each blacklisted frontier
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime.sec = 1;
    marker.lifetime.nanosec = 1;

    // Set marker pose (position)
    marker.pose.position.x = frontier->getGoalPoint().x;
    marker.pose.position.y = frontier->getGoalPoint().y;
    marker.pose.position.z = 0.0;

    // Set marker orientation
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.15 * 2;
    marker.scale.y = 0.15 * 2;
    marker.scale.z = 0.2;
    marker.color.a = 0.7; // Semi-transparent
    marker.color.r = 0.5; // Gray color
    marker.color.g = 0.5;
    marker.color.b = 0.5;

    markers.markers.push_back(marker);
  }

  publisher->publish(markers);
}

// Utility methods
size_t RosVisualizer::getNumSubscribers(const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(publisher_mutex_);

  // Check in all publisher maps
  auto pc_it = pointcloud_publishers_.find(topic_name);
  if (pc_it != pointcloud_publishers_.end()) {
    return pc_it->second->get_subscription_count();
  }

  auto marker_it = marker_publishers_.find(topic_name);
  if (marker_it != marker_publishers_.end()) {
    return marker_it->second->get_subscription_count();
  }

  auto marker_array_it = marker_array_publishers_.find(topic_name);
  if (marker_array_it != marker_array_publishers_.end()) {
    return marker_array_it->second->get_subscription_count();
  }

  auto path_it = path_publishers_.find(topic_name);
  if (path_it != path_publishers_.end()) {
    return path_it->second->get_subscription_count();
  }

  auto pose_array_it = pose_array_publishers_.find(topic_name);
  if (pose_array_it != pose_array_publishers_.end()) {
    return pose_array_it->second->get_subscription_count();
  }

  return 0; // Topic not found
}
