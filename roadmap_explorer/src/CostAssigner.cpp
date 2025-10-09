#include <roadmap_explorer/CostAssigner.hpp>

namespace roadmap_explorer
{
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using rcl_interfaces::msg::ParameterType;

CostAssigner::CostAssigner(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros)
{
  layered_costmap_ = explore_costmap_ros->getLayeredCostmap();
  costmap_ = explore_costmap_ros->getCostmap();
  explore_costmap_ros_ = explore_costmap_ros;
  LOG_DEBUG("Got costmap pointer");

  LOG_INFO("CostAssigner::CostAssigner");
}

CostAssigner::~CostAssigner()
{
  LOG_INFO("CostAssigner::~CostAssigner()");
}

bool CostAssigner::populateFrontierCosts(
  std::shared_ptr<CalculateFrontierCostsRequest> requestData)
{
  // set the blacklist
  setFrontierBlacklist(requestData->prohibited_frontiers);
  rosVisualizerInstance.visualizeBlacklistedFrontierMarkers("blacklisted_frontiers", requestData->prohibited_frontiers);

  // process the chosen approach
  return assignCosts(requestData->frontier_list, requestData->start_pose.pose);
}

void CostAssigner::updateBoundaryPolygon(geometry_msgs::msg::PolygonStamped & explore_boundary)
{
  geometry_msgs::msg::Polygon polygon;
  // Transform all points of boundary polygon into costmap frame
  geometry_msgs::msg::PointStamped in;
  in.header = explore_boundary.header;
  for (const auto & point32 : explore_boundary.polygon.points) {
    LOG_TRACE(
      "Sending Polygon from config x:" << point32.x << " y: " << point32.y << " z: " <<
        point32.z);
    in.point = nav2_costmap_2d::toPoint(point32);
    polygon.points.push_back(nav2_costmap_2d::toPoint32(in.point));
  }

  // if empty boundary provided, set to whole map
  if (polygon.points.empty()) {
    geometry_msgs::msg::Point32 temp;
    temp.x = layered_costmap_->getCostmap()->getOriginX();
    temp.y = layered_costmap_->getCostmap()->getOriginY();
    polygon.points.push_back(temp);
    temp.y = layered_costmap_->getCostmap()->getSizeInMetersY();
    polygon.points.push_back(temp);
    temp.x = layered_costmap_->getCostmap()->getSizeInMetersX();
    polygon.points.push_back(temp);
    temp.y = layered_costmap_->getCostmap()->getOriginY();
    polygon.points.push_back(temp);
  }

  // Find map size and origin by finding min/max points of polygon
  double min_x_polygon = std::numeric_limits<double>::infinity();
  double min_y_polygon = std::numeric_limits<double>::infinity();
  double max_x_polygon = -std::numeric_limits<double>::infinity();       // observe the minus here
  double max_y_polygon = -std::numeric_limits<double>::infinity();       // observe the minus here

  for (const auto & point : polygon.points) {
    min_x_polygon = std::min(min_x_polygon, (double)point.x);
    min_y_polygon = std::min(min_y_polygon, (double)point.y);
    max_x_polygon = std::max(max_x_polygon, (double)point.x);
    max_y_polygon = std::max(max_y_polygon, (double)point.y);
  }

  polygon_xy_min_max_.push_back(min_x_polygon);
  polygon_xy_min_max_.push_back(min_y_polygon);
  polygon_xy_min_max_.push_back(max_x_polygon);
  polygon_xy_min_max_.push_back(max_y_polygon);
  return;
}

bool CostAssigner::assignCosts(
  std::vector<FrontierPtr> & frontier_list,
  geometry_msgs::msg::Pose start_pose_w)
{
  auto informationGainPlugin = parameterInstance.getValue<std::string>(
    "costAssigner.information_gain_plugin");

  auto plannerPlugin = parameterInstance.getValue<std::string>(
    "costAssigner.planner_plugin");

  LOG_INFO("Information gain plugin: " << informationGainPlugin);
  LOG_INFO("Planner plugin: " << plannerPlugin);

  min_traversable_distance = std::numeric_limits<double>::max();
  max_traversable_distance = -1.0 * std::numeric_limits<double>::max();

  min_arrival_info_per_frontier = std::numeric_limits<double>::max();
  max_arrival_info_per_frontier = -1.0 * std::numeric_limits<double>::max();
  LOG_DEBUG("CostAssigner::assignCosts");
  
  // Sanity checks
  if (frontier_list.empty()) {
    LOG_ERROR("No frontiers found from frontier search.");
    return false;
  }

  if (polygon_xy_min_max_.size() <= 0) {
    LOG_ERROR("FrontierPtr cannot be selected, no polygon.");
    return false;
  }
  
  // Check for duplicates
  if (findDuplicates(frontier_list).size() > 0) {
    throw RoadmapExplorerException("Duplicate frontiers found.");
  }

  // Initialize plugins once for all frontiers
  initializePlugins(informationGainPlugin, plannerPlugin);

  // Process each frontier
  LOG_DEBUG("FrontierPtr list size is (loop): " + std::to_string(frontier_list.size()));
  
  for (auto & frontier : frontier_list) {
    processFrontier(frontier, start_pose_w);
  }

  return true;
}

std::vector<FrontierPtr> CostAssigner::findDuplicates(const std::vector<FrontierPtr> & vec)
{
  std::vector<FrontierPtr> duplicates;

  // Iterate through the vector
  for (size_t i = 0; i < vec.size(); ++i) {
    // Compare the current element with all subsequent elements
    for (size_t j = i + 1; j < vec.size(); ++j) {
      if (vec[i] == vec[j]) {
        // If a duplicate is found, add it to the duplicates vector
        duplicates.push_back(vec[i]);
        break;             // Break the inner loop to avoid adding the same duplicate multiple times
      }
    }
  }

  return duplicates;
}

void CostAssigner::setFrontierBlacklist(std::vector<FrontierPtr> & blacklist)
{
  std::lock_guard<std::mutex> lock(blacklist_mutex_);
  LOG_DEBUG("Setting frontier blacklist with size: " << blacklist.size());
  for (auto frontier : blacklist) {
    LOG_DEBUG("Adding frontier to blacklist: " << frontier);
    frontier_blacklist_[frontier] = true;
  }

  LOG_DEBUG("Blacklist size is: " << frontier_blacklist_.size());
}

void CostAssigner::initializePlugins(const std::string & informationGainPlugin, const std::string & plannerPlugin)
{
  // Initialize information gain plugin if needed
  try {
    pluginlib::ClassLoader<BaseInformationGain> plugin_loader(
      "roadmap_explorer", "roadmap_explorer::BaseInformationGain");
    
    information_gain_plugin_ = plugin_loader.createSharedInstance(informationGainPlugin);
    information_gain_plugin_->configure(explore_costmap_ros_);
    LOG_INFO("Loaded information gain plugin: " << informationGainPlugin);
  } catch (const std::exception& e) {
    LOG_WARN("Failed to load information gain plugin: " << e.what());
    throw std::runtime_error("Failed to load information gain plugin");
  }

  // Initialize planner plugin if needed
  try {
    pluginlib::ClassLoader<BasePlanner> plugin_loader(
      "roadmap_explorer", "roadmap_explorer::BasePlanner");

    planner_plugin_ = plugin_loader.createSharedInstance(plannerPlugin);
    planner_plugin_->configure(explore_costmap_ros_);
    LOG_INFO("Loaded planner plugin: " << plannerPlugin);
  } catch (const std::exception& e) {
    LOG_WARN("Failed to load planner plugin: " << e.what());
    throw std::runtime_error("Failed to load planner plugin");
  }
}

void CostAssigner::processFrontier(FrontierPtr & frontier,
  const geometry_msgs::msg::Pose & start_pose_w)
{
  // Check if frontier is blacklisted
  if (frontier_blacklist_.count(frontier) > 0) {
    setFrontierUnachievable(frontier);
    return;
  }

  // Check if frontier is outside polygon
  auto goal_point = frontier->getGoalPoint();
  if (goal_point.x < polygon_xy_min_max_[0] ||
      goal_point.y < polygon_xy_min_max_[1] ||
      goal_point.x > polygon_xy_min_max_[2] ||
      goal_point.y > polygon_xy_min_max_[3]) {
    LOG_DEBUG("Frontier is outside of the polygon, skipping.");
    setFrontierUnachievable(frontier);
    return;
  }

  // Process information gain if needed
  if (information_gain_plugin_) {
    information_gain_plugin_->setInformationGainForFrontier(frontier, polygon_xy_min_max_);
  }

  // Process planning if needed
  if (planner_plugin_) {
    planner_plugin_->setPlanForFrontier(start_pose_w, frontier);
  }

  recomputeNormalizationFactors(frontier);
}

void CostAssigner::recomputeNormalizationFactors(FrontierPtr & frontier)
{
  if (!frontier->isAchievable()) {
    return;
  }

  min_traversable_distance = std::min(min_traversable_distance, frontier->getPathLength());
  max_traversable_distance = std::max(max_traversable_distance, frontier->getPathLength());
  min_arrival_info_per_frontier = std::min(
    min_arrival_info_per_frontier,
    frontier->getArrivalInformation());
  max_arrival_info_per_frontier = std::max(
    max_arrival_info_per_frontier,
    frontier->getArrivalInformation());
}

void CostAssigner::setFrontierUnachievable(FrontierPtr & frontier)
{
  frontier->setAchievability(false);
  frontier->setArrivalInformation(0.0);
  frontier->setGoalOrientation(0.0);
  frontier->setPathLength(std::numeric_limits<double>::max());
  frontier->setPathLengthInM(std::numeric_limits<double>::max());
  frontier->setWeightedCost(std::numeric_limits<double>::max());
}
}
