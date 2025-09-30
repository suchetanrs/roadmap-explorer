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

bool CostAssigner::getFrontierCosts(
  std::shared_ptr<GetFrontierCostsRequest> requestData,
  std::shared_ptr<GetFrontierCostsResponse> resultData)
{
  setFrontierBlacklist(requestData->prohibited_frontiers);
  rosVisualizerInstance.visualizeBlacklistedFrontierMarkers("blacklisted_frontiers", requestData->prohibited_frontiers);
  bool costsResult =
    processChosenApproach(requestData->frontier_list, requestData->start_pose.pose, requestData->force_grid_base_planning);
  if (costsResult == false) {
    resultData->success = false;
    return resultData->success;
  }
  resultData->success = true;
  std::vector<FrontierPtr> frontiers_list;
  std::vector<double> frontier_distances;
  std::vector<double> frontier_arrival_information;
  for (auto & frontier : requestData->frontier_list) {
    frontiers_list.push_back(frontier);
    frontier_distances.push_back(frontier->getPathLengthInM());
    frontier_arrival_information.push_back(frontier->getArrivalInformation());
    // LOG_INFO("Cost is: " << resultData->frontier_costs.size();
  }
  LOG_DEBUG("Making list");
  resultData->frontier_list = frontiers_list;
  resultData->frontier_distances = frontier_distances;
  resultData->frontier_arrival_information = frontier_arrival_information;
  if (resultData->frontier_list != requestData->frontier_list) {
    throw RoadmapExplorerException("Lists are not SAME!");
  }
  LOG_DEBUG("Res frontier distances size: " << resultData->frontier_distances.size());
  LOG_DEBUG("Res frontier list size: " << resultData->frontier_list.size());
  return resultData->success;
}

void CostAssigner::updateBoundaryPolygon(geometry_msgs::msg::PolygonStamped & explore_boundary)
{
  // Transform all points of boundary polygon into costmap frame
  geometry_msgs::msg::PointStamped in;
  in.header = explore_boundary.header;
  for (const auto & point32 : explore_boundary.polygon.points) {
    LOG_TRACE(
      "Sending Polygon from config x:" << point32.x << " y: " << point32.y << " z: " <<
        point32.z);
    in.point = nav2_costmap_2d::toPoint(point32);
    polygon_.points.push_back(nav2_costmap_2d::toPoint32(in.point));
  }

  // if empty boundary provided, set to whole map
  if (polygon_.points.empty()) {
    geometry_msgs::msg::Point32 temp;
    temp.x = layered_costmap_->getCostmap()->getOriginX();
    temp.y = layered_costmap_->getCostmap()->getOriginY();
    polygon_.points.push_back(temp);
    temp.y = layered_costmap_->getCostmap()->getSizeInMetersY();
    polygon_.points.push_back(temp);
    temp.x = layered_costmap_->getCostmap()->getSizeInMetersX();
    polygon_.points.push_back(temp);
    temp.y = layered_costmap_->getCostmap()->getOriginY();
    polygon_.points.push_back(temp);
  }

  // Find map size and origin by finding min/max points of polygon
  double min_x_polygon = std::numeric_limits<double>::infinity();
  double min_y_polygon = std::numeric_limits<double>::infinity();
  double max_x_polygon = -std::numeric_limits<double>::infinity();       // observe the minus here
  double max_y_polygon = -std::numeric_limits<double>::infinity();       // observe the minus here

  for (const auto & point : polygon_.points) {
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

bool CostAssigner::processChosenApproach(
  std::vector<FrontierPtr> & frontier_list,
  geometry_msgs::msg::Pose & start_pose_w, bool force_grid_base_planning)
{
  LOG_DEBUG("CostAssigner::processChosenApproach");
  EventLoggerInstance.startEvent("processChosenApproach");

  std::vector<std::string> chosenMethods = parameterInstance.getValue<std::vector<std::string>>(
    "costAssigner.cost_calculation_methods");
  if (force_grid_base_planning)
  {
    // resort to grid-based planning
    chosenMethods.clear();
    chosenMethods.push_back("ArrivalInformation");
    chosenMethods.push_back("A*PlannerDistance");
  }
  LOG_INFO("Methods chosen are: " << chosenMethods);
  std::vector<std::vector<std::string>> costTypes;
  for (auto frontier : frontier_list) {
    // for each frontier, we need to assign the costs that it should use.
    costTypes.push_back(chosenMethods);
    if (force_grid_base_planning)
    {
      // make all frontiers achieveable. They will become unachievable if they cant find a path.
      frontier->setAchievability(true);
      frontier->setBlacklisted(true);
    }
  }
  bool costsResult;
  costsResult = assignCosts(frontier_list, polygon_xy_min_max_, start_pose_w, costTypes);
  if (costsResult == false) {
    LOG_CRITICAL("The result for the chosen method is false!");
    return costsResult;
  }

  EventLoggerInstance.endEvent("processChosenApproach", 1);
  return costsResult;
}

std::vector<FrontierPtr> findDuplicates(const std::vector<FrontierPtr> & vec)
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

bool CostAssigner::assignCosts(
  std::vector<FrontierPtr> & frontier_list, std::vector<double> polygon_xy_min_max,
  geometry_msgs::msg::Pose start_pose_w, std::vector<std::vector<std::string>> & costTypes)
{
  min_traversable_distance = std::numeric_limits<double>::max();
  max_traversable_distance = -1.0 * std::numeric_limits<double>::max();
  min_arrival_info_per_frontier = std::numeric_limits<double>::max();
  max_arrival_info_per_frontier = -1.0 * std::numeric_limits<double>::max();
  LOG_DEBUG("CostAssigner::assignCosts");
  // sanity checks
  if (frontier_list.size() == 0) {
    LOG_ERROR("No frontiers found from frontier search.");
    return false;
  }

  if (polygon_xy_min_max.size() <= 0) {
    LOG_ERROR("FrontierPtr cannot be selected, no polygon.");
    return false;
  }

  // Iterate through each frontier
  LOG_DEBUG("FrontierPtr list size is (loop): " + std::to_string(frontier_list.size()));
  if (findDuplicates(frontier_list).size() > 0) {
    throw RoadmapExplorerException("Duplicate frontiers found.");
  }
  LOG_DEBUG("Blacklist size is: " << frontier_blacklist_.size());
  for (int frontier_idx = 0; frontier_idx < (int)frontier_list.size(); frontier_idx++) {
    auto & frontier = frontier_list[frontier_idx];
    if (frontier_blacklist_.count(frontier) > 0) {
      frontier->setAchievability(false);
      frontier->setArrivalInformation(0.0);
      frontier->setGoalOrientation(0.0);
      frontier->setPathLength(std::numeric_limits<double>::max());
      frontier->setPathLengthInM(std::numeric_limits<double>::max());
      frontier->setWeightedCost(std::numeric_limits<double>::max());
      continue;
    }

    auto goal_point = frontier->getGoalPoint();
    if (goal_point.x < polygon_xy_min_max[0] ||
      goal_point.y<polygon_xy_min_max[1] ||
      goal_point.x> polygon_xy_min_max[2] ||
      goal_point.y > polygon_xy_min_max[3])
    {
      LOG_DEBUG("Frontier is outside of the polygon, skipping.");
      frontier->setAchievability(false);
      frontier->setArrivalInformation(0.0);
      frontier->setGoalOrientation(0.0);
      frontier->setPathLength(std::numeric_limits<double>::max());
      frontier->setPathLengthInM(std::numeric_limits<double>::max());
      frontier->setWeightedCost(std::numeric_limits<double>::max());
      continue;
    }

    /**
         * IMPORTANT! Always compute the arrival information first before the planning.
         * If the frontier is not achievable, the plan is not computed.
         */
    if (vectorContains(costTypes[frontier_idx], std::string("ArrivalInformation"))) {
      try {
        pluginlib::ClassLoader<BaseInformationGain> plugin_loader(
          "roadmap_explorer", "roadmap_explorer::BaseInformationGain");
        
        std::string plugin_name = "roadmap_explorer::CountBasedGain";
        // TODO: Make plugin name configurable via parameter
        
        auto plannerPtr_ = plugin_loader.createSharedInstance(plugin_name);
        plannerPtr_->configure(explore_costmap_ros_);
        plannerPtr_->setInformationGainForFrontier(frontier, polygon_xy_min_max);
        LOG_INFO("Loaded planner plugin: " << plugin_name);
      } catch (const std::exception& e) {
        LOG_WARN("Failed to load planner plugin, using direct instantiation: " << e.what());
        throw std::runtime_error("Failed to load planner plugin");
      }
    }
    if (vectorContains(costTypes[frontier_idx], std::string("A*PlannerDistance"))) {
      try {
        pluginlib::ClassLoader<BasePlanner> plugin_loader(
          "roadmap_explorer", "roadmap_explorer::BasePlanner");
        
        std::string plugin_name = "roadmap_explorer::PluginNavFn";
        // TODO: Make plugin name configurable via parameter
        
        auto plannerPtr_ = plugin_loader.createSharedInstance(plugin_name);
        plannerPtr_->configure(explore_costmap_ros_);
        plannerPtr_->setPlanForFrontier(start_pose_w, frontier);
        LOG_INFO("Loaded planner plugin: " << plugin_name);
      } catch (const std::exception& e) {
        LOG_WARN("Failed to load planner plugin, using direct instantiation: " << e.what());
        throw std::runtime_error("Failed to load planner plugin");
      }
    } else if (vectorContains(costTypes[frontier_idx], std::string("RoadmapPlannerDistance"))) {
      try {
        pluginlib::ClassLoader<BasePlanner> plugin_loader(
          "roadmap_explorer", "roadmap_explorer::BasePlanner");
        
        std::string plugin_name = "roadmap_explorer::PluginFrontierRoadmap";
        // TODO: Make plugin name configurable via parameter
        
        auto plannerPtr_ = plugin_loader.createSharedInstance(plugin_name);
        plannerPtr_->configure(explore_costmap_ros_);
        plannerPtr_->setPlanForFrontier(start_pose_w, frontier);
        LOG_INFO("Loaded planner plugin: " << plugin_name);
      } catch (const std::exception& e) {
        LOG_WARN("Failed to load planner plugin, using direct instantiation: " << e.what());
        throw std::runtime_error("Failed to load planner plugin");
      }

    }
    else if (vectorContains(costTypes[frontier_idx], std::string("EuclideanDistance"))) {
      try {
        pluginlib::ClassLoader<BasePlanner> plugin_loader(
          "roadmap_explorer", "roadmap_explorer::BasePlanner");
        
        std::string plugin_name = "roadmap_explorer::EuclideanDistancePlugin";
        // TODO: Make plugin name configurable via parameter
        
        auto plannerPtr_ = plugin_loader.createSharedInstance(plugin_name);
        plannerPtr_->configure(explore_costmap_ros_);
        plannerPtr_->setPlanForFrontier(start_pose_w, frontier);
        LOG_INFO("Loaded planner plugin: " << plugin_name);
      } catch (const std::exception& e) {
        LOG_WARN("Failed to load planner plugin, using direct instantiation: " << e.what());
        throw std::runtime_error("Failed to load planner plugin");
      }
    }
    if (vectorContains(costTypes[frontier_idx], std::string("RandomCosts"))) {
      try {
        pluginlib::ClassLoader<BasePlanner> plugin_loader(
          "roadmap_explorer", "roadmap_explorer::BasePlanner");
        
        std::string plugin_name = "roadmap_explorer::RandomDistancePlugin";
        // TODO: Make plugin name configurable via parameter
        
        auto plannerPtr_ = plugin_loader.createSharedInstance(plugin_name);
        plannerPtr_->configure(explore_costmap_ros_);
        plannerPtr_->setPlanForFrontier(start_pose_w, frontier);
        LOG_INFO("Loaded planner plugin: " << plugin_name);
      } catch (const std::exception& e) {
        LOG_WARN("Failed to load planner plugin, using direct instantiation: " << e.what());
        throw std::runtime_error("Failed to load planner plugin");
      }
    }
    recomputeNormalizationFactors(frontier);
  }       // frontier end
  LOG_DEBUG("Returning for input list size: " << frontier_list.size());
  return true;
}

void CostAssigner::setFrontierBlacklist(std::vector<FrontierPtr> & blacklist)
{
  std::lock_guard<std::mutex> lock(blacklist_mutex_);
  LOG_DEBUG("Setting frontier blacklist with size: " << blacklist.size());
  for (auto frontier : blacklist) {
    LOG_DEBUG("Adding frontier to blacklist: " << frontier);
    frontier_blacklist_[frontier] = true;
  }
}
}
