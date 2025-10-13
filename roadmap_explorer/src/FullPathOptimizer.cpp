/**
    Copyright 2025 Suchetan Saravanan.

    Licensed to the Apache Software Foundation (ASF) under one
    or more contributor license agreements.  See the NOTICE file
    distributed with this work for additional information
    regarding copyright ownership.  The ASF licenses this file
    to you under the Apache License, Version 2.0 (the
    "License"); you may not use this file except in compliance
    with the License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing,
    software distributed under the License is distributed on an
    "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
    KIND, either express or implied.  See the License for the
    specific language governing permissions and limitations
    under the License.
*/


#include <vector>
#include <algorithm>
#include <limits>
#include "roadmap_explorer/FullPathOptimizer.hpp"
namespace roadmap_explorer
{
FullPathOptimizer::FullPathOptimizer(
  std::shared_ptr<nav2_util::LifecycleNode> node,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros)
: node_(node)
{

  num_frontiers_in_local_area = parameterInstance.getValue<double>(
    "fullPathOptimizer.num_frontiers_in_local_area");
  local_frontier_search_radius = parameterInstance.getValue<double>(
    "fullPathOptimizer.local_frontier_search_radius");
  add_yaw_to_tsp = parameterInstance.getValue<bool>("fullPathOptimizer.add_yaw_to_tsp");
  add_distance_to_robot_to_tsp = parameterInstance.getValue<bool>(
    "fullPathOptimizer.add_distance_to_robot_to_tsp");
  goal_hysteresis_threshold_ = parameterInstance.getValue<double>(
    "fullPathOptimizer.goal_hysteresis_threshold");

  explore_costmap_ros_ = explore_costmap_ros;
  local_search_area_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "local_search_area", 10);
  local_search_area_publisher_->on_activate();
  frontier_nav2_plan_ = node_->create_publisher<nav_msgs::msg::Path>(
    "frontier_roadmap_nav2_plan",
    10);
  frontier_nav2_plan_->on_activate();
}

FullPathOptimizer::~FullPathOptimizer()
{
  LOG_INFO("FullPathOptimizer::~FullPathOptimizer");
  frontier_pair_distances_.clear();
  local_search_area_publisher_->on_deactivate();
  frontier_nav2_plan_->on_deactivate();
  local_search_area_publisher_.reset();
  frontier_nav2_plan_.reset();
}

void FullPathOptimizer::addToMarkerArraySolidPolygon(
  visualization_msgs::msg::MarkerArray & marker_array, geometry_msgs::msg::Point center,
  double radius, std::string ns, float r, float g, float b, int id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = rclcpp::Clock().now();
  marker.header.frame_id = "map";       // Set the frame to map
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CYLINDER;       // Change to CYLINDER
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = center;
  marker.scale.x = radius * 2;       // Ensure this is suitable for a cylinder
  marker.scale.y = radius * 2;
  marker.scale.z = 1.0;
  marker.color.a = 0.30;       // Semi-transparent
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.lifetime = rclcpp::Duration::from_seconds(MARKER_LIFETIME_SECONDS);
  marker_array.markers.push_back(marker);
}

double FullPathOptimizer::calculatePathLength(std::vector<FrontierPtr> & path)
{
  double totalLength = 0.0;
  for (size_t i = 0; i < path.size() - 1; ++i) {
    // LOG_DEBUG("Inside loop i =" << i << "limit " << path.size());
    LOG_TRACE("Need distance between: " << path[i] << " , " << path[i + 1]);
    auto it = frontier_pair_distances_.find(FrontierPair(path[i], path[i + 1]));
    if (it != frontier_pair_distances_.end()) {
      LOG_TRACE(
        "Using cache value bw " << path[i] << " to " << path[i + 1] << ". The value is: " <<
          it->second.path_length_m);
      totalLength += it->second.path_length_m;
      continue;
    }
    auto it2 = frontier_pair_distances_.find(FrontierPair(path[i + 1], path[i]));
    if (it2 != frontier_pair_distances_.end()) {
      LOG_TRACE(
        "Using cache value bw " << path[i + 1] << " to " << path[i] << ". The value is: " <<
          it2->second.path_length_m);
      totalLength += it2->second.path_length_m;
      continue;
    }
    LOG_TRACE("Could not find path in cache. Computing bw " << path[i] << " to " << path[i + 1]);
    auto current_length = frontierRoadmapInstance.getPlan(path[i], true, path[i + 1], true);
    // LOG_DEBUG(current_length);
    if (current_length.path_exists == true) {
      totalLength += current_length.path_length_m;
      frontier_pair_distances_[FrontierPair(path[i], path[i + 1])] = current_length;
      std::reverse(current_length.path.begin(), current_length.path.end());
      frontier_pair_distances_[FrontierPair(path[i + 1], path[i])] = current_length;
    } else if (current_length.path_exists == false) {
      // set it to a large value since path could not be found.
      totalLength += local_frontier_search_radius * PATH_NOT_FOUND_PENALTY_MULTIPLIER;
      frontier_pair_distances_[FrontierPair(path[i], path[i + 1])] = current_length;
      std::reverse(current_length.path.begin(), current_length.path.end());
      frontier_pair_distances_[FrontierPair(path[i + 1], path[i])] = current_length;
    }
  }
  return totalLength;
}

double FullPathOptimizer::calculateLengthRobotToGoal(
  const FrontierPtr & robot,
  const FrontierPtr & goal)
{
  std::vector<FrontierPtr> path = {robot, goal};
  return calculatePathLength(path);
}

// seperates global and local frontiers.
void FullPathOptimizer::getFilteredFrontiers(
  std::vector<FrontierPtr> & frontier_list,
  SortedFrontiers & sortedFrontiers,
  geometry_msgs::msg::PoseStamped & robotP)
{
  double closest_global_frontier_length = std::numeric_limits<double>::max();
  // get achievable frontiers
  for (const auto & frontier : frontier_list) {
    if (frontier->isAchievable() && !frontier->isBlacklisted()) {
      LOG_DEBUG("Path length in m" << frontier->getPathLengthInM());
      if (distanceBetweenPoints(
          frontier->getGoalPoint(),
          robotP.pose.position) <= local_frontier_search_radius)
      {
        LOG_DEBUG("Local Frontiers:");
        sortedFrontiers.local_frontiers.push_back(frontier);
        LOG_DEBUG(frontier);
      } else if (distanceBetweenPoints(
          frontier->getGoalPoint(),
          robotP.pose.position) > local_frontier_search_radius)
      {
        sortedFrontiers.global_frontiers.push_back(frontier);
        LOG_DEBUG("Global Frontiers:");
        LOG_DEBUG(frontier);
        if (frontier->getPathLengthInM() < closest_global_frontier_length) {
          LOG_DEBUG("ADDING CLOSEST FRONTIER");
          closest_global_frontier_length = frontier->getPathLengthInM();
          sortedFrontiers.closest_global_frontier = frontier;
        }
      }
    }
  }
  if (sortedFrontiers.global_frontiers.size() > 0) {
    LOG_DEBUG("Closest global FrontierPtr:");
    LOG_DEBUG(sortedFrontiers.closest_global_frontier);
  }
}

void FullPathOptimizer::getFilteredFrontiersN(
  std::vector<FrontierPtr> & frontier_list, int n,
  SortedFrontiers & sortedFrontiers,
  geometry_msgs::msg::PoseStamped & robotP)
{
  // If there are no global frontiers, and local <= n then upto n - 1 are added to local and nth is treated as closest global
  // If there are no global frontiers, and local > n then all up to n are added to local vector and the farthest one is treated as closest_global_frontier
  // If there are no local frontiers, add all to global vector and treat closest one as closest_global_frontier
  // If there are both local and global, and local <= n, then all local added to local vector and all global added to global vector with closest one being closest_g_f
  // If there are both local and global, and local > n, then upto n are added to local vector and global ones are added to global vector. Closest global = closest_g_f
  std::vector<FrontierPtr> all_frontiers;

  for (const auto & frontier : frontier_list) {
    if (frontier->isAchievable() && !frontier->isBlacklisted()) {
      double pathLength = frontier->getPathLengthInM();
      all_frontiers.push_back(frontier);
      LOG_DEBUG("Path length in m: " << pathLength);
    }
  }
  // Sort all frontiers based on path length
  std::sort(
    all_frontiers.begin(), all_frontiers.end(),
    [robotP](const FrontierPtr & a, const FrontierPtr & b)
    {
      // return distanceBetweenPoints(a->getGoalPoint(), robotP.pose.position) < distanceBetweenPoints(b->getGoalPoint(), robotP.pose.position);
      return a->getPathLengthInM() < b->getPathLengthInM();
    });

  // Separate the frontiers into local and global lists.
  bool global_assigned = false;
  // to cater first case.
  if (all_frontiers.size() == 0) {
    LOG_WARN("No achievable frontiers exist during sorting.");
    return;
  }
  int counter = 1;
  bool need_to_pop = false;
  for (const auto & frontier : all_frontiers) {
    // LOG_INFO("Path length in m" << frontier->getPathLengthInM());
    if (frontier->getPathLengthInM() <= local_frontier_search_radius && counter <= n) {
      sortedFrontiers.local_frontiers.push_back(frontier);
      LOG_DEBUG("Local frontier candidate: " << frontier);
      sortedFrontiers.closest_global_frontier = frontier;
      need_to_pop = true;
    } else if (frontier->getPathLengthInM() <= local_frontier_search_radius && counter > n) {
      sortedFrontiers.closest_global_frontier = frontier;
      need_to_pop = false;
    } else if (frontier->getPathLengthInM() > local_frontier_search_radius) {
      if (!global_assigned) {
        sortedFrontiers.closest_global_frontier = frontier;
        global_assigned = true;
      }
      need_to_pop = false;
      sortedFrontiers.global_frontiers.push_back(frontier);
      LOG_DEBUG("Global frontier candidate: " << frontier);
    }
    counter++;
  }
  if (need_to_pop) {
    sortedFrontiers.local_frontiers.pop_back();
  }
  if (sortedFrontiers.global_frontiers.size() == 0 && sortedFrontiers.local_frontiers.size() == 0) {
    sortedFrontiers.global_frontiers.push_back(sortedFrontiers.closest_global_frontier);
  }
}

bool FullPathOptimizer::getBestFullPath(
  SortedFrontiers & sortedFrontiers,
  std::vector<FrontierPtr> & bestFrontierWaypoint,
  geometry_msgs::msg::PoseStamped & robotP)
{
  std::vector<std::vector<FrontierPtr>> bestPaths;
  // Create a MarkerArray
  visualization_msgs::msg::MarkerArray marker_array;
  geometry_msgs::msg::PoseStamped robotPose = robotP;
  int id = 0;
  for (auto & lf : sortedFrontiers.local_frontiers) {
    id++;
    addToMarkerArraySolidPolygon(
      marker_array,
      lf->getGoalPoint(), 0.3, "local_frontier", 0.5, 1.0, 0.5, id);
  }

  addToMarkerArraySolidPolygon(
    marker_array,
    sortedFrontiers.closest_global_frontier->getGoalPoint(), 0.3, "closest_global_frontier", 1.0, 0.5, 0.3,
    0);
  LOG_INFO(
    "Closest global frontier with reasonable information is: " <<
      sortedFrontiers.closest_global_frontier);
  LOG_INFO("Local frontier list size: " << sortedFrontiers.local_frontiers.size());
  LOG_INFO("Global frontier list size: " << sortedFrontiers.global_frontiers.size());

  local_search_area_publisher_->publish(marker_array);

  double minLength = std::numeric_limits<double>::max();
  FrontierPtr robotPoseFrontier = std::make_shared<Frontier>();
  robotPoseFrontier->setGoalPoint(robotP.pose.position.x, robotP.pose.position.y);
  robotPoseFrontier->setUID(generateUID(robotPoseFrontier));
  robotPoseFrontier->setPathLength(0.0);
  robotPoseFrontier->setPathLengthInM(0.0);

  std::sort(sortedFrontiers.local_frontiers.begin(), sortedFrontiers.local_frontiers.end());
  LOG_DEBUG("************************");
  LOG_DEBUG("************************");
  LOG_DEBUG("************************");
  LOG_DEBUG("************************");
  LOG_DEBUG("************************");
  do{
    sortedFrontiers.local_frontiers.insert(
      sortedFrontiers.local_frontiers.begin(), robotPoseFrontier);

    // LOG_DEBUG("Inserting closest global frontier to the end.");
    sortedFrontiers.local_frontiers.push_back(sortedFrontiers.closest_global_frontier);
    // add robot pose to the end to complete the TSP.
    // sortedFrontiers.local_frontiers.push_back(robotPoseFrontier);
    LOG_DEBUG("Computing paths for permutation: " << sortedFrontiers.local_frontiers);

    double currentLength = calculatePathLength(sortedFrontiers.local_frontiers);

    if (add_yaw_to_tsp) {
      auto robot_yaw = quatToEuler(robotP.pose.orientation)[2];
      if (robot_yaw < 0) {
        robot_yaw = robot_yaw + (M_PI * 2);
      }
      double goal_yaw = atan2(
        sortedFrontiers.local_frontiers[1]->getGoalPoint().y - robotP.pose.position.y,
        sortedFrontiers.local_frontiers[1]->getGoalPoint().x - robotP.pose.position.x);
      if (goal_yaw < 0) {
        goal_yaw = goal_yaw + (M_PI * 2);
      }
      double path_heading = abs(robot_yaw - goal_yaw);
      if (path_heading > M_PI) {
        path_heading = path_heading - (2 * M_PI);
      }

      currentLength += (abs(path_heading) * YAW_COST_WEIGHT);
    }

    if (add_distance_to_robot_to_tsp) {
      auto distance_to_add = distanceBetweenFrontiers(
        robotPoseFrontier,
        sortedFrontiers.local_frontiers[1]);
      currentLength += distance_to_add;
    }
    LOG_DEBUG("Path length:" << currentLength);
    if (currentLength < minLength) {
      LOG_DEBUG("Currently tracking min length: " << currentLength);
      minLength = currentLength;
      LOG_DEBUG("Clearing best paths.");
      bestPaths.clear();           // Clear the previous best paths
      LOG_DEBUG("Pushing " << sortedFrontiers.local_frontiers << " to path.");
      bestPaths.push_back(sortedFrontiers.local_frontiers);           // Add the new best path
    } else if (currentLength == minLength) {
      LOG_DEBUG("Pushing " << sortedFrontiers.local_frontiers << " to path.");
      bestPaths.push_back(sortedFrontiers.local_frontiers);           // Add path if it matches the current min length
    }

    // erase the first element (robot position) to prepare for next permutation
    sortedFrontiers.local_frontiers.erase(sortedFrontiers.local_frontiers.begin());

    // erase the global frontier at the end of vector to prepare for next permutation
    sortedFrontiers.local_frontiers.pop_back();

    // remove the robot pose frontier.
    // sortedFrontiers.local_frontiers.pop_back();

    LOG_DEBUG("====permutation ended====")

  } while (std::next_permutation(
    sortedFrontiers.local_frontiers.begin(),
    sortedFrontiers.local_frontiers.end()) && rclcpp::ok());
  if (minLength >= local_frontier_search_radius * PATH_NOT_FOUND_PENALTY_MULTIPLIER) {
    LOG_ERROR("Zero frontiers were reasonable post FI check...returning zero frontier.");
    return false;
  }
  LOG_INFO("Number of best minimum paths: " << bestPaths.size());
  minLength = std::numeric_limits<double>::max();
  LOG_DEBUG("Getting distance with Robot Pose: " << robotPoseFrontier);
  // this is useful if there are more than one paths with the same length.
  for (const auto & path : bestPaths) {
    LOG_DEBUG("Getting distance with local frontiers: " << path);
    auto distance_to_get_minima = calculateLengthRobotToGoal(robotPoseFrontier, path[1]);
    LOG_DEBUG("Distance is: " << distance_to_get_minima);
    if (distance_to_get_minima < minLength) {
      minLength = distance_to_get_minima;
      bestFrontierWaypoint = path;
    }
  }
  return true;
}

bool FullPathOptimizer::getNextGoal(
  std::vector<FrontierPtr> & frontier_list,
  FrontierPtr & nextFrontier,
  geometry_msgs::msg::PoseStamped & robotP)
{
  num_frontiers_in_local_area = parameterInstance.getValue<double>(
    "fullPathOptimizer.num_frontiers_in_local_area");
  local_frontier_search_radius = parameterInstance.getValue<double>(
    "fullPathOptimizer.local_frontier_search_radius");
  add_yaw_to_tsp = parameterInstance.getValue<bool>("fullPathOptimizer.add_yaw_to_tsp");
  add_distance_to_robot_to_tsp = parameterInstance.getValue<bool>(
    "fullPathOptimizer.add_distance_to_robot_to_tsp");
  goal_hysteresis_threshold_ = parameterInstance.getValue<double>(
    "fullPathOptimizer.goal_hysteresis_threshold");

  // Check if current committed goal is still valid
  bool current_goal_valid = false;
  if (has_committed_goal_ && current_committed_goal_) {
    // Check if the committed goal still exists in the frontier list and is valid
    auto it = std::find_if(frontier_list.begin(), frontier_list.end(),
      [this](const FrontierPtr& f) {
        return f->getUID() == current_committed_goal_->getUID();
      });
    
    if (it != frontier_list.end() && (*it)->isAchievable() && !(*it)->isBlacklisted()) {
      current_goal_valid = true;
      LOG_INFO("Current committed goal is still valid (UID: " << current_committed_goal_->getUID() << ")");
    } else {
      LOG_INFO("Current committed goal is no longer valid - will compute new goal");
      has_committed_goal_ = false;
    }
  }

  SortedFrontiers sortedFrontiers;
  // sort based on path length
  getFilteredFrontiersN(frontier_list, num_frontiers_in_local_area, sortedFrontiers, robotP);

  FrontierPtr zeroFrontier = std::make_shared<Frontier>();
  std::vector<FrontierPtr> bestFrontierWaypoint;
  if (sortedFrontiers.local_frontiers.size() == 0) {
    // LOG_ERROR("Could not find local frontiers. Returning a zero frontiers. The program may crash if goal point is checked...");
    if (sortedFrontiers.global_frontiers.size() >= 1) {
      LOG_WARN(
        "Global repositioning!! -> Could not find local frontiers. Returning the best global frontier.");
      nextFrontier = sortedFrontiers.closest_global_frontier;

      nav_msgs::msg::Path globalReposPath;
      globalReposPath.header.frame_id = "map";
      globalReposPath.header.stamp = node_->now();
      rosVisualizerInstance.visualizePath("full_path", globalReposPath);

      computePathBetweenPointsThetaStar(
        globalReposPath,
        nextFrontier->getGoalPoint(),
        robotP.pose.position, true, explore_costmap_ros_->getCostmap());

      rosVisualizerInstance.visualizePath("global_repositioning_path", globalReposPath);

      // Update committed goal since we're switching to global repositioning
      current_committed_goal_ = nextFrontier;
      has_committed_goal_ = true;
      current_goal_path_cost_ = nextFrontier->getPathLengthInM();
      LOG_INFO("Committing to global repositioning goal (UID: " << nextFrontier->getUID() << ")");

      return true;
    } else {
      LOG_ERROR(
        "Could not find local or global frontiers. Returning a zero frontier. The program may crash if goal point is checked...");
      nextFrontier = zeroFrontier;
      has_committed_goal_ = false;
      return false;
    }
  }

  if (!getBestFullPath(sortedFrontiers, bestFrontierWaypoint, robotP)) {
    nextFrontier = zeroFrontier;
    has_committed_goal_ = false;
    return false;
  }

  LOG_INFO("Best full path points: " << bestFrontierWaypoint);
  std::vector<std::shared_ptr<Node>> bestPathViz;
  // LOG_INFO("Best full path to follow: ");
  EventLoggerInstance.startEvent("publishPlan");
  if (rosVisualizerInstance.getNumSubscribers("full_path") > 0) {
    nav_msgs::msg::Path bestPathROS;
    bestPathROS.header.frame_id = "map";
    bestPathROS.header.stamp = node_->now();
    rosVisualizerInstance.visualizePath("global_repositioning_path", bestPathROS);
    for (int o = 0; o < (int)bestFrontierWaypoint.size() - 1; o++) {
      // for (auto &fullPoint : frontier_pair_distances_[FrontierPair(bestFrontierWaypoint[o], bestFrontierWaypoint[o + 1])].path)
      // {
      //     LOG_INFO(fullPoint->frontier << " , ");
      // }
      // computePathBetweenPoints(bestPathROS, bestFrontierWaypoint[o]->getGoalPoint(), bestFrontierWaypoint[o+1]->getGoalPoint(), true, explore_costmap_ros_->getCostmap());
      computePathBetweenPointsThetaStar(
        bestPathROS,
        bestFrontierWaypoint[o]->getGoalPoint(),
        bestFrontierWaypoint[o + 1]->getGoalPoint(), true, explore_costmap_ros_->getCostmap());
      // bestPathViz.insert(bestPathViz.end(), frontier_pair_distances_[FrontierPair(bestFrontierWaypoint[o], bestFrontierWaypoint[o + 1])].path.begin(), frontier_pair_distances_[FrontierPair(bestFrontierWaypoint[o], bestFrontierWaypoint[o + 1])].path.end());
    }
    // frontierRoadmapInstance.publishPlan(bestPathViz, 1.0, 0.0, 0.0);
    rosVisualizerInstance.visualizePath("full_path", bestPathROS);
  }
  EventLoggerInstance.endEvent("publishPlan", 2);
  
  // 0 is robot pose. Return the first frontier in the path.
  FrontierPtr candidateGoal = bestFrontierWaypoint[1];
  
  // Apply hysteresis logic to prevent frequent goal switching
  if (current_goal_valid) {
    // Calculate cost to reach the current committed goal
    FrontierPtr robotPoseFrontier = std::make_shared<Frontier>();
    robotPoseFrontier->setGoalPoint(robotP.pose.position.x, robotP.pose.position.y);
    robotPoseFrontier->setUID(generateUID(robotPoseFrontier));
    
    double cost_to_current_goal = calculateLengthRobotToGoal(robotPoseFrontier, current_committed_goal_);
    double cost_to_new_goal = calculateLengthRobotToGoal(robotPoseFrontier, candidateGoal);
    
    // Calculate improvement ratio
    double improvement = (cost_to_current_goal - cost_to_new_goal) / cost_to_current_goal;
    
    LOG_INFO("Hysteresis check - Current goal cost: " << cost_to_current_goal << 
             ", New goal cost: " << cost_to_new_goal << 
             ", Improvement: " << (improvement * 100.0) << "%");
    
    if (improvement >= goal_hysteresis_threshold_) {
      // New goal is significantly better - switch to it
      LOG_INFO("New goal is " << (improvement * 100.0) << "% better (threshold: " << 
               (goal_hysteresis_threshold_ * 100.0) << "%) - switching goals");
      LOG_INFO("Switching from goal UID " << current_committed_goal_->getUID() << 
               " to " << candidateGoal->getUID());
      current_committed_goal_ = candidateGoal;
      current_goal_path_cost_ = cost_to_new_goal;
      nextFrontier = candidateGoal;
    } else {
      // Stick with current goal
      LOG_INFO("Sticking with current goal (improvement " << (improvement * 100.0) << 
               "% below threshold " << (goal_hysteresis_threshold_ * 100.0) << "%)");
      nextFrontier = current_committed_goal_;
    }
  } else {
    // No current goal or it became invalid - commit to the new candidate
    LOG_INFO("No valid committed goal - committing to new goal (UID: " << candidateGoal->getUID() << ")");
    current_committed_goal_ = candidateGoal;
    has_committed_goal_ = true;
    FrontierPtr robotPoseFrontier = std::make_shared<Frontier>();
    robotPoseFrontier->setGoalPoint(robotP.pose.position.x, robotP.pose.position.y);
    robotPoseFrontier->setUID(generateUID(robotPoseFrontier));
    current_goal_path_cost_ = calculateLengthRobotToGoal(robotPoseFrontier, candidateGoal);
    nextFrontier = candidateGoal;
  }
  
  return true;
}

bool FullPathOptimizer::refineAndPublishPath(
  geometry_msgs::msg::PoseStamped & robotP,
  FrontierPtr & goalFrontier, nav_msgs::msg::Path & refined_path)
{
  nav_msgs::msg::Path nav2_plan;
  nav2_plan.header.frame_id = "map";
  nav2_plan.header.stamp = node_->now();
  if (!computePathBetweenPointsThetaStar(
      nav2_plan, robotP.pose.position,
      goalFrontier->getGoalPoint(), true, explore_costmap_ros_->getCostmap()))
  {
    return false;
  }
  refined_path = nav2_plan;
  frontier_nav2_plan_->publish(nav2_plan);
  return true;
}
}
