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

#ifndef FULL_PATH_OPTIMIZER_HPP_
#define FULL_PATH_OPTIMIZER_HPP_

#include <vector>
#include <algorithm>
#include <limits>

#include <visualization_msgs/msg/marker_array.hpp>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

#include "roadmap_explorer/util/GeometryUtils.hpp"
#include "roadmap_explorer/planners/FrontierRoadmap.hpp"
#include "roadmap_explorer/planners/NavFn.hpp"
#include "roadmap_explorer/planners/ThetaStar.hpp"


#include "roadmap_explorer/Parameters.hpp"
#include "roadmap_explorer/Frontier.hpp"
namespace roadmap_explorer
{

struct FrontierPair
{
  // Constructor
  FrontierPair(FrontierPtr f1_, FrontierPtr f2_)
  : f1(f1_), f2(f2_) {}

  FrontierPtr f1;
  FrontierPtr f2;

  // Custom operator< for ordering points in the map
  bool operator<(const FrontierPair & other) const
  {
    // Compare f1 and f2 in lexicographical order
    if (f1->getUID() != other.f1->getUID()) {
      return f1->getUID() < other.f1->getUID();
    }
    return f2->getUID() < other.f2->getUID();
  }

  bool operator==(const FrontierPair & other) const
  {
    // Compare f1 and f2 in lexicographical order
    return f2->getGoalPoint() == other.f2->getGoalPoint() &&
           f1->getGoalPoint() == other.f1->getGoalPoint();
  }
};

// Custom hash function for FrontierPair
struct FrontierPairHash
{
  std::size_t operator()(const FrontierPair & fp) const
  {
    // Combine the hash of both FrontierPtr objects
    std::size_t h1 = std::hash<int>{}(fp.f1->getUID());
    std::size_t h2 = std::hash<int>{}(fp.f2->getUID());

    // Hash combination technique (can vary)
    return h1 ^ (h2 << 1);         // Example of hash combining
  }
};

struct SortedFrontiers
{
  std::vector<FrontierPtr> local_frontiers;
  std::vector<FrontierPtr> global_frontiers;
  FrontierPtr closest_global_frontier;
};

/**
 * @class FullPathOptimizer
 * @brief Optimizes exploration paths using a Traveling Salesman Problem (TSP) approach
 * 
 * This class finds the optimal order to visit multiple frontiers by considering
 * path lengths, yaw changes, and local vs global frontier classification.
 */
class FullPathOptimizer
{
public:
  /**
   * @brief Constructor for FullPathOptimizer
   * @param node Shared pointer to the lifecycle node
   * @param explore_costmap_ros Shared pointer to the exploration costmap
   */
  FullPathOptimizer(
    std::shared_ptr<nav2_util::LifecycleNode> node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros);

  /**
   * @brief Destructor
   */
  ~FullPathOptimizer();

  /**
   * @brief Computes the next optimal frontier to visit
   * @param frontier_list List of available frontiers to consider
   * @param nextFrontier Output parameter for the selected next frontier
   * @param robotP Current robot pose
   * @return true if a valid next frontier was found, false otherwise
   */
  bool getNextGoal(
    std::vector<FrontierPtr> & frontier_list, FrontierPtr & nextFrontier,
    geometry_msgs::msg::PoseStamped & robotP);

  /**
   * @brief Clears the cached path distances between frontier pairs
   */
  void clearPlanCache()
  {
    frontier_pair_distances_.clear();
  }

  /**
   * @brief Calculates the path length between robot position and goal frontier
   * @param robot Frontier representing the robot's current position
   * @param goal Goal frontier to reach
   * @return Path length in meters
   */
  double calculateLengthRobotToGoal(
    const FrontierPtr & robot, const FrontierPtr & goal);

  /**
   * @brief Refines and publishes the path from robot to goal frontier using Theta* planner
   * @param robotP Current robot pose
   * @param goalFrontier Target frontier
   * @param refined_path Output parameter containing the refined path
   * @return true if path was successfully computed, false otherwise
   */
  bool refineAndPublishPath(
    geometry_msgs::msg::PoseStamped & robotP, FrontierPtr & goalFrontier,
    nav_msgs::msg::Path & refined_path);

private:
  void addToMarkerArraySolidPolygon(
    visualization_msgs::msg::MarkerArray & marker_array,
    geometry_msgs::msg::Point center, double radius, std::string ns,
    float r, float g, float b, int id);

  double calculatePathLength(std::vector<FrontierPtr> & path);

  void getFilteredFrontiers(
    std::vector<FrontierPtr> & frontier_list,
    SortedFrontiers & sortedFrontiers,
    geometry_msgs::msg::PoseStamped & robotP);

  void getFilteredFrontiersN(
    std::vector<FrontierPtr> & frontier_list, int n,
    SortedFrontiers & sortedFrontiers,
    geometry_msgs::msg::PoseStamped & robotP);

  bool getBestFullPath(
    SortedFrontiers & sortedFrontiers, std::vector<FrontierPtr> & bestPath,
    geometry_msgs::msg::PoseStamped & robotP);

  std::shared_ptr<nav2_util::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    local_search_area_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr frontier_nav2_plan_;
  std::unordered_map<FrontierPair, RoadmapPlanResult, FrontierPairHash> frontier_pair_distances_;

  double num_frontiers_in_local_area = 5.0;
  double local_frontier_search_radius = 12.0; // 6.0 in m
  bool add_yaw_to_tsp = false;
  bool add_distance_to_robot_to_tsp = false;

  // Goal hysteresis variables to prevent frequent goal switching
  FrontierPtr current_committed_goal_;  // The goal we're currently committed to
  double current_goal_path_cost_;       // Path cost to the current goal
  bool has_committed_goal_ = false;     // Whether we have a committed goal
  
  // Hysteresis parameters
  double goal_hysteresis_threshold_ = 0.15;  // New goal must be 15% better to switch
  
  // Constants for path optimization
  static constexpr double PATH_NOT_FOUND_PENALTY_MULTIPLIER = 100000.0;
  static constexpr double YAW_COST_WEIGHT = 2.3;
  static constexpr double MARKER_LIFETIME_SECONDS = 2.5;
};
}

#endif
