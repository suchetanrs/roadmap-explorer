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

#ifndef FRONTIER_SEARCH_BASE_HPP_
#define FRONTIER_SEARCH_BASE_HPP_

#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "roadmap_explorer/Frontier.hpp"

namespace roadmap_explorer
{

/**
 * @brief Enumeration of possible results from frontier search operations
 */
enum class FrontierSearchResult
{
  ROBOT_OUT_OF_BOUNDS = 0,        ///< Robot position is outside the costmap bounds
  CANNOT_FIND_CELL_TO_SEARCH = 1, ///< No valid cells found to start the search
  SUCCESSFUL_SEARCH = 2,           ///< Search completed successfully
  NO_FRONTIERS_FOUND = 3           ///< No frontiers detected in the search area
};

/**
 * @brief Base class for frontier search algorithms
 *
 * This abstract base class defines the interface that all frontier search
 * algorithms must implement. It provides a plugin-based architecture for
 * different frontier detection strategies.
 */
class FrontierSearchBase
{
public:
  /**
   * @brief Default constructor
   */
  FrontierSearchBase() = default;

  /**
   * @brief Virtual destructor
   */
  virtual ~FrontierSearchBase() = default;

  /**
   * @brief Configure the frontier search with a costmap
   * @param explore_costmap_ros Shared pointer to the costmap ROS wrapper for frontier detection
   * @param name Name prefix for parameter namespace
   * @param node Shared pointer to the lifecycle node for parameter management
   */
  virtual void configure(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, std::string name, std::shared_ptr<nav2_util::LifecycleNode> node) = 0;

  /**
   * @brief Reset the frontier search state
   *
   * Clears any internal state or cached data from previous searches.
   */
  virtual void reset() = 0;

  /**
   * @brief Search for frontiers from a given position
   * @param position The starting position for the search in world coordinates
   * @param output_frontier_list Output vector to store found "clustered" frontiers (will be cleared and populated)
   * @param max_frontier_search_distance Maximum distance to search for frontiers from the starting position
   * @return Result of the search operation indicating success or failure reason
   */
  virtual FrontierSearchResult searchFrom(
    geometry_msgs::msg::Point position,
    std::vector<FrontierPtr> & output_frontier_list,
    double max_frontier_search_distance) = 0;

  /**
   * @brief Get all frontiers found in the last search operation
   * @return Vector of all frontier points as [x, y] coordinates in world frame
   * @note This method should only be called after a successful searchFrom() operation
   */
  virtual std::vector<std::vector<double>> getAllFrontiers() = 0;

protected:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_ = nullptr; ///< Pointer to the costmap used for frontier detection
  std::shared_ptr<nav2_util::LifecycleNode> node_ = nullptr; ///< Shared pointer to the lifecycle node
};

}  // namespace roadmap_explorer

#endif  // FRONTIER_SEARCH_BASE_HPP_
