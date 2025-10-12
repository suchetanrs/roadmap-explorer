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

#ifndef BASE_PLANNER_HPP_
#define BASE_PLANNER_HPP_

#include <memory>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "roadmap_explorer/Frontier.hpp"
#include "roadmap_explorer/util/RosVisualizer.hpp"
#include "roadmap_explorer/Parameters.hpp"

namespace roadmap_explorer
{

/**
 * @brief Base class for path planning algorithms
 *
 * This abstract base class defines the interface that all path planning
 * algorithms must implement. It provides a plugin-based architecture for
 * different planning strategies used in frontier exploration.
 */
class BasePlanner
{
public:
  /**
   * @brief Default constructor
   */
  BasePlanner() = default;

  /**
   * @brief Virtual destructor
   */
  virtual ~BasePlanner() = default;

  /**
   * @brief Configure the planner with necessary components
   * @param explore_costmap_ros Shared pointer to the costmap ROS wrapper
   */
  virtual void configure(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, std::string name, std::shared_ptr<nav2_util::LifecycleNode> node) = 0;

  /**
   * @brief Reset the planner state
   *
   * Clears any internal state or cached data from previous planning operations.
   */
  virtual void reset() = 0;

  /**
   * @brief Plan a path from start pose to frontier goal
   * @param start_pose_w Starting pose in world coordinates
   * @param goal_frontier Target frontier to plan to
   */
  virtual void setPlanForFrontier(
    const geometry_msgs::msg::Pose start_pose_w,
    FrontierPtr & goal_point_w) = 0;

protected:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
  double closeness_rejection_threshold_ = 0.5;
  double max_planning_distance_ = 50.0;
  bool planner_allow_unknown_ = false;
};

}  // namespace roadmap_explorer

#endif  // BASE_PLANNER_HPP_