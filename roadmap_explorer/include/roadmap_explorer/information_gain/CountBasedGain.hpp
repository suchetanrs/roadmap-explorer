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

#ifndef COUNT_BASED_GAIN_HPP_
#define COUNT_BASED_GAIN_HPP_

#include <limits>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "roadmap_explorer/information_gain/BaseInformationGain.hpp"
#include "roadmap_explorer/Frontier.hpp"
#include "roadmap_explorer/Helpers.hpp"
#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/Parameters.hpp"

namespace roadmap_explorer
{

/**
 * @brief Count-based information gain calculator
 *
 * This implementation calculates information gain by counting unknown cells
 * visible through raytracing from a frontier position. It uses a camera model
 * with configurable field of view and depth to simulate sensor observations.
 */
class CountBasedGain : public BaseInformationGain
{
public:
  /**
   * @brief Default constructor
   */
  CountBasedGain();

  /**
   * @brief Virtual destructor
   */
  ~CountBasedGain() override;

  /**
   * @brief Configure the information gain calculator with costmap
   * @param explore_costmap_ros Shared pointer to the costmap ROS wrapper
   */
  void configure(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, std::string name, std::shared_ptr<nav2_util::LifecycleNode> node) override;

  /**
   * @brief Reset the calculator state
   */
  void reset() override;

  /**
   * @brief Calculate and set information gain for a frontier using ray tracing
   * @param frontier The frontier to calculate information gain for
   * @param polygon_xy_min_max Boundary polygon limits [x_min, y_min, x_max, y_max]
   */
  void setInformationGainForFrontier(
    FrontierPtr & frontier,
    std::vector<double> & polygon_xy_min_max) override;

private:

  double setArrivalInformationLimits();

  bool arrival_info_limits_set_ = false;
  nav2_costmap_2d::Costmap2D * exploration_costmap_ = nullptr;
  double min_arrival_info_gt_ = std::numeric_limits<double>::max();
  double max_arrival_info_gt_ = -1.0 * std::numeric_limits<double>::max();

  double MAX_CAMERA_DEPTH;
  double DELTA_THETA;
  double CAMERA_FOV;
  double FACTOR_OF_MAX_IS_MIN_ = 0.70;
};

}  // namespace roadmap_explorer

#endif  // COUNT_BASED_GAIN_HPP_
