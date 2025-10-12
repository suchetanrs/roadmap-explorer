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

#ifndef NAV_FN_PLANNER_HPP_
#define NAV_FN_PLANNER_HPP_

#include <cmath>
#include <chrono>
#include <vector>
#include <queue>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include <nav2_navfn_planner/navfn.hpp>

#include "roadmap_explorer/util/Logger.hpp"


namespace roadmap_explorer
{

bool computePathBetweenPointsThetaStar(
  nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Point & start_point,
  const geometry_msgs::msg::Point & goal_point,
  bool planner_allow_unknown,
  nav2_costmap_2d::Costmap2D * exploration_costmap_);

} // namespace roadmap_explorer

#endif // NAV_FN_PLANNER_HPP_