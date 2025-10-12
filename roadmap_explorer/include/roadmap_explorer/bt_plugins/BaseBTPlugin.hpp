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

#pragma once

#ifdef ROS_DISTRO_HUMBLE
  #include <behaviortree_cpp_v3/bt_factory.h>
#elif ROS_DISTRO_JAZZY || ROS_DISTRO_KILTED
  #include <behaviortree_cpp/bt_factory.h>
#else
  #error "Unsupported ROS distro"
#endif

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include "roadmap_explorer/bt_plugins/BTContext.hpp"

namespace roadmap_explorer
{

enum class ExplorationErrorCode
{
  NO_FRONTIERS_IN_CURRENT_RADIUS,
  MAX_FRONTIER_SEARCH_RADIUS_EXCEEDED,
  COST_COMPUTATION_FAILURE,
  NO_ACHIEVABLE_FRONTIERS_LEFT,
  FULL_PATH_OPTIMIZATION_FAILURE,
  REFINED_PATH_COMPUTATION_FAILURE,
  UNHANDLED_ERROR,
  NAV2_GOAL_ABORT,
  NO_ERROR
};

class BTPlugin
{
  public:
  virtual ~BTPlugin() = default;

  virtual void registerNodes(BT::BehaviorTreeFactory & factory, std::shared_ptr<BTContext> context) = 0;
};

}  // namespace roadmap_explorer