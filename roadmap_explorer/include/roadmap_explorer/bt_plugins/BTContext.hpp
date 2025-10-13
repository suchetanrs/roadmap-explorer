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

#ifndef ROADMAP_EXPLORER__BT_PLUGINS__BT_CONTEXT_HPP_
#define ROADMAP_EXPLORER__BT_PLUGINS__BT_CONTEXT_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

// Forward declarations
namespace roadmap_explorer
{
  class CostAssigner;
  class FrontierSearchBase;
  class FullPathOptimizer;
  template<typename T>
  class Nav2Interface;
}

namespace nav2_msgs::action
{
  class NavigateToPose;
}

namespace roadmap_explorer
{

/**
 * @brief Context object that holds all shared resources for BT plugins
 * 
 * This allows plugins to access shared resources without polluting the 
 * plugin interface with individual pointers. New resources can be added
 * here without breaking the plugin interface.
 */
struct BTContext
{
  // Core ROS resources
  std::shared_ptr<nav2_util::LifecycleNode> node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  
  // Exploration-specific resources
  std::shared_ptr<CostAssigner> cost_assigner;
  std::shared_ptr<FrontierSearchBase> frontier_search;
  std::shared_ptr<FullPathOptimizer> full_path_optimizer;
  std::shared_ptr<Nav2Interface<nav2_msgs::action::NavigateToPose>> nav2_interface;
  
  // Constructor
  BTContext() = default;
  
  /**
   * @brief Get a resource by type (convenience method)
   */
  template<typename T>
  std::shared_ptr<T> get() const
  {
    if constexpr (std::is_same_v<T, nav2_util::LifecycleNode>) {
      return node;
    } else if constexpr (std::is_same_v<T, nav2_costmap_2d::Costmap2DROS>) {
      return explore_costmap_ros;
    } else if constexpr (std::is_same_v<T, tf2_ros::Buffer>) {
      return tf_buffer;
    } else if constexpr (std::is_same_v<T, CostAssigner>) {
      return cost_assigner;
    } else if constexpr (std::is_same_v<T, FrontierSearchBase>) {
      return frontier_search;
    } else if constexpr (std::is_same_v<T, FullPathOptimizer>) {
      return full_path_optimizer;
    } else {
      static_assert(sizeof(T) == 0, "Type not supported in BTContext");
      return nullptr;
    }
  }
};

}  // namespace roadmap_explorer

#endif  // ROADMAP_EXPLORER__BT_PLUGINS__BT_CONTEXT_HPP_
