// plugin_interface.hpp
#pragma once

#ifdef ROS_DISTRO_HUMBLE
  #include <behaviortree_cpp_v3/bt_factory.h>
#elif ROS_DISTRO_JAZZY
  #include <behaviortree_cpp/bt_factory.h>
#else
  #error "Unsupported ROS distro"
#endif

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"

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

  virtual void registerNodes(BT::BehaviorTreeFactory & factory, std::shared_ptr<nav2_util::LifecycleNode> node, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros) = 0;
};

}  // namespace roadmap_explorer