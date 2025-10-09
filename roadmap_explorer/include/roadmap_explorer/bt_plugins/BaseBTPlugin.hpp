// plugin_interface.hpp
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

namespace roadmap_explorer
{

class BTPlugin
{
  public:
  virtual ~BTPlugin() = default;

  virtual void registerNodes(BT::BehaviorTreeFactory & factory, std::shared_ptr<nav2_util::LifecycleNode> node, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, std::shared_ptr<tf2_ros::Buffer> tf_buffer) = 0;
};

}  // namespace roadmap_explorer