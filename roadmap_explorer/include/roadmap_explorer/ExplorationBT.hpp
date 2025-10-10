#ifndef EXPLORE_SERVER_
#define EXPLORE_SERVER_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/parameter.hpp>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_util/node_thread.hpp>
#include <nav2_behavior_tree/plugins/control/pipeline_sequence.hpp>
#include <nav2_behavior_tree/plugins/decorator/rate_controller.hpp>
#include <nav2_behavior_tree/plugins/control/recovery_node.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>

#include <pluginlib/class_loader.hpp>

#ifdef ROS_DISTRO_HUMBLE
  #include <behaviortree_cpp_v3/bt_factory.h>
#elif ROS_DISTRO_JAZZY || ROS_DISTRO_KILTED
  #include <behaviortree_cpp/bt_factory.h>
#else
  #error "Unsupported ROS distro"
#endif

#include "roadmap_explorer/util/RosVisualizer.hpp"
#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/util/EventLogger.hpp"
#include "roadmap_explorer/CostAssigner.hpp"
#include "roadmap_explorer/frontier_search/BaseFrontierSearch.hpp"
#include "roadmap_explorer/frontier_search/PluginBFSearch.hpp"
#include "roadmap_explorer/Nav2Interface.hpp"
#include "roadmap_explorer/Helpers.hpp"
#include "roadmap_explorer/FullPathOptimizer.hpp"
#include "roadmap_explorer/SensorSimulator.hpp"

#include "roadmap_explorer_msgs/action/explore.hpp"
#include "roadmap_explorer/bt_plugins/BaseBTPlugin.hpp"
#include "roadmap_explorer/bt_plugins/BTContext.hpp"

namespace roadmap_explorer
{

using ExploreActionResult = roadmap_explorer_msgs::action::Explore_Result;

class RoadmapExplorationBT
{
public:
  RoadmapExplorationBT(std::shared_ptr<nav2_util::LifecycleNode> node, bool localisation_only_mode);

  ~RoadmapExplorationBT();

  bool makeBTNodes();

  uint16_t tickOnceWithSleep();

  void halt();

  bool incrementFrontierSearchDistance();

  bool resetFrontierSearchDistance();

private:
  std::shared_ptr<nav2_util::LifecycleNode> bt_node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<Nav2Interface<nav2_msgs::action::NavigateToPose>> nav2_interface_;

  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr blackboard;
  BT::Tree behaviour_tree;

  std::shared_ptr<FrontierSearchBase> frontierSearchPtr_;
  std::shared_ptr<CostAssigner> cost_assigner_ptr_;
  std::shared_ptr<FullPathOptimizer> full_path_optimizer_;
  std::shared_ptr<SensorSimulator> sensor_simulator_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> explore_costmap_thread_;
};
}

#endif
