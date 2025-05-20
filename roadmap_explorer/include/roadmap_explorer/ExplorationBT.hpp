#ifndef EXPLORE_SERVER_
#define EXPLORE_SERVER_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/parameter.hpp>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_util/node_thread.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>

#include "behaviortree_cpp/bt_factory.h"

#include "roadmap_explorer/util/RosVisualizer.hpp"
#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/util/EventLogger.hpp"
#include "roadmap_explorer/CostAssigner.hpp"
#include "roadmap_explorer/FrontierSearchAllCells.hpp"
#include "roadmap_explorer/FrontierSearch.hpp"
#include "roadmap_explorer/Nav2Interface.hpp"
#include "roadmap_explorer/Helpers.hpp"
#include "roadmap_explorer/FullPathOptimizer.hpp"
#include "roadmap_explorer/controllers/recovery_controller.hpp"
#include "roadmap_explorer/controllers/initialization_controller.hpp"

namespace roadmap_explorer
{

struct RobotActiveGoals
{
  std::mutex mutex;
  std::map<std::string, std::shared_ptr<geometry_msgs::msg::PoseStamped>> goals;
};

enum class CurrentGoalStatus
{
  RUNNING,
  COMPLETE
};

class FrontierExplorationServer
{
public:
  FrontierExplorationServer(rclcpp::Node::SharedPtr node);

  ~FrontierExplorationServer();

  void buildBoundaryAndCenter();

  void makeBTNodes();

  void run();

  void rvizControl(std_msgs::msg::Int32 rvizControlValue);

private:
  // ROS Internal
  rclcpp::Node::SharedPtr bt_node_;
  rclcpp::CallbackGroup::SharedPtr explore_server_callback_group_;
  rclcpp::CallbackGroup::SharedPtr multirobot_service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr rviz_control_callback_group_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> explore_costmap_thread_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr exploration_rviz_sub_;

  // Nav2 related
  int nav2WaitTime_;
  std::mutex process_active_goals_lock_;
  std::shared_ptr<Nav2Interface> nav2_interface_;

  // Exploration related
  geometry_msgs::msg::PolygonStamped explore_boundary_;
  geometry_msgs::msg::PointStamped explore_center_;
  std::vector<std::string> config_;

  // BT related
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr blackboard;
  BT::Tree behaviour_tree;

  std::shared_ptr<FrontierSearch> frontierSearchPtr_;
  std::shared_ptr<CostAssigner> bel_ptr_;
  std::shared_ptr<FullPathOptimizer> full_path_optimizer_;

  int retry_;
  std::vector<std::string> robot_namespaces_;
  bool use_custom_sim_;
  bool process_other_robots_;
  bool exploration_active_;
  std::vector<FrontierPtr> blacklisted_frontiers_;       // these are the frontiers traversed by this robot.
  RobotActiveGoals robot_active_goals_;
  std::shared_ptr<RecoveryController> recovery_controller_;
  std::shared_ptr<InitCommandVelNode> initialization_controller_;
  std::string bt_xml_path_;
};
}

#endif
