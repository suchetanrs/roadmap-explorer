#ifndef ROADMAP_PLANNER_PLUGIN_
#define ROADMAP_PLANNER_PLUGIN_

#include <iostream>
#include <cmath>
#include <string>
#include <chrono>
#include <queue>
#include <algorithm>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace roadmap_explorer
{

class FrontierRoadmapPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;


  // Callback function for plan messages
  void planCallback(const nav_msgs::msg::Path::SharedPtr msg);

protected:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger Logger_{rclcpp::get_logger("FrontierRoadmapPlanner")};
  std::string global_frame_, name_;
  bool use_final_approach_orientation_;

  // parent node weak ptr
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // Subscriber for plan messages
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  std::mutex latest_plan_mutex_;
  nav_msgs::msg::Path latest_plan_;

  static nav_msgs::msg::Path linearInterpolation(
    const nav_msgs::msg::Path& raw_path,
    const double & dist_bw_points);
};
}   //  namespace nav2_roadmap_explorer

#endif  //  ROADMAP_PLANNER_PLUGIN_