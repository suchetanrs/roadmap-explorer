#include "roadmap_explorer/planners/EuclideanPlugin.hpp"

namespace roadmap_explorer
{
void EuclideanDistancePlugin::configure(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, std::string name, std::shared_ptr<nav2_util::LifecycleNode> node)
{
  LOG_INFO("EuclideanDistancePlugin::configure");
  exploration_costmap_ = explore_costmap_ros->getCostmap();

  nav2_util::declare_parameter_if_not_declared(
    node, name + "closeness_rejection_threshold", rclcpp::ParameterValue(
      0.5));
  
  closeness_rejection_threshold_ = node->get_parameter(
    name + "closeness_rejection_threshold").as_double();
}

void EuclideanDistancePlugin::reset()
{
  return;
}

void EuclideanDistancePlugin::setPlanForFrontier(
  const geometry_msgs::msg::Pose start_pose_w,
  FrontierPtr & goal_point_w)
{
  // if already not achievable, return
  auto start_point_w = start_pose_w.position;
  if (goal_point_w->isAchievable() == false) {
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }

  // compute euclidean distance to frontier, if too close, reject
  auto length_to_frontier =
    sqrt(
    pow(
      start_point_w.x - goal_point_w->getGoalPoint().x,
      2) + pow(start_point_w.y - goal_point_w->getGoalPoint().y, 2));
  if (length_to_frontier < closeness_rejection_threshold_) {
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }

  // if far enough, set achievability true and path length to euclidean distance
  goal_point_w->setAchievability(true);
  goal_point_w->setPathLength(length_to_frontier);
  goal_point_w->setPathLengthInM(length_to_frontier);
  goal_point_w->setPathHeading(std::numeric_limits<double>::max());
  return;
}

} // namespace roadmap_explorer


#include <pluginlib/class_list_macros.hpp>
// Register the plugin
PLUGINLIB_EXPORT_CLASS(roadmap_explorer::EuclideanDistancePlugin, roadmap_explorer::BasePlanner)
