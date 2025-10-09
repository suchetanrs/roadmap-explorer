#include "roadmap_explorer/planners/RandomDistancePlugin.hpp"

namespace roadmap_explorer
{
void RandomDistancePlugin::configure(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros)
{
  LOG_INFO("RandomDistancePlugin::configure");
  exploration_costmap_ = explore_costmap_ros->getCostmap();
  updateParameters();
}

void RandomDistancePlugin::reset()
{
  return;
}

double RandomDistancePlugin::getRandomVal()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 1.0);
  return dis(gen);
}

void RandomDistancePlugin::setPlanForFrontier(
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

  // if far enough, set achievability true and path length to a random value
  goal_point_w->setArrivalInformation(getRandomVal());
  goal_point_w->setGoalOrientation(getRandomVal());
  goal_point_w->setPathLength(getRandomVal());
  goal_point_w->setPathLengthInM(getRandomVal());
  goal_point_w->setPathHeading(getRandomVal());
  return;
}

} // namespace roadmap_explorer


#include <pluginlib/class_list_macros.hpp>
// Register the plugin
PLUGINLIB_EXPORT_CLASS(roadmap_explorer::RandomDistancePlugin, roadmap_explorer::BasePlanner)
