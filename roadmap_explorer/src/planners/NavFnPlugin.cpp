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
#include "roadmap_explorer/planners/NavFnPlugin.hpp"

namespace roadmap_explorer
{
void PluginNavFn::configure(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, std::string name, std::shared_ptr<nav2_util::LifecycleNode> node)
{
  exploration_costmap_ = explore_costmap_ros->getCostmap();

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".closeness_rejection_threshold", rclcpp::ParameterValue(
      0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".planner_allow_unknown", rclcpp::ParameterValue(
      true));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_planning_distance_roadmap", rclcpp::ParameterValue(
      6.0));

  closeness_rejection_threshold_ = node->get_parameter(
    name + ".closeness_rejection_threshold").as_double();
  planner_allow_unknown_ = node->get_parameter(
    name + ".planner_allow_unknown").as_bool();
  max_planning_distance_ = node->get_parameter(
    name + ".max_planning_distance_roadmap").as_double();
}

void PluginNavFn::reset()
{
  return;
}

void PluginNavFn::setPlanForFrontier(
  const geometry_msgs::msg::Pose start_pose_w,
  FrontierPtr & goal_point_w)
{
  // if already not achievable, return
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
      start_pose_w.position.x - goal_point_w->getGoalPoint().x,
      2) + pow(start_pose_w.position.y - goal_point_w->getGoalPoint().y, 2));
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

  // check if the set euclidean distance is greater than max planning distance, if yes, set to a large value and return
  if (goal_point_w->getPathLengthInM() > max_planning_distance_)
  {
    goal_point_w->setAchievability(true);
    goal_point_w->setPathLength(goal_point_w->getPathLength() * 5.0);
    goal_point_w->setPathLengthInM(goal_point_w->getPathLengthInM() * 5.0);
    return;
  }

  // otherwise, compute the path using the NavFn
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  std::unique_ptr<nav2_navfn_planner::NavFn> planner_;
  planner_ = std::make_unique<nav2_navfn_planner::NavFn>(
    exploration_costmap_->getSizeInCellsX(), exploration_costmap_->getSizeInCellsY());
  planner_->setNavArr(
    exploration_costmap_->getSizeInCellsX(),
    exploration_costmap_->getSizeInCellsY());
  planner_->setCostmap(exploration_costmap_->getCharMap(), true, planner_allow_unknown_);

  // start point
  unsigned int mx, my;
  if (!exploration_costmap_->worldToMap(start_pose_w.position.x, start_pose_w.position.y, mx, my)) {
    LOG_ERROR(
      "Cannot create a plan: the robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }
  int map_start[2];
  map_start[0] = mx;
  map_start[1] = my;

  // goal point
  if (!exploration_costmap_->worldToMap(
      goal_point_w->getGoalPoint().x,
      goal_point_w->getGoalPoint().y, mx, my))
  {
    LOG_ERROR(
      "The goal sent to the planner is off the global costmap Planning will always fail to this goal.");
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  // Take note this is computed backwards. Copied what was done in nav2 navfn planner.
  planner_->setStart(map_goal);
  planner_->setGoal(map_start);

#ifdef ROS_DISTRO_HUMBLE
  if (!planner_->calcNavFnAstar()) {
    LOG_WARN(
      "Plan not Found for frontier at x: " << goal_point_w->getGoalPoint().x << " y: " <<
        goal_point_w->getGoalPoint().y);
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }
#elif ROS_DISTRO_JAZZY || ROS_DISTRO_KILTED
  // Run the A* search to compute the navigation function.
  std::function<bool()> cancelChecker = []()
    {
      return !rclcpp::ok();
    };
  if (!planner_->calcNavFnAstar(cancelChecker)) {
    LOG_WARN(
      "Plan not Found for frontier at x: " << goal_point_w->getGoalPoint().x << " y: " <<
        goal_point_w->getGoalPoint().y);
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }
#else
#error Unsupported ROS DISTRO
#endif

  const int & max_cycles =
    (exploration_costmap_->getSizeInCellsX() >=
    exploration_costmap_->getSizeInCellsY()) ?
    (exploration_costmap_->getSizeInCellsX() *
    4) :
    (exploration_costmap_->getSizeInCellsY() * 4);
  int path_len = planner_->calcPath(max_cycles);
  if (path_len == 0) {
    LOG_INFO("Plan not found for " << goal_point_w << " path length is zero.");
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }

  // extract the plan
  float * x = planner_->getPathX();
  float * y = planner_->getPathY();
  int len = planner_->getPathLen();
  double path_length_m = 0;
  std::shared_ptr<geometry_msgs::msg::PoseStamped> previous_pose = nullptr;
  for (int i = len - 1; i >= 0; --i) {
    // convert the plan to world coordinates
    double world_x, world_y;
    exploration_costmap_->mapToWorld(x[i], y[i], world_x, world_y);
    geometry_msgs::msg::PoseStamped pose_from;
    pose_from.pose.position.x = world_x;
    pose_from.pose.position.y = world_y;
    pose_from.pose.position.z = 0.0;
    plan.poses.push_back(pose_from);
    if (i != 0 && previous_pose != nullptr) {
      path_length_m += distanceBetweenPoints(pose_from.pose.position, previous_pose->pose.position);
    }
    previous_pose = std::make_shared<geometry_msgs::msg::PoseStamped>(pose_from);
  }

  // if path does not exist, set to unachievable
  if (plan.poses.size() == 0) {
    LOG_INFO("Plan not found for " << goal_point_w << " path length is zero.");
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }

  // if path length is less than closeness threshold, set to unachievable
  if (path_length_m < closeness_rejection_threshold_) {
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }

  // else set to achievable and set path length
  rosVisualizerInstance.visualizePath("grid_based_frontier_plan", plan);
  goal_point_w->setAchievability(true);
  goal_point_w->setPathLength(plan.poses.size());
  goal_point_w->setPathLengthInM(path_length_m);
  goal_point_w->setPathHeading(std::numeric_limits<double>::max());
  return;
}
} // namespace roadmap_explorer

#include <pluginlib/class_list_macros.hpp>
// Register the plugin
PLUGINLIB_EXPORT_CLASS(roadmap_explorer::PluginNavFn, roadmap_explorer::BasePlanner)