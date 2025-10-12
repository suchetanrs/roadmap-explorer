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

#include "roadmap_explorer/planners/FrontierRoadmapPlugin.hpp"

namespace roadmap_explorer
{
void PluginFrontierRoadmap::configure(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, std::string name, std::shared_ptr<nav2_util::LifecycleNode> node)
{
  exploration_costmap_ = explore_costmap_ros->getCostmap();

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".closeness_rejection_threshold", rclcpp::ParameterValue(
      0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_planning_distance_roadmap", rclcpp::ParameterValue(
      6.0));

  closeness_rejection_threshold_ = node->get_parameter(
    name + ".closeness_rejection_threshold").as_double();
  max_planning_distance_ = node->get_parameter(
    name + ".max_planning_distance_roadmap").as_double();

  LOG_DEBUG("closess_rejection_threshold: " << closeness_rejection_threshold_);
  LOG_DEBUG("max_planning_distance_roadmap: " << max_planning_distance_);
}

void PluginFrontierRoadmap::reset()
{
  return;
}

void PluginFrontierRoadmap::setPlanForFrontier(
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
  if (goal_point_w->getPathLengthInM() >
    max_planning_distance_)
  {
    goal_point_w->setAchievability(true);
    goal_point_w->setPathLength(goal_point_w->getPathLength() * 5.0);
    goal_point_w->setPathLengthInM(goal_point_w->getPathLengthInM() * 5.0);
    return;
  }

  // otherwise, compute the path using the roadmap
  auto path_length = frontierRoadmapInstance.getPlan(
    start_pose_w.position.x,
    start_pose_w.position.y, true,
    goal_point_w->getGoalPoint().x, goal_point_w->getGoalPoint().y, true, false);

  // if path does not exist, set to unachievable
  if (path_length.path_exists == false) {
    LOG_INFO("Plan not found for " << goal_point_w << " path does not exist.");
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }

  // if path length is less than closeness threshold, set to unachievable
  if (path_length.path_length_m < closeness_rejection_threshold_) {
    goal_point_w->setAchievability(false);
    goal_point_w->setPathLength(std::numeric_limits<double>::max());
    goal_point_w->setPathLengthInM(std::numeric_limits<double>::max());
    goal_point_w->setPathHeading(std::numeric_limits<double>::max());
    return;
  }

  // else set to achievable and set path length
  goal_point_w->setAchievability(true);
  goal_point_w->setPathLength(path_length.path_length_m);
  goal_point_w->setPathLengthInM(path_length.path_length_m);
  goal_point_w->setPathHeading(std::numeric_limits<double>::max());
  return;
}
} // namespace roadmap_explorer

#include <pluginlib/class_list_macros.hpp>
// Register the plugin
PLUGINLIB_EXPORT_CLASS(roadmap_explorer::PluginFrontierRoadmap, roadmap_explorer::BasePlanner)