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

#include "roadmap_explorer/planners/RandomDistancePlugin.hpp"

namespace roadmap_explorer
{
void RandomDistancePlugin::configure(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, std::string name, std::shared_ptr<nav2_util::LifecycleNode> node)
{
  LOG_INFO("RandomDistancePlugin::configure");
  exploration_costmap_ = explore_costmap_ros->getCostmap();

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".closeness_rejection_threshold", rclcpp::ParameterValue(
      0.5));

  closeness_rejection_threshold_ = node->get_parameter(
    name + ".closeness_rejection_threshold").as_double();
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