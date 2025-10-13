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

#ifndef ROADMAP_EXPLORER__EXPLORATION_SERVER_HPP_
#define ROADMAP_EXPLORER__EXPLORATION_SERVER_HPP_

#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_util/lifecycle_node.hpp>

#include "roadmap_explorer/ExplorationBT.hpp"
#include "roadmap_explorer_msgs/action/explore.hpp"
#include "roadmap_explorer_msgs/srv/save_map_and_roadmap.hpp"

namespace roadmap_explorer
{

enum class ActionTerminalState { SUCCEED, ABORT, CANCEL };

class ExplorationServer : public nav2_util::LifecycleNode
{
public:
  using ExploreAction = roadmap_explorer_msgs::action::Explore;
  using GoalHandleExplore = rclcpp_action::ServerGoalHandle<ExploreAction>;

  explicit ExplorationServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ExplorationServer();

protected:
  // Lifecycle transitions
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  // Action-server callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExploreAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExplore> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleExplore> goal_handle);

  // Spawned per-goal execution
  void execute(const std::shared_ptr<GoalHandleExplore> goal_handle);

  void terminateGoal(
    const ActionTerminalState terminal_state,
    const std::shared_ptr<GoalHandleExplore> goal_handle,
    std::shared_ptr<ExploreAction::Result> result);

  void publish_feedback(const std::shared_ptr<GoalHandleExplore> & goal_handle);
  
  bool make_exploration_bt(bool exploration_mode);

  // Service callback
  void handle_save_map_and_roadmap(
    const std::shared_ptr<roadmap_explorer_msgs::srv::SaveMapAndRoadmap::Request> request,
    std::shared_ptr<roadmap_explorer_msgs::srv::SaveMapAndRoadmap::Response> response);

  // Members
  rclcpp_action::Server<ExploreAction>::SharedPtr action_server_;
  rclcpp::Service<roadmap_explorer_msgs::srv::SaveMapAndRoadmap>::SharedPtr save_map_service_;
  std::shared_ptr<RoadmapExplorationBT> exploration_bt_;
  std::mutex exploration_bt_mutex_;
  bool server_active_{false};
  bool localisation_only_mode_{false};

  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  rclcpp::CallbackGroup::SharedPtr action_server_cb_group_;
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  std::shared_ptr<nav2_util::LifecycleNode> node_;
};

}  // namespace roadmap_explorer

#endif  // ROADMAP_EXPLORER__EXPLORATION_SERVER_HPP_
