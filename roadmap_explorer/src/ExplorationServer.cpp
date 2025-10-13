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

#include "roadmap_explorer/ExplorationServer.hpp"

using namespace std::chrono_literals;

namespace roadmap_explorer
{

ExplorationServer::ExplorationServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("roadmap_explorer_node", "", options)
{
  LOG_INFO("Created " << get_name());
  server_active_ = false;
}

ExplorationServer::~ExplorationServer()
{
  LOG_WARN("Destroyed!! " << get_name());
}

nav2_util::CallbackReturn ExplorationServer::on_configure(const rclcpp_lifecycle::State &)
{
  LOG_INFO("Configured " << get_name());
  node_ = shared_from_this();

  nav2_util::declare_parameter_if_not_declared(node_, "localisation_only_mode", rclcpp::ParameterValue(false));
  node_->get_parameter("localisation_only_mode", localisation_only_mode_);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ExplorationServer::on_activate(const rclcpp_lifecycle::State &)
{
  LOG_INFO("Activating " << get_name());
  server_active_ = true;
  action_server_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  action_server_ = rclcpp_action::create_server<ExploreAction>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "roadmap_explorer",
    std::bind(&ExplorationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ExplorationServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&ExplorationServer::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_cb_group_);
  
  save_map_service_ = create_service<roadmap_explorer_msgs::srv::SaveMapAndRoadmap>(
    "save_map_and_roadmap",
    std::bind(&ExplorationServer::handle_save_map_and_roadmap, this, 
              std::placeholders::_1, std::placeholders::_2),
#ifdef ROS_DISTRO_HUMBLE
    rmw_qos_profile_services_default,
#elif ROS_DISTRO_JAZZY || ROS_DISTRO_KILTED
    rclcpp::ServicesQoS(),
#else
    #error "Unsupported ROS distro"
#endif
    service_cb_group_);
  
  LOG_INFO("Created save_map_and_roadmap service");
  
  // Create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ExplorationServer::on_deactivate(const rclcpp_lifecycle::State &)
{
  LOG_WARN("Deactivating " << get_name());
  server_active_ = false;
  action_server_.reset();
  save_map_service_.reset();
  LOG_WARN("Stopping BT!! " << get_name());
  {
    std::lock_guard<std::mutex> lock(exploration_bt_mutex_);
    if (exploration_bt_) {
      exploration_bt_->halt();
    }
    LOG_WARN("Halted BT!! " << get_name());
    exploration_bt_.reset();
  }
  LOG_WARN("Destroying bond!! " << get_name());
  destroyBond();
  LOG_WARN("Deactivated!! " << get_name());
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ExplorationServer::on_cleanup(const rclcpp_lifecycle::State &)
{
  LOG_INFO("Cleaning up " << get_name());
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ExplorationServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  LOG_INFO("Shutting down " << get_name());
  return nav2_util::CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse ExplorationServer::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const ExploreAction::Goal>/*goal*/)
{
  if (!server_active_) {
    LOG_WARN("Received goal while inactive -> rejecting");
    return rclcpp_action::GoalResponse::REJECT;
  }
  LOG_INFO("Accepted new ExploreArea goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ExplorationServer::handle_cancel(
  const std::shared_ptr<GoalHandleExplore> goal_handle)
{
  (void)goal_handle;
  LOG_INFO("Cancel requested");
  std::lock_guard<std::mutex> lock(exploration_bt_mutex_);
  if (!exploration_bt_)
  {
    LOG_WARN("Rejecting cancel request because an instance of exploration is yet to be created.");
    return rclcpp_action::CancelResponse::REJECT;    
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ExplorationServer::handle_accepted(
  const std::shared_ptr<GoalHandleExplore> goal_handle)
{
  (void)goal_handle;
  LOG_INFO("Goal accepted, starting execution");
  // Spin the work off to a new thread so we don’t block the executor
  std::thread{std::bind(&ExplorationServer::execute, this, goal_handle)}.detach();
}

void ExplorationServer::publish_feedback(
  const std::shared_ptr<GoalHandleExplore> & goal_handle)
{
  auto feedback = std::make_shared<ExploreAction::Feedback>();
  goal_handle->publish_feedback(feedback);
}

bool ExplorationServer::make_exploration_bt(bool localisation_only_mode)
{
  exploration_bt_ = std::make_shared<RoadmapExplorationBT>(node_, localisation_only_mode);
  return exploration_bt_->makeBTNodes();
}

void ExplorationServer::execute(const std::shared_ptr<GoalHandleExplore> goal_handle)
{
  LOG_INFO("Running exploration BT for goal");
  auto result = std::make_shared<ExploreAction::Result>();
  result->success = false;
  try {

    switch (goal_handle->get_goal()->exploration_bringup_mode) {
      case ExploreAction::Goal::NEW_EXPLORATION_SESSION:
        {
          std::lock_guard<std::mutex> lock(exploration_bt_mutex_);
          exploration_bt_.reset();
          if(!make_exploration_bt(localisation_only_mode_))
          {
            result->success = false;
            terminateGoal(ActionTerminalState::ABORT, goal_handle, result);
            return;
          }
        }
        break;
      case ExploreAction::Goal::CONTINUE_FROM_TERMINATED_SESSION:
        LOG_INFO("Continue from terminated session. Nothing to reset.");
        {
          std::lock_guard<std::mutex> lock(exploration_bt_mutex_);
          if(exploration_bt_ == nullptr)
          {
            LOG_ERROR("Exploration BT not initialized yet to continue from a session. Please start a new session first");
            throw RoadmapExplorerException("Exploration BT not initialized yet to continue from a session. Please start a new session first");
          }
        }
        break;
      case ExploreAction::Goal::CONTINUE_FROM_SAVED_SESSION:
        // LOG_ERROR("Continue from saved session is not implemented yet.");
        // throw RoadmapExplorerException("Continue from saved session is not implemented yet.");
        if(localisation_only_mode_)
        {
          std::lock_guard<std::mutex> lock(exploration_bt_mutex_);
          exploration_bt_.reset();
          if(!make_exploration_bt(localisation_only_mode_))
          {
            result->success = false;
            terminateGoal(ActionTerminalState::ABORT, goal_handle, result);
            return;
          }
          exploration_bt_->loadExplorationMetaData(goal_handle->get_goal()->session_name.data, goal_handle->get_goal()->load_from_folder.data);
        }
        else
        {
          LOG_ERROR("You cannot continue from saved session when in mapping mode.");
          throw RoadmapExplorerException("You cannot continue from saved session when in mapping mode.");
        }
        break;
      default:
        throw RoadmapExplorerException("Unknown exploration_bringup_mode value");
    }

    while (rclcpp::ok()) {
      auto error_code = exploration_bt_->tickOnceWithSleep();

      // Check for cancellation
      if (goal_handle->is_canceling()) {
        LOG_WARN("Cancelling goal");
        terminateGoal(ActionTerminalState::CANCEL, goal_handle, result);
        return;
      }

      if (error_code == ExploreActionResult::NO_ERROR) {
        continue;
      } else if (error_code == ExploreActionResult::NO_MORE_REACHABLE_FRONTIERS) {
        LOG_INFO("No more reachable frontiers found, exploration complete");
        result->success = true;
        result->error_code = ExploreActionResult::NO_MORE_REACHABLE_FRONTIERS;
        terminateGoal(ActionTerminalState::SUCCEED, goal_handle, result);
        return;
      } else if (error_code == ExploreActionResult::NAV2_INTERNAL_FAULT) {
        LOG_ERROR("Internal fault during exploration, aborting");
        result->success = false;
        result->error_code = ExploreActionResult::NAV2_INTERNAL_FAULT;
        terminateGoal(ActionTerminalState::ABORT, goal_handle, result);
        return;
      } else {
        LOG_ERROR("Unknown error occurred during exploration");
        result->success = false;
        result->error_code = ExploreActionResult::UNKNOWN;
        terminateGoal(ActionTerminalState::ABORT, goal_handle, result);
        return;
      }
    }

  } catch (const RoadmapExplorerException & ex) {
    LOG_ERROR("Exploration failed with exception: " << ex.what());
    result->success = false;
    result->error_code = ExploreActionResult::UNKNOWN;
    terminateGoal(ActionTerminalState::ABORT, goal_handle, result);
  }
}

void ExplorationServer::terminateGoal(
    const ActionTerminalState terminal_state,
    const std::shared_ptr<GoalHandleExplore> goal_handle,
    std::shared_ptr<ExploreAction::Result> result)
{
  {
    std::lock_guard<std::mutex> lock(exploration_bt_mutex_);
    if(exploration_bt_)
    {
      exploration_bt_->halt();
    }
  }

  switch (terminal_state) {
    case ActionTerminalState::SUCCEED:
      goal_handle->succeed(result);
      break;
    case ActionTerminalState::ABORT:
      goal_handle->abort(result);
      break;
    case ActionTerminalState::CANCEL:
      goal_handle->canceled(result);
      break;
  }
}

void ExplorationServer::handle_save_map_and_roadmap(
  const std::shared_ptr<roadmap_explorer_msgs::srv::SaveMapAndRoadmap::Request> request,
  std::shared_ptr<roadmap_explorer_msgs::srv::SaveMapAndRoadmap::Response> response)
{
  LOG_INFO("Received save_map_and_roadmap service request");
  LOG_INFO("Base path: " << request->base_path);
  LOG_INFO("Session name: " << request->session_name);

  std::lock_guard<std::mutex> lock(exploration_bt_mutex_);
  
  if (!exploration_bt_) {
    LOG_ERROR("Cannot save exploration data: exploration_bt is not initialized");
    response->success = false;
    response->status_message = "Exploration BT not initialized";
    return;
  }

  try {
    bool result = exploration_bt_->saveExplorationMetaData(request->session_name, request->base_path);
    
    if (result) {
      LOG_INFO("Successfully saved exploration metadata");
      response->success = true;
      response->status_message = "Exploration metadata saved successfully";
    } else {
      LOG_ERROR("Failed to save exploration metadata");
      response->success = false;
      response->status_message = "Failed to save exploration metadata";
    }
  } catch (const std::exception& e) {
    LOG_ERROR("Exception while saving exploration metadata: " << e.what());
    response->success = false;
    response->status_message = std::string("Exception: ") + e.what();
  }
}

}  // namespace roadmap_explorer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(roadmap_explorer::ExplorationServer)
