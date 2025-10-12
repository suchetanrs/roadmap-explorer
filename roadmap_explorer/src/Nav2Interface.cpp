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


#include "roadmap_explorer/Nav2Interface.hpp"

namespace roadmap_explorer
{
template<typename ActionT>
Nav2Interface<ActionT>::Nav2Interface(
  std::shared_ptr<nav2_util::LifecycleNode> node, std::string action_name,
  std::string update_topic_name)
{
  node_ = node;
  nav2_goal_state_ = NavGoalStatus::DEFAULT;
  nav2_client_callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  nav2Client_ =
    rclcpp_action::create_client<ActionT>(node, action_name, nav2_client_callback_group_);

  nav2_goal_options_.feedback_callback = std::bind(
    &Nav2Interface::nav2GoalFeedbackCallback, this,
    std::placeholders::_1, std::placeholders::_2);
  nav2_goal_options_.result_callback = std::bind(
    &Nav2Interface::nav2GoalResultCallback, this,
    std::placeholders::_1);
  nav2_goal_options_.goal_response_callback = std::bind(
    &Nav2Interface::nav2GoalResponseCallback,
    this, std::placeholders::_1);

  while (!nav2Client_->wait_for_action_server(std::chrono::seconds(1)) && rclcpp::ok()) {
    LOG_WARN("Waiting for Nav2 action server to be available");
  }
  updated_goal_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    update_topic_name, 10);
  updated_goal_publisher_->on_activate();
}

template<typename ActionT>
Nav2Interface<ActionT>::~Nav2Interface()
{
  LOG_INFO("Nav2Interface::~Nav2Interface()");
  updated_goal_publisher_->on_deactivate();
}
// --------------------Internal function-----------------------

template<typename ActionT>
bool Nav2Interface<ActionT>::sendCancelWaitForResponse(
  action_msgs::srv::CancelGoal::Response & response)
{
  if (current_goal_handle_ == nullptr) {
    LOG_ERROR("No valid goal to cancel");
    response.return_code = action_msgs::srv::CancelGoal::Response::ERROR_GOAL_TERMINATED;
    return true;
  }
  if (isGoalTerminated()) {
    LOG_ERROR("The goal is terminated. What are you cancelling/manually aborting?");
    return false;
  }
  if (nav2_goal_state_ == NavGoalStatus::CANCELLING) {
    LOG_ERROR("Canceling the previous goal. Wait for cancel result before sending another one.");
    return false;
  }
  // Get the cancellation future from nav2.
  // Lock ordering: always acquire goal_state_mutex_ before nav2Clientlock_ to prevent deadlock
  LOG_INFO("Acquiring lock for cancel operation");
  std::lock_guard<std::recursive_mutex> lock(goal_state_mutex_);
  std::lock_guard<std::mutex> lock2(nav2Clientlock_);
  LOG_INFO("Canceling goal");
  
  auto cancel_future = nav2Client_->async_cancel_all_goals();
  LOG_WARN("Sent cancel request to nav2, waiting for response");
  
  // Wait for cancel response with timeout (ROS2 recommended approach)
  const int timeout_seconds = 5;
  int elapsed_seconds = 0;
  while (cancel_future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
    elapsed_seconds++;
    LOG_WARN("Waiting for cancel response (" << elapsed_seconds << "/" << timeout_seconds << " seconds)");
    
    if (elapsed_seconds >= timeout_seconds) {
      LOG_ERROR(
        "Cancel request timed out after " << timeout_seconds << " seconds. The action server may be unresponsive. Moving on to result.");
      response.return_code = action_msgs::srv::CancelGoal::Response::ERROR_NONE;
      return false;
    }
  }

  // Wait until the cancel request has been accepted or an error occurs.
  auto result = cancel_future.get();
  LOG_WARN("Got cancel response");
  if (!result) {
    LOG_ERROR("The future response is null");
    return false;
  }
  response.return_code = result->return_code;
  return true;
}

template<typename ActionT>
void Nav2Interface<ActionT>::cancelAllGoals()
{
  std::lock_guard<std::recursive_mutex> lock(goal_state_mutex_);
  LOG_INFO("Cancelling goal.");
  action_msgs::srv::CancelGoal::Response response;
  if (sendCancelWaitForResponse(response)) {
    if (response.return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
      LOG_ERROR("Cancellation request accepted by nav2 server. Will cancel the goal.");
      nav2_goal_state_ = NavGoalStatus::CANCELLING;
    } else if (response.return_code == action_msgs::srv::CancelGoal::Response::ERROR_REJECTED) {
      LOG_INFO("Cancellation request rejected by Nav2 action server, not cancelling goal");
    } else {
      LOG_ERROR("No ongoing goal in nav2.");
      nav2_goal_state_ = NavGoalStatus::CANCELLED;
    }
  } else {
    LOG_ERROR("sendCancelWaitForResponse did not succeed in cancelAllGoals");
  }
}

template<typename ActionT>
void Nav2Interface<ActionT>::nav2GoalResultCallback(
  const typename GoalHandle::WrappedResult & result)
{
  std::lock_guard<std::recursive_mutex> lock(goal_state_mutex_);
  if (!current_goal_handle_) {
    LOG_ERROR("Received result for a goal that does not exist. Ignoring.");
    return;
  }
  if (current_goal_handle_ && current_goal_handle_->get_goal_id() != result.goal_id) {
    LOG_WARN("Received result for a stale goal. Ignoring.");
    return;
  }
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      LOG_WARN("Goal succeeded");
      nav2_goal_state_ = NavGoalStatus::SUCCEEDED;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      LOG_ERROR("Nav2 internal fault! Nav2 aborted the goal!");
      nav2_goal_state_ = NavGoalStatus::FAILED;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      LOG_INFO("Nav2 goal canceled successfully");
      nav2_goal_state_ = NavGoalStatus::CANCELLED;
      break;
    default:
      LOG_ERROR("Unknown nav2 goal result code");
  }
  current_goal_handle_.reset();
  LOG_INFO("Nav2 result callback executed");
}

template<typename ActionT>
void Nav2Interface<ActionT>::nav2GoalResponseCallback(
  const typename GoalHandle::SharedPtr & goal_handle)
{
  std::lock_guard<std::recursive_mutex> lock(goal_state_mutex_);
  if (!goal_handle) {
    LOG_ERROR("Goal was rejected by Nav2 server");
    nav2_goal_state_ = NavGoalStatus::REJECTED;
  } else {
    LOG_INFO("Goal accepted by Nav2 server, waiting for result");
    current_goal_handle_ = goal_handle;
    nav2_goal_state_ = NavGoalStatus::ONGOING;
  }
}

template<typename ActionT>
bool Nav2Interface<ActionT>::canSendNewGoal()
{
  if (current_goal_handle_ != nullptr) {
    LOG_ERROR(
      "Goal handle not null. There is an ongoing goal but you are sending a new one. Terminate this first.");
    return false;
  }
  if (nav2_goal_state_ == NavGoalStatus::ONGOING) {
    LOG_ERROR("There is an ongoing goal but you are sending a new one. Terminate this first.");
    return false;
  }
  if (nav2_goal_state_ == NavGoalStatus::CANCELLING) {
    LOG_ERROR("Cancelling is in progress. Wait for terminal state before sending a new one.");
    return false;
  }
  if (nav2_goal_state_ == NavGoalStatus::SENDING_GOAL) {
    LOG_ERROR("Send goal is in progress. Wait for terminal state before sending a new one.");
    return false;
  }
  return true;
}

template<typename ActionT>
bool Nav2Interface<ActionT>::sendGoal(geometry_msgs::msg::PoseStamped & pose)
{
  // Only allow a new goal if the current state is IDLE or in a terminal state.
  if (!canSendNewGoal()) {
    LOG_ERROR("Cannot send new goal");
    return false;
  }
  std::lock_guard<std::recursive_mutex> lock(this->goal_state_mutex_);
  auto goal_pose = pose;
  goal_pose.header.stamp = this->node_->get_clock()->now();

  LOG_INFO("New goal being sent to nav2");
  this->nav2_goal_state_ = NavGoalStatus::SENDING_GOAL;
  LOG_INFO("SendGoal but new goal");
  std::lock_guard<std::mutex> lock2(this->nav2Clientlock_);
  auto nav2_goal_ = std::make_shared<typename ActionT::Goal>();
  nav2_goal_->pose = goal_pose;
  nav2_goal_->behavior_tree = parameterInstance.getValue<std::string>("explorationBT.nav2_bt_xml");
  this->nav2Client_->async_send_goal(*nav2_goal_, this->nav2_goal_options_);
  return true;
}

template<typename ActionT>
void Nav2Interface<ActionT>::sendUpdatedGoal(geometry_msgs::msg::PoseStamped pose)
{
  std::lock_guard<std::recursive_mutex> lock(goal_state_mutex_);
  pose.header.stamp = node_->get_clock()->now();
  if (isGoalActive()) {
    LOG_INFO("Updated goal being sent to nav2");
    updated_goal_publisher_->publish(pose);
  } else {
    LOG_ERROR("There is no ongoing goal. What are you re-planning?");
  }
}

template<typename ActionT>
void Nav2Interface<ActionT>::nav2GoalFeedbackCallback(
  typename GoalHandle::SharedPtr, const std::shared_ptr<const typename ActionT::Feedback> feedback)
{
  std::lock_guard<std::mutex> lock(this->nav2_feedback_mutex_);
  (void)feedback;
}

// Explicit template instantiation for NavigateToPose action.
// This is required because template definitions must be visible at instantiation time.
template class Nav2Interface<nav2_msgs::action::NavigateToPose>;
}  // namespace roadmap_explorer
