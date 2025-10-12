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


#ifndef NAV2_INTERFACE_HPP_
#define NAV2_INTERFACE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.h>

#include <nav2_msgs/action/navigate_to_pose.hpp>

#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/util/MagicEnum.hpp"
#include "roadmap_explorer/Parameters.hpp"

namespace roadmap_explorer
{

enum class NavGoalStatus
{
  // terminal states
  DEFAULT,
  SUCCEEDED,
  FAILED,
  REJECTED,
  CANCELLED,
  // ongoing states
  SENDING_GOAL,
  ONGOING,
  CANCELLING,
};

inline std::ostream & operator<<(std::ostream & os, const NavGoalStatus status)
{
  os << magic_enum::enum_name(status);
  return os;
}

template<typename ActionT>
class Nav2Interface
{
public:
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  explicit Nav2Interface(
    std::shared_ptr<nav2_util::LifecycleNode> node, std::string action_name,
    std::string update_topic_name);
  ~Nav2Interface();
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  bool sendCancelWaitForResponse(action_msgs::srv::CancelGoal::Response & response);
  void cancelAllGoals();
  virtual bool sendGoal(geometry_msgs::msg::PoseStamped & pose);
  void sendUpdatedGoal(geometry_msgs::msg::PoseStamped pose);
  NavGoalStatus getGoalStatus()
  {
    std::lock_guard<std::recursive_mutex> lock(goal_state_mutex_);
    return nav2_goal_state_;
  }

  bool isGoalTerminated()
  {
    return !isGoalActive();
  }

  bool isGoalActive()
  {
    std::lock_guard<std::recursive_mutex> lock(goal_state_mutex_);
    return nav2_goal_state_ == NavGoalStatus::ONGOING ||
           nav2_goal_state_ == NavGoalStatus::CANCELLING ||
           nav2_goal_state_ == NavGoalStatus::SENDING_GOAL;
  }

  bool isGoalOngoing()
  {
    std::lock_guard<std::recursive_mutex> lock(goal_state_mutex_);
    return nav2_goal_state_ == NavGoalStatus::ONGOING;
  }

  bool canSendNewGoal();

  bool isGoalHandleNull()
  {
    return current_goal_handle_ == nullptr;
  }

protected:
  virtual void nav2GoalFeedbackCallback(
    typename GoalHandle::SharedPtr,
    const std::shared_ptr<const typename ActionT::Feedback> feedback);
  void nav2GoalResultCallback(const typename GoalHandle::WrappedResult & result);
  void nav2GoalResponseCallback(const typename GoalHandle::SharedPtr & goal_handle);

  rclcpp_action::Client<ActionT>::SharedPtr nav2Client_;
  std::mutex nav2Clientlock_;
  rclcpp_action::Client<ActionT>::SendGoalOptions nav2_goal_options_;
  rclcpp::CallbackGroup::SharedPtr nav2_client_callback_group_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    updated_goal_publisher_;
  std::recursive_mutex goal_state_mutex_;
  std::mutex nav2_feedback_mutex_;
  NavGoalStatus nav2_goal_state_;

  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr current_goal_handle_;
  bool shutting_down_ = false;

  std::shared_ptr<nav2_util::LifecycleNode> node_;
};
}  // namespace roadmap_explorer

#endif  // NAV2_INTERFACE_HPP
