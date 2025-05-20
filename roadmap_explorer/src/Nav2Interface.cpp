#include "roadmap_explorer/Nav2Interface.hpp"

namespace roadmap_explorer
{
Nav2Interface::Nav2Interface(rclcpp::Node::SharedPtr node)
: node_(node)
{
  nav2_goal_state_ = 1;
  nav2_client_callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  nav2Client_ = rclcpp_action::create_client<NavigateToPose>(
    node_, "navigate_to_pose",
    nav2_client_callback_group_);                                                                                           // was previously move_base, true

  nav2_goal_options_.feedback_callback = std::bind(
    &Nav2Interface::nav2GoalFeedbackCallback, this,
    std::placeholders::_1, std::placeholders::_2);
  nav2_goal_options_.result_callback = std::bind(
    &Nav2Interface::nav2GoalResultCallback, this,
    std::placeholders::_1);
  nav2_goal_options_.goal_response_callback = std::bind(
    &Nav2Interface::nav2GoalResponseCallback,
    this, std::placeholders::_1);

  if (!nav2Client_->wait_for_action_server(std::chrono::seconds(50))) {
    LOG_ERROR("Nav2 Action server not available after waiting for " << 50 << " seconds");
    rclcpp::shutdown();
  } else {
    LOG_INFO("Nav2 action server available");
  }


  updated_goal_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "goal_update", 10);
  // --------- To test from rviz --------------

  goal_pose_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose_exploration", 10,
    std::bind(&Nav2Interface::goalPoseCallback, this, std::placeholders::_1));

  cancel_goal_subscriber_ =
    node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 10,
    std::bind(&Nav2Interface::cancelAllGoalsCallback, this, std::placeholders::_1));
}

Nav2Interface::~Nav2Interface()
{
  nav2_client_callback_group_.reset();
  nav2Client_.reset();
  rclcpp::shutdown();
}

// ---------------------Test Node from Rviz------------------

void Nav2Interface::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  LOG_INFO("Received goal pose from RViz2");
  sendGoal(*msg);
}

void Nav2Interface::cancelAllGoalsCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  cancelAllGoals();
}

// --------------------Internal function-----------------------

void Nav2Interface::cancelAllGoals()
{
  std::unique_lock<std::mutex> lock(nav2Clientlock_);
  nav2Client_->async_cancel_all_goals();
}

void Nav2Interface::sendGoal(geometry_msgs::msg::PoseStamped & goal_pose)
{
  std::lock_guard<std::mutex> lock(goal_active_mutex_);
  goal_pose.header.stamp = node_->get_clock()->now();
  if (nav2_goal_state_ == 0) {
    LOG_INFO("SendGoal but updating goal since a goal is active.");
    updated_goal_publisher_->publish(goal_pose);
  } else {
    nav2_goal_state_ = 0;
    LOG_INFO("SendGoal but new goal");
    std::unique_lock<std::mutex> lock(nav2Clientlock_);
    auto nav2_goal_ = std::make_shared<NavigateToPose::Goal>();
    nav2_goal_->pose = goal_pose;
    // nav2_goal_->behavior_tree = node_->get_namespace();
    nav2Client_->async_send_goal(*nav2_goal_, nav2_goal_options_);
  }
}

void Nav2Interface::nav2GoalFeedbackCallback(
  GoalHandleNav2::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  auto nav2_feedback_ = std::make_shared<NavigateToPose::Feedback>();
  nav2_feedback_->current_pose = feedback->current_pose;
  nav2_feedback_->distance_remaining = feedback->distance_remaining;
}

void Nav2Interface::nav2GoalResultCallback(const GoalHandleNav2::WrappedResult & result)
{
  std::lock_guard<std::mutex> lock(goal_active_mutex_);
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      LOG_WARN("Goal succeeded");
      nav2_goal_state_ = 1;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      LOG_ERROR("Nav2 internal fault! Nav2 aborted the goal!");
      nav2_goal_state_ = -1;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      LOG_INFO("Nav2 goal canceled successfully");
      nav2_goal_state_ = 1;
      break;
    default:
      LOG_ERROR("Unknown nav2 goal result code");
  }
  LOG_INFO("Nav2 result callback executed");
}

void Nav2Interface::nav2GoalResponseCallback(const GoalHandleNav2::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    LOG_ERROR("Goal was rejected by Nav2 server");
    std::lock_guard<std::mutex> lock(goal_active_mutex_);
    nav2_goal_state_ = -1;
  } else {
    LOG_INFO("Goal accepted by Nav2 server, waiting for result");
    std::lock_guard<std::mutex> lock(goal_active_mutex_);
    nav2_goal_state_ = 0;
  }
}
}
