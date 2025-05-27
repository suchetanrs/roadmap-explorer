#include "panel.hpp"

#include <pluginlib/class_list_macros.hpp>

using namespace std::chrono_literals;
namespace rviz_plugin
{

// ─────────────────────────────────────────────────────────────────────────────
RoadmapExplorerPanel::RoadmapExplorerPanel(QWidget * parent)
: rviz_common::Panel(parent),
  start_button_(new QPushButton("Start exploration", this)),
  stop_button_(new QPushButton("Stop exploration",  this)),
  node_(std::make_shared<rclcpp::Node>("roadmap_explorer_panel")),
  client_(rclcpp_action::create_client<Explore>(node_, "roadmap_explorer"))
{
  // Basic UI
  auto * layout = new QVBoxLayout;
  layout->addWidget(start_button_);
  layout->addWidget(stop_button_);
  setLayout(layout);

  // Initial state
  updateButtons(false);

  // Qt connections
  connect(start_button_, &QPushButton::clicked, this, &RoadmapExplorerPanel::onStartClicked);
  connect(stop_button_,  &QPushButton::clicked, this, &RoadmapExplorerPanel::onStopClicked);

  // Periodic spinning so ROS callbacks run inside RViz’s GUI thread
  spin_timer_.setInterval(50);  // 20 Hz
  connect(&spin_timer_, &QTimer::timeout, [this]() { rclcpp::spin_some(node_); });
  spin_timer_.start();
}

// ───── Qt slots ──────────────────────────────────────────────────────────────
void RoadmapExplorerPanel::onStartClicked()
{
  sendGoal();
}

void RoadmapExplorerPanel::onStopClicked()
{
  cancelGoal();
}

// ───── Internal helpers ──────────────────────────────────────────────────────
void RoadmapExplorerPanel::sendGoal()
{
  if (!client_->wait_for_action_server(500ms))
  {
    RCLCPP_WARN(node_->get_logger(), "roadmap_explorer action server not available");
    return;
  }

  // Goal is empty for this action
  Explore::Goal goal_msg;

  auto goal_options = rclcpp_action::Client<Explore>::SendGoalOptions();
  goal_options.goal_response_callback =
    std::bind(&RoadmapExplorerPanel::goalResponseCallback, this, std::placeholders::_1);
  goal_options.feedback_callback =
    std::bind(&RoadmapExplorerPanel::feedbackCallback, this,
              std::placeholders::_1, std::placeholders::_2);
  goal_options.result_callback =
    std::bind(&RoadmapExplorerPanel::resultCallback, this, std::placeholders::_1);

  client_->async_send_goal(goal_msg, goal_options);
  updateButtons(true);
}

void RoadmapExplorerPanel::cancelGoal()
{
  if (goal_handle_)
  {
    client_->async_cancel_goal(goal_handle_);
  }
  updateButtons(false);
}

void RoadmapExplorerPanel::updateButtons(bool goal_active)
{
  start_button_->setEnabled(!goal_active);
  stop_button_->setEnabled(goal_active);
}

// ───── Action callbacks ──────────────────────────────────────────────────────
void RoadmapExplorerPanel::goalResponseCallback(
  const GoalHandle::SharedPtr & goal_handle)
{
  goal_handle_ = goal_handle;
  if (!goal_handle_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by action server");
    updateButtons(false);
  }
}

void RoadmapExplorerPanel::feedbackCallback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const Explore::Feedback> feedback)
{
  // Example: print the frontier count and elapsed time
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                       "Exploring for %.1fs, frontier count: %zu",
                       rclcpp::Duration(feedback->exploration_time).seconds(),
                       feedback->current_frontier.size());
}

void RoadmapExplorerPanel::resultCallback(const GoalHandle::WrappedResult & result)
{
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "Exploration finished successfully: %s",
                  result.result->success ? "true" : "false");
      break;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(node_->get_logger(),
                  "Exploration aborted (error code %u)", result.result->error_code);
      break;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(node_->get_logger(), "Exploration goal canceled");
      break;

    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
      break;
  }

  goal_handle_.reset();
  updateButtons(false);
}

}  // namespace rviz_plugin

// ─── Pluginlib export ─────────────────────────────────────────────────────────
PLUGINLIB_EXPORT_CLASS(rviz_plugin::RoadmapExplorerPanel, rviz_common::Panel)