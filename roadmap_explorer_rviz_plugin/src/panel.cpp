#include "panel.hpp"

#include <pluginlib/class_list_macros.hpp>

using namespace std::chrono_literals;
namespace roadmap_explorer
{

RoadmapExplorerPanel::RoadmapExplorerPanel(QWidget * parent)
: rviz_common::Panel(parent),
  rb_new_session_(new QRadioButton("New exploration session", this)),
  rb_continue_terminated_(new QRadioButton("Continue from terminated session", this)),
  rb_continue_saved_(new QRadioButton("Continue from saved session", this)),
  mode_group_(new QButtonGroup(this)),
  start_button_(new QPushButton("Start exploration", this)),
  stop_button_(new QPushButton("Stop exploration",  this)),
  node_(std::make_shared<rclcpp::Node>("roadmap_explorer_rviz_anel")),
  client_(rclcpp_action::create_client<Explore>(node_, "roadmap_explorer"))
{
  // Basic UI
  auto * layout = new QVBoxLayout;

  {
    // Put them all into a QButtonGroup so only one is checked at a time
    mode_group_->addButton(rb_new_session_,  static_cast<int>(Explore::Goal::NEW_EXPLORATION_SESSION));
    mode_group_->addButton(rb_continue_terminated_, static_cast<int>(Explore::Goal::CONTINUE_FROM_TERMINATED_SESSION));
    mode_group_->addButton(rb_continue_saved_, static_cast<int>(Explore::Goal::CONTINUE_FROM_SAVED_SESSION));

    // Check the “New session” one by default
    rb_new_session_->setChecked(true);

    // Add them to the layout (stacked vertically)
    layout->addWidget(rb_new_session_);
    layout->addWidget(rb_continue_terminated_);
    layout->addWidget(rb_continue_saved_);
  }

  layout->addWidget(start_button_);
  layout->addWidget(stop_button_);
  setLayout(layout);

  // Initial state
  updateButtons(ButtonSetting::GOAL_INACTIVE);

  // Qt connections
  connect(start_button_, &QPushButton::clicked, this, &RoadmapExplorerPanel::onStartClicked);
  connect(stop_button_,  &QPushButton::clicked, this, &RoadmapExplorerPanel::onStopClicked);

  spin_thread_ = std::make_unique<std::thread>([this]() {
    while(rclcpp::ok())
    {
      rclcpp::sleep_for(100ms);
      rclcpp::spin_some(node_);
      if(!client_->wait_for_action_server(1s))
      {
        updateButtons(ButtonSetting::GOAL_INACTIVE);
        goal_handle_.reset();
      }
    }
  });
  spin_thread_->detach();
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
uint16_t RoadmapExplorerPanel::currentMode() const
{
  // use with the same IDs as the constants in the action:
  //   0 = NEW_EXPLORATION_SESSION
  //   1 = CONTINUE_FROM_TERMINATED_SESSION
  //   2 = CONTINUE_FROM_SAVED_SESSION
  int id = mode_group_->checkedId();
  if (id < 0) {
    // Fallback: if somehow none is checked (should not happen), default to 0.
    RCLCPP_ERROR(node_->get_logger(), "No exploration bringup mode was specified, resorting to new exploration session");
    return Explore::Goal::NEW_EXPLORATION_SESSION;
  }
  return static_cast<uint16_t>(id);
}

// ───── Internal helpers ──────────────────────────────────────────────────────
void RoadmapExplorerPanel::sendGoal()
{
  // Goal is empty for this action
  Explore::Goal goal_msg;
  goal_msg.exploration_bringup_mode = this->currentMode();

  auto goal_options = rclcpp_action::Client<Explore>::SendGoalOptions();
  goal_options.goal_response_callback =
    std::bind(&RoadmapExplorerPanel::goalResponseCallback, this, std::placeholders::_1);
  goal_options.feedback_callback =
    std::bind(&RoadmapExplorerPanel::feedbackCallback, this,
              std::placeholders::_1, std::placeholders::_2);
  goal_options.result_callback =
    std::bind(&RoadmapExplorerPanel::resultCallback, this, std::placeholders::_1);

  client_->async_send_goal(goal_msg, goal_options);
  updateButtons(ButtonSetting::IN_PROCESS);
}

void RoadmapExplorerPanel::cancelGoal()
{
  if (goal_handle_)
  {
    updateButtons(ButtonSetting::IN_PROCESS);
    client_->async_cancel_goal(goal_handle_, std::bind(&RoadmapExplorerPanel::cancelCallback, this, std::placeholders::_1));
  }
  else
  {
    updateButtons(ButtonSetting::GOAL_INACTIVE);
  }
}

void RoadmapExplorerPanel::updateButtons(ButtonSetting setting)
{
  if (setting == ButtonSetting::IN_PROCESS)
  {
    start_button_->setEnabled(false);
    stop_button_->setEnabled(false);
    // Also disable radio buttons while exploration is active
    rb_new_session_->setEnabled(false);
    rb_continue_terminated_->setEnabled(false);
    rb_continue_saved_->setEnabled(false);
  }
  else
  {
    start_button_->setEnabled(setting == ButtonSetting::GOAL_INACTIVE);
    stop_button_->setEnabled(setting == ButtonSetting::GOAL_ACTIVE);
    // Also disable radio buttons while exploration is active
    rb_new_session_->setEnabled(setting == ButtonSetting::GOAL_INACTIVE);
    rb_continue_terminated_->setEnabled(setting == ButtonSetting::GOAL_INACTIVE);
    rb_continue_saved_->setEnabled(setting == ButtonSetting::GOAL_INACTIVE);
  }
}

// ───── Action callbacks ──────────────────────────────────────────────────────
void RoadmapExplorerPanel::goalResponseCallback(
  const GoalHandle::SharedPtr & goal_handle)
{
  RCLCPP_INFO(node_->get_logger(), "Thread goal response id: %d", std::this_thread::get_id());
  goal_handle_ = goal_handle;
  if (!goal_handle_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by action server");
    updateButtons(ButtonSetting::GOAL_INACTIVE);  
    return;
  }
  updateButtons(ButtonSetting::GOAL_ACTIVE);
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
  RCLCPP_INFO(node_->get_logger(), "Thread result cb id: %d", std::this_thread::get_id());
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
  updateButtons(ButtonSetting::GOAL_INACTIVE);
}

void RoadmapExplorerPanel::cancelCallback(std::shared_ptr<action_msgs::srv::CancelGoal_Response> response)
{
  using CancelResp = action_msgs::srv::CancelGoal_Response;
  RCLCPP_INFO(node_->get_logger(), "Thread cancel cb id: %d", std::this_thread::get_id());
  if (!goal_handle_) {
    RCLCPP_WARN(node_->get_logger(), "Cancel response received, but no goal was active.");
    return;
  }

  switch (response->return_code) {
    case CancelResp::ERROR_NONE:
      RCLCPP_INFO(node_->get_logger(),
                  "Cancel request was ACCEPTED by the action server.");
      // Now that cancellation succeeded, switch into GOAL_INACTIVE.
      updateButtons(ButtonSetting::IN_PROCESS);
      break;

    case CancelResp::ERROR_REJECTED:
      RCLCPP_WARN(node_->get_logger(),
                  "Cancel request was REJECTED by the action server.");
      // Keep GOAL_ACTIVE (since the server refused to cancel).
      updateButtons(ButtonSetting::GOAL_ACTIVE);
      break;

    case CancelResp::ERROR_GOAL_TERMINATED:
      updateButtons(ButtonSetting::GOAL_INACTIVE);
      break;

    case CancelResp::ERROR_UNKNOWN_GOAL_ID:
      updateButtons(ButtonSetting::GOAL_INACTIVE);
      break;

    default:
      RCLCPP_ERROR(node_->get_logger(),
                    "Unexpected return_code %u from cancel service.",
                    response->return_code);
      // Fallback: assume goal is still active
      updateButtons(ButtonSetting::GOAL_ACTIVE);
      break;
  }
}

}  // namespace roadmap_explorer

// ─── Pluginlib export ─────────────────────────────────────────────────────────
PLUGINLIB_EXPORT_CLASS(roadmap_explorer::RoadmapExplorerPanel, rviz_common::Panel)