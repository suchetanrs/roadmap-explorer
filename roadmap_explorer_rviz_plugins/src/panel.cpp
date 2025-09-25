#include "panel.hpp"

#include <pluginlib/class_list_macros.hpp>

using namespace std::chrono_literals;
namespace roadmap_explorer
{

ExplorationPanel::ExplorationPanel(QWidget * parent)
: rviz_common::Panel(parent),
  rb_new_session_(new QRadioButton("New exploration session", this)),
  rb_continue_terminated_(new QRadioButton("Continue from previous session", this)),
  rb_continue_saved_(new QRadioButton("Continue from saved session", this)),
  mode_group_(new QButtonGroup(this)),
  start_button_(new QPushButton("Start exploration", this)),
  stop_button_(new QPushButton("Stop exploration", this)),
  node_(std::make_shared<rclcpp::Node>("roadmap_explorer_rviz_panel")),
  action_client_(rclcpp_action::create_client<Explore>(node_, "roadmap_explorer")),
  save_map_client_(node_->create_client<roadmap_explorer_msgs::srv::SaveMapAndRoadmap>("save_map"))
{
  // Basic UI
  auto * layout = new QVBoxLayout;

  {
    // Put them all into a QButtonGroup so only one is checked at a time
    mode_group_->addButton(
      rb_new_session_,
      static_cast<int>(Explore::Goal::NEW_EXPLORATION_SESSION));
    mode_group_->addButton(
      rb_continue_terminated_,
      static_cast<int>(Explore::Goal::CONTINUE_FROM_TERMINATED_SESSION));
    mode_group_->addButton(
      rb_continue_saved_,
      static_cast<int>(Explore::Goal::CONTINUE_FROM_SAVED_SESSION));

    mode_group_->setExclusive(true);
    // Check the “New session” one by default
    rb_new_session_->setChecked(true);

    // Add them to the layout (stacked vertically)
    layout->addWidget(rb_new_session_);
    layout->addWidget(rb_continue_terminated_);
    layout->addWidget(rb_continue_saved_);

    // load from saved session ui. This will work only if the “Continue from saved session” is checked
    {
      auto * load_layout = new QHBoxLayout;
      auto default_path = QString::fromStdString(
        std::string(
          std::getenv(
            "HOME")) + "/.ros/roadmap-explorer");
      load_base_path_edit_ = new QLineEdit(default_path, this);
      session_name_edit_ = new QLineEdit("my_session", this);

      load_layout->addWidget(new QLabel("Folder:", this));
      load_layout->addWidget(load_base_path_edit_);
      load_layout->addWidget(new QLabel("Session name:", this));
      load_layout->addWidget(session_name_edit_);
      session_name_edit_->setEnabled(false);
      layout->addLayout(load_layout);

      /* disabled until “continue saved session” is chosen */
      load_base_path_edit_->setEnabled(false);
      session_name_edit_->setEnabled(false);

      connect(
        rb_continue_saved_, &QRadioButton::toggled, this,
        [this](bool checked) {
          load_base_path_edit_->setEnabled(checked);
          session_name_edit_->setEnabled(checked);
        });
    }
  }

  layout->addWidget(start_button_);
  connect(start_button_, &QPushButton::clicked, this, &ExplorationPanel::onStartClicked);
  layout->addWidget(stop_button_);
  connect(stop_button_, &QPushButton::clicked, this, &ExplorationPanel::onStopClicked);

  auto * separator = new QFrame(this);
  separator->setFrameShape(QFrame::HLine);
  // separator->setFrameShadow(QFrame::Sunken);
  separator->setStyleSheet("border: 1px solid green;");
  layout->addWidget(separator);

  // save-map UI
  {
    auto * save_layout = new QGridLayout;
    auto default_path = QString::fromStdString(
      std::string(
        std::getenv(
          "HOME")) + "/.ros/roadmap-explorer");
    save_base_path_edit_ = new QLineEdit(default_path, this);
    save_map_name_edit_ = new QLineEdit("my_session", this);
    save_button_ = new QPushButton("Save session", this);

    save_layout->addWidget(new QLabel("Save to folder:", this), 0, 0);
    save_layout->addWidget(save_base_path_edit_, 0, 1);
    save_layout->addWidget(new QLabel("Session name:", this), 1, 0);
    save_layout->addWidget(save_map_name_edit_, 1, 1);
    save_layout->addWidget(save_button_, 2, 0, 1, 2);
    layout->addLayout(save_layout);

    connect(
      save_button_, &QPushButton::clicked,
      this, &ExplorationPanel::onSaveClicked);
  }

  {
    // Create a horizontal layout with a label and a read‐only QLineEdit
    auto * log_layout = new QHBoxLayout;
    log_box_ = new QLabel("Status: Startup...", this);

    log_layout->addWidget(log_box_);
    layout->addLayout(log_layout);
  }
  setLayout(layout);

  // Initial state
  updateButtons(ButtonSetting::IN_PROCESS);

  // Start the ROS timer for spin. We use QTimer so that the GUI updates happen on the same thread.
  panel_active_ = false;
  ros_timer_ = new QTimer(this);
  ros_timer_->setInterval(100); // 100 ms
  connect(ros_timer_, &QTimer::timeout, this, &ExplorationPanel::spinnerOnGUI);
  ros_timer_->start();
}

void ExplorationPanel::spinnerOnGUI()
{
  rclcpp::spin_some(node_);
  if (!action_client_->wait_for_action_server(0ms)) {
    updateButtons(ButtonSetting::IN_PROCESS);
    setLog("Exploration action server unavailable", true);
    goal_handle_.reset();
    panel_active_ = false;
  }
  // else if (!save_map_client_->wait_for_service(0ms)) {
  //   updateButtons(ButtonSetting::IN_PROCESS);
  //   setLog("Save roadmap service unavailable", true);
  //   goal_handle_.reset();
  //   panel_active_ = false;
  // }
  /**
   *  this is set to GOAL_INACTIVE only if panel is not active because if it's active
   * then the button states are handled based on action request / results.
  */
  else if (!panel_active_) {
    updateButtons(ButtonSetting::GOAL_INACTIVE);
    panel_active_ = true;
    setLog("Exploration server active", false);
  }
}

// ───── Qt slots ──────────────────────────────────────────────────────────────
void ExplorationPanel::onStartClicked()
{
  if (!action_client_->wait_for_action_server(0ms)) {
    setLog("Action server unavailable: cannot send goal", true);
    return;
  }
  sendGoal();
}

void ExplorationPanel::onStopClicked()
{
  cancelGoal();
}

void ExplorationPanel::onSaveClicked()
{
  auto req = std::make_shared<roadmap_explorer_msgs::srv::SaveMapAndRoadmap::Request>();
  req->base_path = save_base_path_edit_->text().toStdString();
  req->session_name = save_map_name_edit_->text().toStdString();
  save_map_client_->async_send_request(req);
}

// ───── Internal ──────────────────────────────────────────────────────────────
void ExplorationPanel::sendGoal()
{
  Explore::Goal goal_msg;
  goal_msg.exploration_bringup_mode = this->currentMode();
  if (this->currentMode() == Explore::Goal::CONTINUE_FROM_SAVED_SESSION) {
    std_msgs::msg::String session_name;
    session_name.data = load_base_path_edit_->text().toStdString() +
      session_name_edit_->text().toStdString();
    goal_msg.load_from_folder = session_name;
  }

  auto goal_options = rclcpp_action::Client<Explore>::SendGoalOptions();
  goal_options.goal_response_callback =
    std::bind(&ExplorationPanel::actionGoalResponseCallback, this, std::placeholders::_1);
  goal_options.feedback_callback =
    std::bind(
    &ExplorationPanel::actionFeedbackCallback, this,
    std::placeholders::_1, std::placeholders::_2);
  goal_options.result_callback =
    std::bind(&ExplorationPanel::actionResultCallback, this, std::placeholders::_1);

  updateButtons(ButtonSetting::IN_PROCESS);
  action_client_->async_send_goal(goal_msg, goal_options);
}

void ExplorationPanel::cancelGoal()
{
  if (goal_handle_) {
    updateButtons(ButtonSetting::IN_PROCESS);
    action_client_->async_cancel_goal(
      goal_handle_,
      std::bind(&ExplorationPanel::actionCancelCallback, this, std::placeholders::_1));
  } else {
    updateButtons(ButtonSetting::GOAL_INACTIVE);
  }
}

void ExplorationPanel::updateButtons(ButtonSetting setting)
{
  if (setting == ButtonSetting::IN_PROCESS) {
    rb_new_session_->setEnabled(false);
    rb_continue_terminated_->setEnabled(false);
    rb_continue_saved_->setEnabled(false);

    // this is disabled here because of the need to disable the entire panel.
    load_base_path_edit_->setEnabled(false);
    session_name_edit_->setEnabled(false);

    start_button_->setEnabled(false);
    stop_button_->setEnabled(false);

    save_base_path_edit_->setEnabled(false);
    save_map_name_edit_->setEnabled(false);
    save_button_->setEnabled(false);
  } else {
    rb_new_session_->setEnabled(setting == ButtonSetting::GOAL_INACTIVE);
    rb_continue_terminated_->setEnabled(setting == ButtonSetting::GOAL_INACTIVE);
    rb_continue_saved_->setEnabled(setting == ButtonSetting::GOAL_INACTIVE);

    // load_base_path_edit_ and load_folder_combo_ are not enabled here because they are automatically enabled when continue from saved session is selected

    start_button_->setEnabled(setting == ButtonSetting::GOAL_INACTIVE);
    stop_button_->setEnabled(setting == ButtonSetting::GOAL_ACTIVE);

    save_base_path_edit_->setEnabled(setting == ButtonSetting::GOAL_INACTIVE);
    save_map_name_edit_->setEnabled(setting == ButtonSetting::GOAL_INACTIVE);
    save_button_->setEnabled(setting == ButtonSetting::GOAL_INACTIVE);
  }
}

uint16_t ExplorationPanel::currentMode() const
{
  // use with the same IDs as the constants in the action:
  //   0 = NEW_EXPLORATION_SESSION
  //   1 = CONTINUE_FROM_TERMINATED_SESSION
  //   2 = CONTINUE_FROM_SAVED_SESSION
  int id = mode_group_->checkedId();
  if (id < 0) {
    // Fallback: if somehow none is checked (should not happen), default to 0.
    RCLCPP_WARN(
      node_->get_logger(),
      "No exploration bringup mode was specified, resorting to new exploration session");
    return Explore::Goal::NEW_EXPLORATION_SESSION;
  }
  return static_cast<uint16_t>(id);
}

void ExplorationPanel::setLog(const QString & text, bool error)
{
  if (error) {
    QString html = QString("<span style=\"color:red;\">Status: %1</span>").arg(text);
    log_box_->setText(html);
    // log_box_->setTextFormat(Qt::RichText);
  } else {
    log_box_->setText("Status: " + text);
  }
}

// ───── Action callbacks ──────────────────────────────────────────────────────
void ExplorationPanel::actionGoalResponseCallback(
  const GoalHandle::SharedPtr & goal_handle)
{
  goal_handle_ = goal_handle;
  if (!goal_handle_) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by action server");
    updateButtons(ButtonSetting::GOAL_INACTIVE);
    return;
  }
  updateButtons(ButtonSetting::GOAL_ACTIVE);
}

void ExplorationPanel::actionFeedbackCallback(
  GoalHandle::SharedPtr,
  const std::shared_ptr<const Explore::Feedback> feedback)
{
  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "Exploring for %.1fs, explored frontier count: %zu",
    rclcpp::Duration(feedback->exploration_time).seconds(),
    feedback->current_frontier.size());
}

void ExplorationPanel::actionResultCallback(const GoalHandle::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(
        node_->get_logger(), "Exploration finished successfully: %s",
        result.result->success ? "true" : "false");
      break;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(
        node_->get_logger(),
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
  rb_continue_terminated_->setChecked(true);
}

void ExplorationPanel::actionCancelCallback(
  std::shared_ptr<action_msgs::srv::CancelGoal_Response> response)
{
  using CancelResp = action_msgs::srv::CancelGoal_Response;
  if (!goal_handle_) {
    RCLCPP_INFO(node_->get_logger(), "Cancel response received, but no goal was active.");
    return;
  }

  switch (response->return_code) {
    case CancelResp::ERROR_NONE:
      RCLCPP_INFO(
        node_->get_logger(),
        "Cancel request was ACCEPTED by the action server.");
      // Now that cancellation succeeded, switch into GOAL_INACTIVE.
      updateButtons(ButtonSetting::IN_PROCESS);
      break;

    case CancelResp::ERROR_REJECTED:
      RCLCPP_WARN(
        node_->get_logger(),
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
      RCLCPP_ERROR(
        node_->get_logger(),
        "Unexpected return_code %u from cancel service.",
        response->return_code);
      // Fallback: assume goal is still active
      updateButtons(ButtonSetting::GOAL_ACTIVE);
      break;
  }
}

}  // namespace roadmap_explorer

// ─── Pluginlib export ─────────────────────────────────────────────────────────
PLUGINLIB_EXPORT_CLASS(roadmap_explorer::ExplorationPanel, rviz_common::Panel)
