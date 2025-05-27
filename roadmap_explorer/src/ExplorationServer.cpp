#include "roadmap_explorer/ExplorationServer.hpp"

using namespace std::chrono_literals;

namespace roadmap_explorer
{

ExplorationServer::ExplorationServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("roadmap_explorer_node", "", options)
{
  RCLCPP_INFO(get_logger(), "Created %s", get_name());
}

nav2_util::CallbackReturn ExplorationServer::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configured %s", get_name());
  node_ = shared_from_this();

  exploration_bt_ = std::make_shared<RoadmapExplorationBT>(node_);
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ExplorationServer::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating %s", get_name());
  server_active_ = true;
  action_server_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  action_server_ = rclcpp_action::create_server<ExploreAction>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "explore_area",
    std::bind(&ExplorationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ExplorationServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&ExplorationServer::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_cb_group_);

  // Create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ExplorationServer::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating %s", get_name());
  server_active_ = false;
  action_server_.reset();
  exploration_bt_->halt();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ExplorationServer::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s", get_name());
  exploration_bt_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ExplorationServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down %s", get_name());
  return nav2_util::CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse ExplorationServer::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const ExploreAction::Goal>/*goal*/)
{
  if (!server_active_) {
    RCLCPP_WARN(get_logger(), "Received goal while inactive -> rejecting");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(get_logger(), "Accepted new ExploreArea goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ExplorationServer::handle_cancel(
  const std::shared_ptr<GoalHandleExplore> goal_handle)
{
  RCLCPP_WARN(get_logger(), "Cancel requested");
  exploration_bt_->halt();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ExplorationServer::handle_accepted(
  const std::shared_ptr<GoalHandleExplore> goal_handle)
{
  // Spin the work off to a new thread so we don’t block the executor
  std::thread{std::bind(&ExplorationServer::execute, this, goal_handle)}.detach();
}

void ExplorationServer::publish_feedback(
  const std::shared_ptr<GoalHandleExplore> & goal_handle,
  const std::string & message)
{
  auto feedback = std::make_shared<ExploreAction::Feedback>();
  feedback->status_message = message;
  goal_handle->publish_feedback(feedback);
}

void ExplorationServer::execute(const std::shared_ptr<GoalHandleExplore> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Running exploration BT for goal");

  auto result = std::make_shared<ExploreAction::Result>();
  result->success = false;

  // Build the tree
  try {
    exploration_bt_->makeBTNodes();
    publish_feedback(goal_handle, "BT constructed, starting loop");

    // Run will loop internally; if user cancels mid-way, we’ll check once loop finishes
    exploration_bt_->run();

    // Check for cancellation
    if (goal_handle->is_canceling()) {
      RCLCPP_WARN(get_logger(), "Goal canceled; aborting");
      goal_handle->canceled(result);
      return;
    }

    // Otherwise, success
    result->success = true;
    RCLCPP_INFO(get_logger(), "Exploration succeeded");
    goal_handle->succeed(result);

  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Exploration error: %s", ex.what());
    result->success = false;
    result->error_message = ex.what();
    goal_handle->abort(result);
  }
}

}  // namespace roadmap_explorer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(roadmap_explorer::ExplorationServer)
