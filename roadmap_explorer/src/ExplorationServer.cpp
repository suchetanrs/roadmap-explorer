#include "roadmap_explorer/ExplorationServer.hpp"

using namespace std::chrono_literals;

namespace roadmap_explorer
{

ExplorationServer::ExplorationServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("roadmap_explorer_node", "", options)
{
  RCLCPP_INFO(get_logger(), "Created %s", get_name());
  server_active_ = false;
  LOG_INFO("Creating exploration costmap instance");
  explore_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "roadmap_explorer_costmap", "", "roadmap_explorer_costmap");
  // Launch a thread to run the costmap node
  explore_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(explore_costmap_ros_);
  LOG_INFO("Created exploration costmap instance");
}

nav2_util::CallbackReturn ExplorationServer::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configured %s", get_name());
  node_ = shared_from_this();
  node_->declare_parameter("localisation_only_mode", false);
  node_->get_parameter("localisation_only_mode", localisation_only_mode_);

  explore_costmap_ros_->configure();

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
    "roadmap_explorer",
    std::bind(&ExplorationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ExplorationServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&ExplorationServer::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_cb_group_);
  
  explore_costmap_ros_->activate();
  
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
  exploration_bt_.reset();
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ExplorationServer::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s", get_name());
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
  (void)goal_handle;
  RCLCPP_WARN(get_logger(), "Cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ExplorationServer::handle_accepted(
  const std::shared_ptr<GoalHandleExplore> goal_handle)
{
  (void)goal_handle;
  RCLCPP_WARN(get_logger(), "Goal accepted, starting execution");
  // Spin the work off to a new thread so we don’t block the executor
  std::thread{std::bind(&ExplorationServer::execute, this, goal_handle)}.detach();
}

void ExplorationServer::publish_feedback(
  const std::shared_ptr<GoalHandleExplore> & goal_handle,
  const std::string & message)
{
  (void)message;
  auto feedback = std::make_shared<ExploreAction::Feedback>();
  goal_handle->publish_feedback(feedback);
}

void ExplorationServer::make_exploration_bt(bool localisation_only_mode)
{
  exploration_bt_ = std::make_shared<RoadmapExplorationBT>(node_, localisation_only_mode, explore_costmap_ros_);
  exploration_bt_->makeBTNodes();
}

void ExplorationServer::execute(const std::shared_ptr<GoalHandleExplore> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Running exploration BT for goal");

  auto result = std::make_shared<ExploreAction::Result>();
  result->success = false;
  if (!exploration_bt_) {
    RCLCPP_ERROR(get_logger(), "Exploration BT not initialized");
    make_exploration_bt(localisation_only_mode_);
    RCLCPP_ERROR(get_logger(), "Exploration BT initialized");
  }

  switch (goal_handle->get_goal()->exploration_bringup_mode) {
    case ExploreAction::Goal::NEW_EXPLORATION_SESSION:
      RCLCPP_ERROR(get_logger(), "New exploration session.");
      exploration_bt_.reset();
      RCLCPP_ERROR(get_logger(), "Exploration making new");
      make_exploration_bt(localisation_only_mode_);
      break;
    case ExploreAction::Goal::CONTINUE_FROM_TERMINATED_SESSION:
      RCLCPP_INFO(get_logger(), "Continue from terminated session. Nothing to reset.");
      break;
    case ExploreAction::Goal::CONTINUE_FROM_SAVED_SESSION:
      if(!localisation_only_mode_)
      {
        RCLCPP_ERROR(get_logger(), "You cannot continue from saved session when in mapping mode.");
        return;
      }
      else
      {
        // To be implemented
        RCLCPP_ERROR(get_logger(), "Continue from saved session is not implemented yet.");
        throw std::runtime_error("Continue from saved session is not implemented yet.");
        return;
      }
    default:
      throw std::runtime_error("Unknown exploration_bringup_mode value");
  }

  // Build the tree
  try {
    while (rclcpp::ok()) {
      auto error_code = exploration_bt_->tickOnceWithSleep();

      // Check for cancellation
      if (goal_handle->is_canceling()) {
        RCLCPP_WARN(get_logger(), "Goal canceled; aborting");
        exploration_bt_->halt();
        goal_handle->canceled(result);
        return;
      }

      if (error_code == ExploreActionResult::NO_ERROR) {
        continue;
      } else if (error_code == ExploreActionResult::NO_MORE_REACHABLE_FRONTIERS) {
        RCLCPP_INFO(get_logger(), "No more reachable frontiers found, exploration complete");
        result->success = true;
        result->error_code = ExploreActionResult::NO_MORE_REACHABLE_FRONTIERS;
        goal_handle->succeed(result);
        return;
      } else if (error_code == ExploreActionResult::NAV2_INTERNAL_FAULT) {
        RCLCPP_ERROR(get_logger(), "Internal fault during exploration, aborting");
        result->success = false;
        result->error_code = ExploreActionResult::NAV2_INTERNAL_FAULT;
        goal_handle->abort(result);
        return;
      } else {
        RCLCPP_ERROR(get_logger(), "Unknown error occurred during exploration");
        result->success = false;
        result->error_code = ExploreActionResult::UNKNOWN;
        goal_handle->abort(result);
        return;
      }
    }

  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Exploration error: %s", ex.what());
    result->success = false;
    result->error_code = ExploreActionResult::UNKNOWN;
    goal_handle->abort(result);
  }
}

}  // namespace roadmap_explorer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(roadmap_explorer::ExplorationServer)
