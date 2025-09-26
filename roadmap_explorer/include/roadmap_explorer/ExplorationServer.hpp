#ifndef ROADMAP_EXPLORER__EXPLORATION_SERVER_HPP_
#define ROADMAP_EXPLORER__EXPLORATION_SERVER_HPP_

#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_util/lifecycle_node.hpp>

#include "roadmap_explorer/ExplorationBT.hpp"
#include "roadmap_explorer_msgs/action/explore.hpp"

namespace roadmap_explorer
{

enum class ActionTerminalState { SUCCEED, ABORT, CANCEL };

class ExplorationServer : public nav2_util::LifecycleNode
{
public:
  using ExploreAction = roadmap_explorer_msgs::action::Explore;
  using GoalHandleExplore = rclcpp_action::ServerGoalHandle<ExploreAction>;

  explicit ExplorationServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ExplorationServer() = default;

protected:
  // Lifecycle transitions
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  // Action‐server callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExploreAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExplore> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleExplore> goal_handle);

  // Spawned per‐goal execution
  void execute(const std::shared_ptr<GoalHandleExplore> goal_handle);

  void terminateGoal(
    const ActionTerminalState terminal_state,
    const std::shared_ptr<GoalHandleExplore> goal_handle,
    std::shared_ptr<ExploreAction::Result> result);

  void publish_feedback(
    const std::shared_ptr<GoalHandleExplore> & goal_handle,
    const std::string & message);
  
  bool make_exploration_bt(bool exploration_mode);

  // Members
  rclcpp_action::Server<ExploreAction>::SharedPtr action_server_;
  std::shared_ptr<RoadmapExplorationBT> exploration_bt_;
  bool server_active_{false};
  bool localisation_only_mode_{false};

  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  rclcpp::CallbackGroup::SharedPtr action_server_cb_group_;
  std::shared_ptr<nav2_util::LifecycleNode> node_;
};

}  // namespace roadmap_explorer

#endif  // ROADMAP_EXPLORER__EXPLORATION_SERVER_HPP_
