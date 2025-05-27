#ifndef ROADMAP_EXPLORER_PANEL_HPP_
#define ROADMAP_EXPLORER_PANEL_HPP_

#include <memory>
#include <chrono>

#include <QPushButton>
#include <QVBoxLayout>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>

#include <roadmap_explorer_msgs/action/explore.hpp>

namespace rviz_plugin
{

class RoadmapExplorerPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit RoadmapExplorerPanel(QWidget * parent = nullptr);
  ~RoadmapExplorerPanel() override = default;

private Q_SLOTS:
  void onStartClicked();
  void onStopClicked();

private:
  using Explore = roadmap_explorer_msgs::action::Explore;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Explore>;

  void sendGoal();
  void cancelGoal();
  void updateButtons(bool goal_active);

  // ── Action-related callbacks ────────────────────────────────────────────────
  void goalResponseCallback(const GoalHandle::SharedPtr & goal_handle);
  void feedbackCallback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const Explore::Feedback> feedback);
  void resultCallback(const GoalHandle::WrappedResult & result);

  // ── Members ────────────────────────────────────────────────────────────────
  QPushButton * start_button_;
  QPushButton * stop_button_;

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Explore>::SharedPtr client_;
  GoalHandle::SharedPtr goal_handle_;

  // spin_some() from Qt side so we don’t need a separate thread
  QTimer spin_timer_;
};

}  // namespace rviz_plugin

#endif  // ROADMAP_EXPLORER_PANEL_HPP_