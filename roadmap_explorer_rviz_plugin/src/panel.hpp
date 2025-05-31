#ifndef ROADMAP_EXPLORER_PANEL_HPP_
#define ROADMAP_EXPLORER_PANEL_HPP_

#include <memory>
#include <chrono>

#include <QPushButton>
#include <QVBoxLayout>
#include <QRadioButton>
#include <QButtonGroup>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>

#include <roadmap_explorer_msgs/action/explore.hpp>

namespace roadmap_explorer
{

enum class ButtonSetting
{
  GOAL_ACTIVE = 0,
  GOAL_INACTIVE = 1,
  IN_PROCESS = 2
};

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
  void updateButtons(ButtonSetting setting);
  uint16_t  currentMode() const;  // helper to read which radio is checked

  // ── Action-related callbacks ────────────────────────────────────────────────
  void goalResponseCallback(const GoalHandle::SharedPtr & goal_handle);
  void feedbackCallback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const Explore::Feedback> feedback);
  void resultCallback(const GoalHandle::WrappedResult & result);
  void cancelCallback(std::shared_ptr<action_msgs::srv::CancelGoal_Response> response);

  // ── QT Members ────────────────────────────────────────────────────────────────
  QRadioButton * rb_new_session_;
  QRadioButton * rb_continue_terminated_;
  QRadioButton * rb_continue_saved_;
  QButtonGroup  * mode_group_;

  QPushButton * start_button_;
  QPushButton * stop_button_;
  
  std::unique_ptr<std::thread> spin_thread_;

  // ── ROS Members ────────────────────────────────────────────────────────────────
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Explore>::SharedPtr client_;
  GoalHandle::SharedPtr goal_handle_;
};

}  // namespace roadmap_explorer

#endif  // ROADMAP_EXPLORER_PANEL_HPP_