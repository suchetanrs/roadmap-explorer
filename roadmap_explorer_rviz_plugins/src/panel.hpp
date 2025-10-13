#ifndef ROADMAP_EXPLORER_PANEL_HPP_
#define ROADMAP_EXPLORER_PANEL_HPP_

#include <memory>
#include <chrono>

#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QRadioButton>
#include <QButtonGroup>
#include <QTimer>
#include <QLineEdit>
#include <QComboBox>
#include <QLabel>
#include <QGridLayout>
#include <QDir>
#include <QEvent>
#include <QStringList>
#include <QString>
#include <QCheckBox>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>

#include "roadmap_explorer_msgs/action/explore.hpp"
#include "roadmap_explorer_msgs/srv/save_map_and_roadmap.hpp"

namespace roadmap_explorer
{

enum class ButtonSetting
{
  GOAL_ACTIVE = 0,
  GOAL_INACTIVE = 1,
  IN_PROCESS = 2
};

class ExplorationPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ExplorationPanel(QWidget * parent = nullptr);
  ~ExplorationPanel() override;

private Q_SLOTS:
  void onStartClicked();
  void onStopClicked();
  void onSaveClicked();

private:
  using Explore = roadmap_explorer_msgs::action::Explore;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Explore>;

  void spinnerOnGUI();

  void sendGoal();
  void cancelGoal();
  void updateButtons(ButtonSetting setting);
  uint16_t  currentMode() const;  // helper to read which radio is checked
  void setLog(const QString & text, bool error);

  // ── Action-related callbacks ────────────────────────────────────────────────
  void actionGoalResponseCallback(const GoalHandle::SharedPtr & goal_handle);
  void actionFeedbackCallback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const Explore::Feedback> feedback);
  void actionResultCallback(const GoalHandle::WrappedResult & result);
  void actionCancelCallback(std::shared_ptr<action_msgs::srv::CancelGoal_Response> response);

  // ── QT Members ──────────────────────────────────────────────────────────────
  // radio buttons
  QRadioButton * rb_new_session_;
  QRadioButton * rb_continue_terminated_;
  QRadioButton * rb_continue_saved_;
  QButtonGroup * mode_group_;

  // load ui
  QLineEdit * load_base_path_edit_;
  QLineEdit * session_name_edit_;

  // action buttons
  QPushButton * start_button_;
  QPushButton * stop_button_;

  // save ui
  QLineEdit * save_base_path_edit_;
  QLineEdit * save_map_name_edit_;
  QPushButton * save_button_;

  // log ui
  QLabel * log_box_;

  QTimer * ros_timer_;

  // ── ROS Members ──────────────────────────────────────────────────────────────
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Explore>::SharedPtr action_client_;
  GoalHandle::SharedPtr goal_handle_;

  rclcpp::Client<roadmap_explorer_msgs::srv::SaveMapAndRoadmap>::SharedPtr save_map_client_;
  bool panel_active_;
};

}  // namespace roadmap_explorer

#endif  // ROADMAP_EXPLORER_PANEL_HPP_
