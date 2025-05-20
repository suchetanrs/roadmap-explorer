#include "panel.hpp"

#include <stdio.h>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace rviz_plugin
{

FrontierExplorationPanel::FrontierExplorationPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  fitslam_panel_node_ = rclcpp::Node::make_shared("fitslam_panel_node");
  topic_layout->addWidget(topic_name1);

  topic_layout2->addWidget(topic_name2);

  topic_layout3->addWidget(topic_name3);

  topic_layout4->addWidget(topic_name4);

  topic_layout5->addWidget(topic_name5);

  topic_layout6->addWidget(topic_name6);

  topic_layout7->addWidget(topic_button1);

  topic_layout8->addWidget(topic_button2);

  // Lay out the topic field above the control widget.
  QVBoxLayout * layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  layout->addLayout(topic_layout2);
  layout->addLayout(topic_layout3);
  layout->addLayout(topic_layout4);
  layout->addLayout(topic_layout5);
  layout->addLayout(topic_layout6);
  layout->addLayout(topic_layout7);
  layout->addLayout(topic_layout8);
  connect(topic_button1, &QPushButton::clicked, this, &FrontierExplorationPanel::onPauseClicked);
  connect(topic_button2, &QPushButton::clicked, this, &FrontierExplorationPanel::onPlayClicked);
  setLayout(layout);

  // subscription_ = fitslam_panel_node_->create_subscription<std_msgs::msg::String>(
  //     "/topic2", 10,
  //     std::bind(&FrontierExplorationPanel::topicCallback, this, std::placeholders::_1));

  // subscription2_ = fitslam_panel_node_->create_subscription<std_msgs::msg::String>(
  //     "/topic1", 10,
  //     std::bind(&FrontierExplorationPanel::topic2Callback, this, std::placeholders::_1));

  exploration_state_publisher_ = fitslam_panel_node_->create_publisher<std_msgs::msg::Int32>(
    "/exploration_state", 10);

  fitslam_panel_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  fitslam_panel_executor_->add_node(fitslam_panel_node_);

  // spin the new executor in a new thread.
  std::thread t1(&FrontierExplorationPanel::threadFunction, this, fitslam_panel_executor_);
  t1.detach();
}

void FrontierExplorationPanel::onPauseClicked()
{
  QPushButton * button = qobject_cast<QPushButton *>(sender());
  if (button) {
    QString buttonText = button->text();
    RCLCPP_INFO(rclcpp::get_logger("rviz_plugin"), "Pause clicked");
    std_msgs::msg::Int32 msg;
    msg.data = 0;
    exploration_state_publisher_->publish(msg);
  }
}

void FrontierExplorationPanel::onPlayClicked()
{
  QPushButton * button = qobject_cast<QPushButton *>(sender());
  if (button) {
    QString buttonText = button->text();
    RCLCPP_INFO(rclcpp::get_logger("rviz_plugin"), "Play clicked");
    std_msgs::msg::Int32 msg;
    msg.data = 1;
    exploration_state_publisher_->publish(msg);
  }
}

void FrontierExplorationPanel::topicCallback(std_msgs::msg::String::SharedPtr msg)
{
  (void)msg;
  topic_name1->setText("");
  topic_name2->setText("");
  topic_name3->setText("");
  topic_name4->setText("");
  topic_name5->setText("");
  topic_name6->setText("");
}

void FrontierExplorationPanel::topic2Callback(std_msgs::msg::String::SharedPtr msg)
{
  (void)msg;
}

} // end namespace rviz_plugin

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
PLUGINLIB_EXPORT_CLASS(rviz_plugin::FrontierExplorationPanel, rviz_common::Panel)
// END_TUTORIAL
