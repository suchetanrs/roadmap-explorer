#ifndef FITSLAM_RVIZ2_PANEL_HPP_
#define FITSLAM_RVIZ2_PANEL_HPP_

#ifndef Q_MOC_RUN
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#endif

#include <stdio.h>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPushButton>

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

class QLineEdit;

namespace rviz_plugin
{

    class FrontierExplorationPanel : public rviz_common::Panel
    {
        Q_OBJECT

    public:
        explicit FrontierExplorationPanel(QWidget *parent = 0);

        void topicCallback(std_msgs::msg::String::SharedPtr msg);

        void topic2Callback(std_msgs::msg::String::SharedPtr msg);

        void threadFunction(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> ptr)
        {
            ptr->spin();
        }

    protected Q_SLOTS:
        void onPauseClicked();

        void onPlayClicked();

    protected:
        rclcpp::Node::SharedPtr fitslam_panel_node_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription2_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr exploration_state_publisher_;
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> fitslam_panel_executor_;
        QHBoxLayout *topic_layout = new QHBoxLayout;
        QHBoxLayout *topic_layout2 = new QHBoxLayout;
        QHBoxLayout *topic_layout3 = new QHBoxLayout;
        QHBoxLayout *topic_layout4 = new QHBoxLayout;
        QHBoxLayout *topic_layout5 = new QHBoxLayout;
        QHBoxLayout *topic_layout6 = new QHBoxLayout;
        QHBoxLayout *topic_layout7 = new QHBoxLayout;
        QHBoxLayout *topic_layout8 = new QHBoxLayout;

        QLabel *topic_name1 = new QLabel("PLACEHOLDER_1");
        QLabel *topic_name2 = new QLabel("PLACEHOLDER_2");
        QLabel *topic_name3 = new QLabel("PLACEHOLDER_3");
        QLabel *topic_name4 = new QLabel("PLACEHOLDER_4");
        QLabel *topic_name5 = new QLabel("PLACEHOLDER_5");
        QLabel *topic_name6 = new QLabel("PLACEHOLDER_6");
        QPushButton *topic_button1 = new QPushButton("PAUSE EXPLORATION");
        QPushButton *topic_button2 = new QPushButton("PLAY EXPLORATION");
    };

} // end namespace rviz_plugin

#endif // FITSLAM_RVIZ2_PANEL_HPP_
