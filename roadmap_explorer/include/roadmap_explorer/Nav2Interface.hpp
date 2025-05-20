#ifndef NAV2_INTERFACE_HPP
#define NAV2_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <memory>
#include <string>
#include "roadmap_explorer/util/Logger.hpp"

// 0  - processing
// 1  - succeeded, no goal active
// -1 - failure

namespace roadmap_explorer
{
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    class Nav2Interface
    {
    public:
        Nav2Interface(rclcpp::Node::SharedPtr node);
        ~Nav2Interface();
        void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void cancelAllGoalsCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
        void cancelAllGoals();
        void sendGoal(geometry_msgs::msg::PoseStamped &goal_pose);
        int goalStatus()
        {
            std::lock_guard<std::mutex> lock(goal_active_mutex_);
            return nav2_goal_state_;
        }

    private:
        void nav2GoalFeedbackCallback(GoalHandleNav2::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);
        void nav2GoalResultCallback(const GoalHandleNav2::WrappedResult &result);
        void nav2GoalResponseCallback(const GoalHandleNav2::SharedPtr &goal_handle);
        
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<NavigateToPose>::SharedPtr nav2Client_;
        std::mutex nav2Clientlock_;
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions nav2_goal_options_;
        rclcpp::CallbackGroup::SharedPtr nav2_client_callback_group_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr cancel_goal_subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr updated_goal_publisher_;

        std::mutex goal_active_mutex_;
        int nav2_goal_state_; 
    };
}

#endif // NAV2_INTERFACE_HPP