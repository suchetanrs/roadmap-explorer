#include "roadmap_explorer/controllers/initialization_controller.hpp"
#include "geometry_msgs/msg/twist.hpp"

InitCommandVelNode::InitCommandVelNode()
    : Node("init_command_vel_node"),
      forward_duration_(8.0),  // Time to move forward in seconds
      rotation_duration_(4.0), // Time to rotate 360 degrees in seconds
      forward_speed_(0.35),    // Speed to move forward in m/s (0.35 m/s for 2 sec -> 0.7 m)
      angular_speed_(M_PI / 6) // Speed to rotate in rad/s (~60 degrees/sec for 6 sec -> 360 degrees)
{
    // Create publisher on /cmd_vel topic
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_nav", 10);

    // Set the initial state for the movement sequence
    start_time_ = this->now();
    moving_forward_ = true;
    running_ = true;
    once_ = false;
}

bool InitCommandVelNode::send_velocity_commands()
{
    if(!once_)
    {
        once_ = true;
        start_time_ = this->now();
    }
    geometry_msgs::msg::Twist cmd_vel_msg;
    auto current_time = this->now();
    auto elapsed_time = (current_time - start_time_).seconds();

    if (moving_forward_)
    {
        // Move forward for `forward_duration_` seconds
        if (elapsed_time < forward_duration_)
        {
            cmd_vel_msg.linear.x = forward_speed_; // Set forward speed
        }
        else
        {
            moving_forward_ = false;  // Stop forward motion
            start_time_ = current_time; // Reset timer for rotation
        }
    }

    if (!moving_forward_)
    {
        // Rotate for `rotation_duration_` seconds
        if (elapsed_time < rotation_duration_)
        {
            cmd_vel_msg.angular.z = angular_speed_; // Set angular velocity for rotation
        }
        else
        {
            // Stop motion after rotation is complete
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = 0.0;
            running_ = false;
        }
    }

    // Publish the velocity command
    cmd_vel_pub_->publish(cmd_vel_msg);
    return running_;
}