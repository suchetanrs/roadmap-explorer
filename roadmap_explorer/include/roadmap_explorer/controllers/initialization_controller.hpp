#ifndef INIT_COMMAND_VEL_NODE_HPP_
#define INIT_COMMAND_VEL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class InitCommandVelNode : public rclcpp::Node
{
public:
  InitCommandVelNode();

  bool send_velocity_commands();

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;

  bool moving_forward_;
  double forward_duration_;
  double rotation_duration_;
  double forward_speed_;
  double angular_speed_;
  bool running_;
  bool once_;
};

#endif  // INIT_COMMAND_VEL_NODE_HPP_
