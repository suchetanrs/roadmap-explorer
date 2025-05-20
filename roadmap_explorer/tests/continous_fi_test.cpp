#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "roadmap_explorer/fisher_information/FisherInfoManager.hpp"

class PoseSafetyCheckerNode
{
public:
  PoseSafetyCheckerNode()
  {
    node_ = rclcpp::Node::make_shared("cont_fi_test");
    // Initialize FisherInformationManager with this node's shared pointer
    fisher_info_manager_ = std::make_shared<roadmap_explorer::FisherInformationManager>(node_);

    // Setup TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create a timer to periodically check pose safety (e.g., every 2 seconds)
    timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PoseSafetyCheckerNode::timerCallback, this));
  }

  rclcpp::Node::SharedPtr node_;

private:
  void timerCallback()
  {
    try {
      // Lookup the latest transform from map to base_link
      geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

      // Convert transform to Pose message
      geometry_msgs::msg::Pose robot_pose;
      robot_pose.position.x = transform.transform.translation.x;
      robot_pose.position.y = transform.transform.translation.y;
      robot_pose.position.z = transform.transform.translation.z;
      robot_pose.orientation = transform.transform.rotation;

      // Check if the current pose is safe
      float information;
      bool is_safe = fisher_info_manager_->isPoseSafe(robot_pose, true, information);

      RCLCPP_INFO(node_->get_logger(), "Pose is %s", is_safe ? "SAFE" : "UNSAFE");
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "Failed to get transform: %s", ex.what());
    }
  }

  // Member variables
  std::shared_ptr<roadmap_explorer::FisherInformationManager> fisher_info_manager_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto instance_fi_test = std::make_shared<PoseSafetyCheckerNode>();
  rclcpp::spin(instance_fi_test->node_);
  rclcpp::shutdown();
  return 0;
}
