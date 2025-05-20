#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "roadmap_explorer/Nav2Interface.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("nav2_interface_node");

  auto nav2_interface = std::make_shared<roadmap_explorer::Nav2Interface>(node);

  // Create a multithreaded executor
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  // Add nodes to the executor
  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}
