#include <rclcpp/rclcpp.hpp>
#include "roadmap_explorer/fisher_information/FisherInfoManager.hpp"

int main(int argc, char ** argv)
{
  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);

  // Create the node
  auto node = rclcpp::Node::make_shared("fisher_info_manager_node");

  // Create the FisherInformationManager instance
  auto fisher_info_manager = std::make_shared<roadmap_explorer::FisherInformationManager>(node);

  // Create an executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Spin the executor
  executor.spin();

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
