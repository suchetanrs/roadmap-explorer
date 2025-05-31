#include "rclcpp/rclcpp.hpp"
#include <roadmap_explorer/ExplorationServer.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  options.use_intra_process_comms(true);
  auto exploration_server = std::make_shared<roadmap_explorer::ExplorationServer>(
    options);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(exploration_server->get_node_base_interface());

  executor->spin();
  rclcpp::shutdown();
  LOG_WARN("Shutdown complete.");
  return 0;
}
