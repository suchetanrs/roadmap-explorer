#include "rclcpp/rclcpp.hpp"
#include <roadmap_explorer/ExplorationBT.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("roadmap_explorer_node");
  auto exploration_server = std::make_shared<roadmap_explorer::FrontierExplorationServer>(node);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  std::thread spin_thread([executor]() {
    executor->spin();
  });
  spin_thread.detach();

  exploration_server->makeBTNodes();
  exploration_server->run();
  LOG_WARN("Exiting main thread, stopping exploration server.");
  
  exploration_server.reset();
  LOG_WARN("Resetted exploration server.");
  
  if(spin_thread.joinable())
  {
    spin_thread.join();
  }
  LOG_WARN("Joined spin thread.");
  executor.reset();
  LOG_WARN("Resetted executor.");
  rclcpp::shutdown();
  LOG_WARN("Shutdown complete.");
  return 0;
}
