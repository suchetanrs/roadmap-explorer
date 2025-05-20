#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <chrono>
#include <fstream>
#include <vector>

class OccupancyGridCounter : public rclcpp::Node
{
public:
  OccupancyGridCounter()
  : Node("occupancy_grid_counter")
  {
    start_time_ = std::chrono::steady_clock::now();
    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "traversability_thresholded", 10,
      std::bind(&OccupancyGridCounter::occupancyGridCallback, this, std::placeholders::_1));

    // Timer to call logData every 1 second
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&OccupancyGridCounter::logData, this));

    std::ostringstream filename_stream;
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    filename_stream << "occupancy_grid_count_" << std::put_time(
      std::localtime(
        &now_time_t), "%Y%m%d_%H%M%S") << ".csv";
    fileName_ = filename_stream.str();
  }

private:
  void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    last_msg_ = msg;     // Store the latest message
  }

  void logData()
  {
    if (!last_msg_) {   // If no message has been received yet, skip
      return;
    }

    int known_cells = 0;

    for (auto cell : last_msg_->data) {
      if (cell != -1) {      // Known or lethal cell
        known_cells++;
      }
    }

    auto current_time = std::chrono::steady_clock::now();
    auto time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(
      current_time - start_time_).count();

    std::ofstream csv_file;
    csv_file.open(fileName_, std::ios_base::app);
    csv_file << time_taken << "," << known_cells << "\n";
    csv_file.close();
    std::cout << "Known cells: " << known_cells << std::endl;

    RCLCPP_INFO(
      this->get_logger(), "Counted %d known cells. Time taken: %.2f seconds.", known_cells,
      time_taken);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point start_time_;
  nav_msgs::msg::OccupancyGrid::SharedPtr last_msg_;   // Store the last received message
  std::string fileName_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OccupancyGridCounter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
