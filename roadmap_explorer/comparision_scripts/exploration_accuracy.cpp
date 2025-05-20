#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Message Filters includes
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

using namespace std::chrono_literals;

class PoseComparator : public rclcpp::Node
{
public:
  PoseComparator()
  : Node("pose_comparator"), latest_pair_received_(false)
  {
    // Open the CSV file and write the header.
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S");

    // Create a unique filename
    std::string filename = "pose_deviation_" + ss.str() + ".csv";

    csv_file_.open(filename, std::ios::out);
    if (!csv_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file!");
      throw std::runtime_error("Failed to open CSV file");
    }
    csv_file_ <<
      "x_gt,y_gt,z_gt,orient_x_gt,orient_y_gt,orient_z_gt,orient_w_gt,x_slam,y_slam,z_slam,orient_x_slam,orient_y_slam,orient_z_slam,orient_w_slam\n";

    // Initialize message_filters subscribers for the two topics.
    // The first topic "ground_truth_pose" is of type nav_msgs::msg::Odometry.
    gt_sub_.subscribe(this, "ground_truth_pose");
    // The second topic "robot_pose_slam" is of type geometry_msgs::msg::PoseStamped.
    slam_sub_.subscribe(this, "robot_pose_slam");

    // Set up the approximate time synchronizer.
    // The queue size (10) is a tunable parameter.
    sync_.reset(new Sync(MySyncPolicy(10), gt_sub_, slam_sub_));
    sync_->registerCallback(&PoseComparator::syncCallback, this);

    // Create a timer to log a CSV entry every 500ms.
    timer_ = this->create_wall_timer(
      500ms, std::bind(&PoseComparator::timerCallback, this));
  }

  ~PoseComparator()
  {
    if (csv_file_.is_open()) {
      csv_file_.close();
    }
  }

private:
  // Define the sync policy for two topics.
  typedef message_filters::sync_policies::ApproximateTime<
      nav_msgs::msg::Odometry,
      geometry_msgs::msg::PoseStamped>
    MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;

  // Message filters subscribers.
  message_filters::Subscriber<nav_msgs::msg::Odometry> gt_sub_;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> slam_sub_;
  std::shared_ptr<Sync> sync_;

  // Timer to trigger CSV logging.
  rclcpp::TimerBase::SharedPtr timer_;

  // CSV file stream.
  std::ofstream csv_file_;

  // Variables to store the latest synchronized messages.
  nav_msgs::msg::Odometry latest_gt_;
  geometry_msgs::msg::PoseStamped latest_slam_;
  bool latest_pair_received_;
  std::mutex mutex_;

  // This callback is called by the synchronizer whenever a pair of messages
  // (ground truth and SLAM) are received with closely matching timestamps.
  void syncCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & gt_msg,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & slam_msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_gt_ = *gt_msg;
    latest_slam_ = *slam_msg;
    latest_pair_received_ = true;
  }

  // Timer callback that runs every 500ms to compute the deviation and log the result.
  void timerCallback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_pair_received_) {
      RCLCPP_WARN(this->get_logger(), "No synchronized messages received yet.");
      return;
    }

    // Extract positions from the ground truth odometry.
    const auto & pose_gt = latest_gt_.pose.pose;
    // Extract positions from the SLAM pose.
    const auto & pose_slam = latest_slam_.pose;

    // Log the data in CSV format.
    csv_file_ << pose_gt.position.x << "," << pose_gt.position.y << "," << pose_gt.position.z <<
      "," << pose_gt.orientation.x << "," << pose_gt.orientation.y << "," <<
      pose_gt.orientation.z << "," << pose_gt.orientation.w << ","
              << pose_slam.position.x << "," << pose_slam.position.y << "," <<
      pose_slam.position.z << "," << pose_slam.orientation.x << "," << pose_slam.orientation.y <<
      "," << pose_slam.orientation.z << "," << pose_slam.orientation.w
              << "\n";
    csv_file_.flush();     // Ensure data is written to disk
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseComparator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
