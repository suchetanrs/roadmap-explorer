#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include "roadmap_explorer/Helpers.hpp"
#include "roadmap_explorer/util/EventLogger.hpp"

class PointCloudPublisher : public rclcpp::Node
{
public:
  PointCloudPublisher()
  : Node("pointcloud_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);

    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 10);

    // Timer to publish point cloud data
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PointCloudPublisher::publish_pointcloud, this));
  }

private:
  void publish_pointcloud()
  {
    auto pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // Setup point cloud data
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.width = 10201;
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);

    float x = 3.0f;      // Position in x-axis
    float y = 2.0f;      // Position in y-axis
    float z = 1.0f;     // Position in z-axis

    float roll = 0.0;      // Roll (rotation around x-axis)
    float pitch = 0.0;     // Pitch (rotation around y-axis)
    float yaw = 0.0;      // Yaw (rotation around z-axis)

    // Convert roll, pitch, yaw to a quaternion
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll * M_PI / 180, pitch * M_PI / 180, yaw * M_PI / 180);

    // Set the pose with the position and quaternion
    geometry_msgs::msg::PoseStamped pose_stamped;
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();
    pose_stamped.pose = pose;
    pose_stamped.header.frame_id = "base_link";
    pose_stamped.header.stamp = this->now();

    // Get the transformation matrix from the pose
    Eigen::Affine3f T_w_c_est = roadmap_explorer::getTransformFromPose(pose);
    // std::cout << T_w_c_est.translation() << std::endl;
    // std::cout << "**" << std::endl;
    // std::cout << T_w_c_est.rotation() << std::endl;
    double info_sum = 0;

    for (float dx = 0; dx <= 5; dx += 2.5) {
      for (float dy = -5; dy <= 5; dy += 2.5) {
        for (float dz = -5; dz <= 5; dz += 2.5) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          auto landmark_x = x + dx;
          auto landmark_y = y + dy;
          auto landmark_z = z + dz;
          // Define the point in the world coordinates
          Eigen::Vector3f p3d_w_eig(landmark_x, landmark_y, landmark_z);           // Adjust this as needed
          Eigen::Vector3f p3d_c_eig;

          // Define the covariance matrix Q (3x3)
          Eigen::Matrix3f Q = Eigen::Matrix3f::Identity();           // Adjust this as needed

          // Compute the information of the point
          float information = roadmap_explorer::computeInformationOfPointLocal(
            p3d_c_eig, p3d_w_eig,
            T_w_c_est, Q);
          info_sum += information;
          pcl::PointXYZI pclPoint;
          pclPoint.x = landmark_x;
          pclPoint.y = landmark_y;
          pclPoint.z = landmark_z;
          pclPoint.intensity = information * 100000;
          cloud.push_back(pclPoint);
          // Print the result
          // std::cout << information << ", ";
        }
      }
    }
    std::cout << "LOOP COMPLETE" << info_sum << std::endl;

    pcl::toROSMsg(cloud, *pointcloud_msg);

    pointcloud_msg->header.frame_id = "base_link";
    pointcloud_msg->header.stamp = this->now();

    publisher_->publish(*pointcloud_msg);
    pose_publisher_->publish(pose_stamped);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}
