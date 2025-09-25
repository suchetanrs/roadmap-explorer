#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>

class PathVisualizer : public rclcpp::Node
{
public:
  PathVisualizer()
  : Node("path_visualizer"), marker_id_(0), point_count_(0)
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "exploration_path_followed", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PathVisualizer::timer_callback, this));

    start_time_ = std::chrono::high_resolution_clock::now();
    total_distance_ = 0;

    this->declare_parameter<std::string>("global_frame_id", "map");
    this->get_parameter("global_frame_id", global_frame_id_);

    this->declare_parameter<std::string>("base_frame_id", "base_link");
    this->get_parameter("base_frame_id", base_frame_id_);
  }

private:
  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      // Look for the transformation between "map" and "base_link"
      if (static_cast<std::string>(this->get_namespace()) == "/") {
        transform_stamped = tf_buffer_->lookupTransform(global_frame_id_, base_frame_id_, tf2::TimePointZero);
      } else {
        transform_stamped =
          tf_buffer_->lookupTransform(
          global_frame_id_,
          static_cast<std::string>(this->get_namespace()) + "/" + base_frame_id_, tf2::TimePointZero);
      }
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform map to base_link: %s", ex.what());
      return;
    }

    geometry_msgs::msg::Point point;
    point.x = transform_stamped.transform.translation.x;
    point.y = transform_stamped.transform.translation.y;
    point.z = transform_stamped.transform.translation.z;

    points_.push_back(point);
    colors_.push_back(compute_color(point_count_));
    point_count_++;

    visualization_msgs::msg::MarkerArray marker_array;
    
    // First, delete all existing markers in the namespace
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = global_frame_id_;
    delete_marker.header.stamp = transform_stamped.header.stamp;
    delete_marker.ns = "path";
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    
    // Then add the new line strip
    visualization_msgs::msg::Marker line_strip;
    line_strip.header.frame_id = global_frame_id_;
    line_strip.header.stamp = transform_stamped.header.stamp;
    line_strip.ns = "path";
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 0;  // Use fixed ID since we're clearing all markers first
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.scale.x = 0.25;     // Line width

    for (size_t i = 0; i < points_.size(); ++i) {
      line_strip.points.push_back(points_[i]);
      line_strip.colors.push_back(compute_color(i));
    }

    marker_array.markers.push_back(line_strip);
    marker_pub_->publish(marker_array);

    // Log information
    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_time = current_time - start_time_;
    if (elapsed_time.count() > 0) {
      double average_speed = total_distance_ / elapsed_time.count();
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 500, "Total distance travelled: %f m", total_distance_);
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 500, "Total time taken: %f s", elapsed_time.count());
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 500, "Average Speed: %f m/s", average_speed);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "=======================");
    }
  }

  std_msgs::msg::ColorRGBA compute_color(size_t index)
  {
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;     // Fully opaque

    // Hue calculation: map the index to a value between 0 and 360 (degrees)
    float hue = static_cast<float>(index) / points_.size() * 360.0f;

    // Convert HSV to RGB
    float c = 1.0f;     // Chroma is full (since saturation is 1)
    float fHPrime = fmod(hue / 60.0, 6);
    float x = (1 - fabs(fmod(fHPrime, 2) - 1));
    float m = 0.0f;     // Value is 1, so m = 0 in RGB

    float r_prime = 0.0f, g_prime = 0.0f, b_prime = 0.0f;
    if (0 <= hue && hue < 60) {
      r_prime = c;
      g_prime = x;
      b_prime = 0;
    } else if (60 <= hue && hue < 120) {
      r_prime = x;
      g_prime = c;
      b_prime = 0;
    } else if (120 <= hue && hue < 180) {
      r_prime = 0;
      g_prime = c;
      b_prime = x;
    } else if (180 <= hue && hue < 240) {
      r_prime = 0;
      g_prime = x;
      b_prime = c;
    } else if (240 <= hue && hue < 300) {
      r_prime = x;
      g_prime = 0;
      b_prime = c;
    } else if (300 <= hue && hue < 360) {
      r_prime = c;
      g_prime = 0;
      b_prime = x;
    }

    color.r = r_prime + m;
    color.g = g_prime + m;
    color.b = b_prime + m;

    return color;
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<geometry_msgs::msg::Point> points_;
  std::vector<std_msgs::msg::ColorRGBA> colors_;
  std::chrono::_V2::system_clock::time_point start_time_;
  double total_distance_;
  size_t marker_id_;
  size_t point_count_;

  std::string global_frame_id_;
  std::string base_frame_id_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
