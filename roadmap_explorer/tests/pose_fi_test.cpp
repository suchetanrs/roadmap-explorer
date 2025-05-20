#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "roadmap_explorer/fisher_information/FisherInfoManager.hpp"

#include <map>
#include <tuple>
#include <cmath>
#include <limits>

class PoseSafetyCheckerNode
{
public:
  PoseSafetyCheckerNode(rclcpp::Node::SharedPtr node, std::string color_value)
  : node_(node)
  {
    color_value_ = color_value;
    // Initialize FisherInformationManager with this node's shared pointer.
    fisher_info_manager_ = std::make_shared<roadmap_explorer::FisherInformationManager>(node_);

    // Subscribe to the pose topic.
    pose_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose_topic", 10,
      std::bind(&PoseSafetyCheckerNode::poseCallback, this, std::placeholders::_1));

    // Create the publisher for MarkerArray.
    marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "pose_markers", 10);

    pose_array_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(
      "pose_markers_array",
      10);

    // Initialize global min/max information values.
    global_min_info_ = std::numeric_limits<float>::max();
    global_max_info_ = std::numeric_limits<float>::lowest();
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
  {
    // Check the safety of the pose and get its information value.
    float information;
    bool is_safe = fisher_info_manager_->isPoseSafe(pose_msg->pose, true, information);
    RCLCPP_WARN_STREAM(node_->get_logger(), "INFORMATION RECVD: " << information);
    RCLCPP_INFO(node_->get_logger(), "Received pose is %s", is_safe ? "SAFE" : "UNSAFE");
    RCLCPP_INFO_STREAM(
      node_->get_logger(), "Pose received is: "
        << pose_msg->pose.position.x << " , " << pose_msg->pose.position.y);

    // Create a new marker for the pose.
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = node_->now();
    marker.ns = "pose_markers";
    marker.id = marker_array_.markers.size();     // Unique id based on current count.
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose_msg->pose;
    marker.scale.x = 1.0;     // Scale of the triangle
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set a temporary color (it will be overwritten in updateMarkerColors()).
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // Define the triangle vertices (in the marker’s frame).
    geometry_msgs::msg::Point p1, p2, p3;
    p1.x = 0.0;
    p1.y = 0.0;
    p1.z = 0.3;
    p2.x = 1.3;
    p2.y = -0.75;
    p2.z = 0.3;
    p3.x = 1.3;
    p3.y = 0.75;
    p3.z = 0.3;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker.points.push_back(p3);

    // Add the new marker to the array.
    marker_array_.markers.push_back(marker);


    // Store the marker's information value (using the marker id as key).
    marker_information_map_[marker.id] = information;

    // Update global min/max if needed.
    if (information < global_min_info_) {
      global_min_info_ = information;
    }
    if (information > global_max_info_) {
      global_max_info_ = information;
    }

    // Re-calc the colors for every marker based on the updated information range.
    updateMarkerColors();

    // Publish the updated marker array.
    marker_publisher_->publish(marker_array_);

    pose_array_.poses.push_back(pose_msg->pose);
    pose_array_.header.frame_id = "map";
    pose_array_.header.stamp = node_->now();
    pose_array_pub_->publish(pose_array_);
  }

  // Iterate over the marker array and update each marker’s color.
  void updateMarkerColors()
  {
    // Compute the range; protect against division by zero.
    float range = global_max_info_ - global_min_info_;
    if (range == 0) {
      range = 1.0f;
    }
    for (auto & marker : marker_array_.markers) {
      // Look up the information value for this marker.
      auto it = marker_information_map_.find(marker.id);
      if (it != marker_information_map_.end()) {
        float info = it->second;
        // Normalize the value between 0 (min) and 1 (max).
        float normalized = (info - global_min_info_) / range;
        // Map the normalized value to a rainbow color.
        if (color_value_ == "rainbow") {
          marker.color = getRainbowColor(normalized);
        } else if (color_value_ == "blue_shade") {
          marker.color = getBlueShadeColor(normalized);
        } else if (color_value_ == "just_blue") {
          marker.color = getBlueColor(normalized);
        } else {
          throw std::runtime_error("Color scheme invalid");
        }
      }
    }
  }

  std_msgs::msg::ColorRGBA getBlueColor(float normalized)
  {
    std_msgs::msg::ColorRGBA color;
    color.r = 0.03f;
    color.g = 0.94f;
    color.b = 0.901f;
    color.a = 1.0f;
    return color;
  }

  // Map a normalized value [0,1] to a color on the rainbow.
  // Here, 0 corresponds to red (min info) and 1 to violet (max info).
  std_msgs::msg::ColorRGBA getRainbowColor(float normalized)
  {
    std_msgs::msg::ColorRGBA color;
    // Map normalized value to a hue angle between 0 and 270 degrees.
    float hue = normalized * 270.0f;     // 0 deg = red, 270 deg ≈ violet.
    float s = 1.0f;                      // Full saturation.
    float v = 1.0f;                      // Full brightness.
    // Convert from HSV to RGB.
    auto rgb = hsvToRgb(hue, s, v);
    color.r = std::get<0>(rgb);
    color.g = std::get<1>(rgb);
    color.b = std::get<2>(rgb);
    color.a = 1.0f;
    return color;
  }

  // Returns a shade of blue based on a normalized value [0,1].
  // This function linearly interpolates between a light blue and a dark blue.
  std_msgs::msg::ColorRGBA getBlueShadeColor(float normalized)
  {
    std_msgs::msg::ColorRGBA color;
    // Define light blue and dark blue colours.
    // Light Blue: approximately RGB(173, 216, 230) normalized to (0.678, 0.847, 0.902)
    // Dark Blue: approximately RGB(16, 77, 181) normalized to (0.0, 0.0, 0.545)
    const float light_blue_r = 0.03f, light_blue_g = 0.94f, light_blue_b = 0.901f;
    const float dark_blue_r = 0.062f, dark_blue_g = 0.301f, dark_blue_b = 0.709f;

    // Linearly interpolate between the two blue shades.
    color.r = (1 - normalized) * light_blue_r + normalized * dark_blue_r;
    color.g = (1 - normalized) * light_blue_g + normalized * dark_blue_g;
    color.b = (1 - normalized) * light_blue_b + normalized * dark_blue_b;
    color.a = 1.0f;
    return color;
  }

  // Helper to convert HSV (h in degrees, s and v in [0,1]) to RGB.
  std::tuple<float, float, float> hsvToRgb(float h, float s, float v)
  {
    float C = v * s;
    float X = C * (1 - std::fabs(std::fmod(h / 60.0f, 2) - 1));
    float m = v - C;
    float r1, g1, b1;
    r1 = 0;
    g1 = C;
    b1 = X;
    if (h >= 0 && h < 60) {
      r1 = C;
      g1 = X;
      b1 = 0;
    } else if (h >= 60 && h < 120) {
      r1 = X;
      g1 = C;
      b1 = 0;
    } else if (h >= 120 && h < 180) {
      r1 = 0;
      g1 = C;
      b1 = X;
    } else if (h >= 180 && h < 240) {
      r1 = 0;
      g1 = X;
      b1 = C;
    } else if (h >= 240 && h < 300) {
      r1 = X;
      g1 = 0;
      b1 = C;
    } else { // h >= 300 && h < 360
      r1 = C;
      g1 = 0;
      b1 = X;
    }
    float r = r1 + m;
    float g = g1 + m;
    float b = b1 + m;
    return std::make_tuple(r, g, b);
  }

  // Member variables
  std::shared_ptr<roadmap_explorer::FisherInformationManager> fisher_info_manager_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
  rclcpp::Node::SharedPtr node_;
  visualization_msgs::msg::MarkerArray marker_array_;
  geometry_msgs::msg::PoseArray pose_array_;
  std::string color_value_;

  // Map to store each marker's information value (using marker id as key).
  std::map<int, float> marker_information_map_;

  // Global bounds for information.
  float global_min_info_;
  float global_max_info_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::string arg_value;
  arg_value = argv[1];
  auto node = rclcpp::Node::make_shared("pose_fi_test");
  auto checker_node = std::make_shared<PoseSafetyCheckerNode>(node, arg_value);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
