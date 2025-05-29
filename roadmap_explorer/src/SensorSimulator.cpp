#include "roadmap_explorer/SensorSimulator.hpp"

#include <tf2/utils.h>
#include <algorithm>
#include <cmath>
#include <chrono>

using std::placeholders::_1;

namespace roadmap_explorer
{

SensorSimulator::SensorSimulator(
  std::shared_ptr<nav2_util::LifecycleNode> node,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros)
: node_(node)
{
  explore_costmap_ros_ = explore_costmap_ros;
  map_subscription_cb_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = map_subscription_cb_group_;

  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    parameterInstance.getValue<std::string>(
      "sensorSimulator.input_map_topic"), rclcpp::SensorDataQoS(),
    std::bind(&SensorSimulator::mapCallback, this, _1), options);

  explored_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
    parameterInstance.getValue<std::string>("sensorSimulator.explored_map_topic"), 10);
  explored_pub_->on_activate();

  timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(
      parameterInstance.getValue<double>(
        "sensorSimulator.sensor_update_rate")),
    std::bind(&SensorSimulator::timerCallback, this), map_subscription_cb_group_);
}

SensorSimulator::~SensorSimulator()
{
  explored_pub_->on_deactivate();
}

void SensorSimulator::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(map_mutex_);
  latest_map_ = msg;                         // always keep the freshest copy

  /* ---------- first ever map ------------------------------------------------ */
  if (explored_map_.data.empty()) {
    explored_map_ = *msg;
    std::fill(explored_map_.data.begin(), explored_map_.data.end(), -1);
    return;
  }

  /* ---------- has geometry (size/origin/resolution) changed? ---------------- */
  const bool geometry_changed =
    explored_map_.info.width != msg->info.width ||
    explored_map_.info.height != msg->info.height ||
    std::fabs(
    explored_map_.info.resolution -
    msg->info.resolution) > 1e-6 ||
    explored_map_.info.origin.position.x != msg->info.origin.position.x ||
    explored_map_.info.origin.position.y != msg->info.origin.position.y;

  if (!geometry_changed) { // nothing changed → just keep the previous explored grid
    return;
  }

  updateAfterChangedGeometry(msg);
}

/* -------------------------------------------------------------------------- */

void SensorSimulator::updateAfterChangedGeometry(
  const nav_msgs::msg::OccupancyGrid::SharedPtr updated_msg)
{
  std::lock_guard<std::recursive_mutex> lock(map_mutex_);
  /* ---------- rebuild explored map with the new geometry -------------------- */
  nav_msgs::msg::OccupancyGrid old_explored = explored_map_;  // keep a copy
  explored_map_ = *updated_msg;                                       // new metadata
  /* new cell indices */
  const double new_res = explored_map_.info.resolution;
  const auto & new_org = explored_map_.info.origin.position;
  std::fill(explored_map_.data.begin(), explored_map_.data.end(), -1);

  /* ---------- project every previously known cell into the new grid --------- */
  for (std::size_t idx = 0; idx < old_explored.data.size(); ++idx) {
    const int8_t val = old_explored.data[idx];
    if (val == -1) {          // cell was still unknown → skip
      continue;
    }

    /* old cell centre in world coordinates */
    double wx, wy;
    cellToWorld(old_explored, idx, wx, wy);

    const int new_col = static_cast<int>((wx - new_org.x) / new_res);
    const int new_row = static_cast<int>((wy - new_org.y) / new_res);

    if (new_col < 0 || new_row < 0 ||
      new_col >= static_cast<int>(explored_map_.info.width) ||
      new_row >= static_cast<int>(explored_map_.info.height))
    {
      continue;               // world point lies outside the new map

    }
    const std::size_t new_idx =
      static_cast<std::size_t>(new_row) * explored_map_.info.width + new_col;

    explored_map_.data[new_idx] = updated_msg->data[new_idx];   // copy the refreshed value into the explored_map
  }

  LOG_INFO("Map geometry changed and rebuilt explored map " <<
    old_explored.info.width << ", " << old_explored.info.height << " -> " <<
    explored_map_.info.width << ", " << explored_map_.info.height);
}

void SensorSimulator::timerCallback()
{
  std::lock_guard<std::recursive_mutex> lock(map_mutex_);
  LOG_DEBUG("TIMER CB");
  if (!latest_map_) {
    LOG_ERROR("No latest map");
    return;  // no map yet
  }

  geometry_msgs::msg::PoseStamped base_pose;
  if(!explore_costmap_ros_->getRobotPose(base_pose))
  {
    LOG_ERROR("Could not get robot pose. Returning.");
    return;
  }
  const double base_yaw = roadmap_explorer::quatToEuler(base_pose.pose.orientation)[2];

  /* ----------------------------- ray-casting ----------------------------- */
  auto min_angle = parameterInstance.getValue<double>("sensorSimulator.sensor_min_angle");
  auto max_angle = parameterInstance.getValue<double>("sensorSimulator.sensor_max_angle");
  auto delta_theta = parameterInstance.getValue<double>("sensorSimulator.angular_resolution");
  for (double a = min_angle;
    a <= max_angle;
    a += delta_theta)
  {
    markRay(base_pose.pose, base_yaw + a);  // cast in world frame
  }

  explored_map_.header.stamp = node_->now();
  explored_map_.header.frame_id = explore_costmap_ros_->getGlobalFrameID();
  explored_pub_->publish(explored_map_);
}

void SensorSimulator::markRay(
  const geometry_msgs::msg::Pose & base_pose,
  double ray_angle)
{
  std::lock_guard<std::recursive_mutex> lock(map_mutex_);
  const double res = latest_map_->info.resolution;
  const int w = static_cast<int>(latest_map_->info.width);
  const int h = static_cast<int>(latest_map_->info.height);
  const auto & org = latest_map_->info.origin.position;

  const double dx = std::cos(ray_angle);
  const double dy = std::sin(ray_angle);
  auto sensor_range = parameterInstance.getValue<double>("sensorSimulator.sensor_max_range");

  for (double r = 0.0; r <= sensor_range; r += res) {
    const double wx = base_pose.position.x + r * dx;   // world
    const double wy = base_pose.position.y + r * dy;

    const int mx = static_cast<int>((wx - org.x) / res); // map cell
    const int my = static_cast<int>((wy - org.y) / res);

    if (mx < 0 || my < 0 || mx >= w || my >= h) {
      break;
    }

    const std::size_t idx = static_cast<std::size_t>(my) * w + mx;

    explored_map_.data[idx] = latest_map_->data[idx];

    if (latest_map_->data[idx] > 80) {
      break;
    }
  }
}

}  // namespace roadmap_explorer
