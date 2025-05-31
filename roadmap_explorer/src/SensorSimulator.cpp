#include <tf2/utils.h>
#include <algorithm>
#include <cmath>
#include <chrono>

#include "roadmap_explorer/SensorSimulator.hpp"

using std::placeholders::_1;

namespace roadmap_explorer
{

SensorSimulator::SensorSimulator(
  std::shared_ptr<nav2_util::LifecycleNode> node,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros)
: node_(node)
{
  manual_cleanup_requested_ = false;
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
  
  should_map_ = false;
  explored_map_ = nullptr;
  latest_map_ = nullptr;
}

SensorSimulator::~SensorSimulator()
{
  explored_pub_->on_deactivate();
  explored_pub_.reset();
}

void SensorSimulator::cleanupMap()
{
  std::lock_guard<std::recursive_mutex> lock(map_mutex_);
  manual_cleanup_requested_ = true;
}

bool SensorSimulator::saveMap(std::string instance_name, std::string base_path)
{
  std::lock_guard<std::recursive_mutex> lock(map_mutex_);
  nav2_map_server::SaveParameters save_params;
  save_params.map_file_name = base_path + "/" + instance_name + "/" + instance_name;
  if (nav2_map_server::saveMapToFile(*explored_map_, save_params)) {
    LOG_INFO("Map saved successfully");
    return true;
  } else {
    LOG_ERROR("Failed to save the map");
    return false;
  }
}

bool SensorSimulator::loadMap(std::string instance_name, std::string base_path)
{
  std::lock_guard<std::recursive_mutex> lock(map_mutex_);
  if(nav2_map_server::loadMapFromYaml(base_path + "/" + instance_name + "/" + instance_name + ".yaml", *explored_map_) == nav2_map_server::LOAD_MAP_STATUS::LOAD_MAP_SUCCESS)
  {
    return true;
  }
  return false;
}

void SensorSimulator::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(map_mutex_);
  latest_map_ = msg;                         // always keep the freshest copy

  /* ---------- first ever map ------------------------------------------------ */
  if (!explored_map_) {
    explored_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(*msg);
    std::fill(explored_map_->data.begin(), explored_map_->data.end(), -1);
    return;
  }

  /* ---------- has geometry (size/origin/resolution) changed? ---------------- */
  const bool geometry_changed =
    explored_map_->info.width != msg->info.width ||
    explored_map_->info.height != msg->info.height ||
    std::fabs(
    explored_map_->info.resolution -
    msg->info.resolution) > 1e-6 ||
    explored_map_->info.origin.position.x != msg->info.origin.position.x ||
    explored_map_->info.origin.position.y != msg->info.origin.position.y;

  if (!geometry_changed && !manual_cleanup_requested_) { // nothing changed → just keep the previous explored grid
    return;
  }

  updateAfterChangedGeometry(msg);
  manual_cleanup_requested_ = false;
}

/* -------------------------------------------------------------------------- */

void SensorSimulator::updateAfterChangedGeometry(
  const nav_msgs::msg::OccupancyGrid::SharedPtr updated_msg)
{
  std::lock_guard<std::recursive_mutex> lock(map_mutex_);
  /* ---------- rebuild explored map with the new geometry -------------------- */
  nav_msgs::msg::OccupancyGrid old_explored = *explored_map_;  // keep a copy
  if(updated_msg != nullptr)
  {
    explored_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(*updated_msg);                                       // new metadata
  }
  /* new cell indices */
  const double new_res = explored_map_->info.resolution;
  const auto & new_org = explored_map_->info.origin.position;
  std::fill(explored_map_->data.begin(), explored_map_->data.end(), -1);

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
      new_col >= static_cast<int>(explored_map_->info.width) ||
      new_row >= static_cast<int>(explored_map_->info.height))
    {
      continue;               // world point lies outside the new map

    }
    const std::size_t new_idx =
      static_cast<std::size_t>(new_row) * explored_map_->info.width + new_col;

    explored_map_->data[new_idx] = updated_msg->data[new_idx];   // copy the refreshed value into the explored_map
  }

  LOG_INFO("Map geometry changed and rebuilt explored map " <<
    old_explored.info.width << ", " << old_explored.info.height << " -> " <<
    explored_map_->info.width << ", " << explored_map_->info.height);
}

void SensorSimulator::timerCallback()
{
  std::lock_guard<std::recursive_mutex> lock(map_mutex_);
  if(!should_map_)
  {
    return;
  }
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
  LOG_TRACE("Marking rays: " << min_angle << " to " << max_angle << " with " << delta_theta);
  for (double a = min_angle;
    a <= max_angle;
    a += delta_theta)
  {
    markRay(base_pose.pose, base_yaw + a);  // cast in world frame
  }

  explored_map_->header.stamp = node_->now();
  explored_map_->header.frame_id = explore_costmap_ros_->getGlobalFrameID();
  auto explored_map_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>(*explored_map_);
  LOG_DEBUG("Publishing explored map with address" << explored_map_msg.get());
  explored_pub_->publish(std::move(explored_map_msg));
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
    LOG_TRACE("Marking ray " << mx << ", " << my << " -> " << idx);

    explored_map_->data[idx] = latest_map_->data[idx];

    if (latest_map_->data[idx] > 80) {
      break;
    }
  }
}

}  // namespace roadmap_explorer
