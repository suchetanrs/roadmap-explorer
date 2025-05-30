#ifndef SENSOR_SIMULATOR_HPP_
#define SENSOR_SIMULATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_map_server/map_io.hpp>

#include "roadmap_explorer/util/GeometryUtils.hpp"
#include "roadmap_explorer/Parameters.hpp"
#include "roadmap_explorer/util/Logger.hpp"

namespace roadmap_explorer
{

struct WorldPoint
{
  WorldPoint(double xin, double yin)
  {
    x = xin;
    y = yin;
  }
  double x;
  double y;
};

/**
 * @brief Node that projects the robot’s field of view onto a persistent
 *        “explored map”, re-using the underlying values of the full map.
 */
class SensorSimulator
{
public:
  SensorSimulator(
    std::shared_ptr<nav2_util::LifecycleNode> node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros);
  
  ~SensorSimulator();

  void cleanupMap();

  bool saveMap(std::string instance_name, std::string base_path);

  bool loadMap(std::string instance_name, std::string base_path);

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  
  void timerCallback();
  
  void markRay(const geometry_msgs::msg::Pose & base_pose, double ray_angle);
  
  void updateAfterChangedGeometry(const nav_msgs::msg::OccupancyGrid::SharedPtr updated_msg);

  inline void cellToWorld(
    const nav_msgs::msg::OccupancyGrid & grid,
    std::size_t idx,
    double & wx,
    double & wy) const
  {
    const uint32_t w = grid.info.width;
    const double res = grid.info.resolution;
    const auto & org = grid.info.origin.position;

    const uint32_t col = idx % w;
    const uint32_t row = idx / w;

    wx = org.x + (static_cast<double>(col) + 0.5) * res;
    wy = org.y + (static_cast<double>(row) + 0.5) * res;
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr explored_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::recursive_mutex map_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  nav_msgs::msg::OccupancyGrid explored_map_;


  std::shared_ptr<nav2_util::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
  rclcpp::CallbackGroup::SharedPtr map_subscription_cb_group_;
  bool manual_cleanup_requested_;
};

}  // namespace roadmap_explorer

#endif  // SENSOR_SIMULATOR_HPP_
