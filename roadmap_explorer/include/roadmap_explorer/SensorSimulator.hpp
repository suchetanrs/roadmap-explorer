/**
    Copyright 2025 Suchetan Saravanan.

    Licensed to the Apache Software Foundation (ASF) under one
    or more contributor license agreements.  See the NOTICE file
    distributed with this work for additional information
    regarding copyright ownership.  The ASF licenses this file
    to you under the Apache License, Version 2.0 (the
    "License"); you may not use this file except in compliance
    with the License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing,
    software distributed under the License is distributed on an
    "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
    KIND, either express or implied.  See the License for the
    specific language governing permissions and limitations
    under the License.
*/

#ifndef SENSOR_SIMULATOR_HPP_
#define SENSOR_SIMULATOR_HPP_

#include <algorithm>
#include <cmath>
#include <chrono>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_map_server/map_io.hpp>

#include "roadmap_explorer/util/GeometryUtils.hpp"
#include "roadmap_explorer/Parameters.hpp"
#include "roadmap_explorer/util/Logger.hpp"

namespace roadmap_explorer
{

/**
 * @brief Node that projects the robot's field of view onto a persistent
 *        "explored map", re-using the underlying values of the full map.
 */
class SensorSimulator
{
public:
  SensorSimulator(
    std::shared_ptr<nav2_util::LifecycleNode> node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros);
  
  ~SensorSimulator();

  void cleanupMap();

  void startMappingIfNotStarted()
  {
    if (!should_map_)
    {
      std::lock_guard<std::recursive_mutex> lock(map_mutex_);
      should_map_ = true;
      // manually fire a callback to start mapping instantly and don't wait for ros timer to trigger first time.
      timerCallback();
    }
  };

  void stopMapping()
  {
    std::lock_guard<std::recursive_mutex> lock(map_mutex_);
    should_map_ = false;
  };

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
  nav_msgs::msg::OccupancyGrid::SharedPtr explored_map_;


  std::shared_ptr<nav2_util::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
  rclcpp::CallbackGroup::SharedPtr map_subscription_cb_group_;
  bool manual_cleanup_requested_;
  std::atomic<bool> should_map_;
};

}  // namespace roadmap_explorer

#endif  // SENSOR_SIMULATOR_HPP_
