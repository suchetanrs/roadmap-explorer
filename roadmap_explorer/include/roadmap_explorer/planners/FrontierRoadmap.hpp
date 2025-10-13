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
#ifndef FRONTIER_ROADMAP_HPP_
#define FRONTIER_ROADMAP_HPP_

#include <vector>
#include <unordered_map>
#include <map>
#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <mutex>
#include <queue>
#include <fstream>
#include <filesystem>

#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include "roadmap_explorer_msgs/msg/map_data.hpp"

#include "roadmap_explorer/Parameters.hpp"
#include "roadmap_explorer/Frontier.hpp"
#include "roadmap_explorer/Helpers.hpp"

#include "roadmap_explorer/planners/Astar.hpp"

#include "roadmap_explorer/util/EventLogger.hpp"
#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/util/RosVisualizer.hpp"

namespace roadmap_explorer
{

struct RoadmapPlanResult
{
  std::vector<std::shared_ptr<Node>> path;
  double path_length_m;
  bool path_exists;
};

class FrontierRoadMap
{
public:
  ~FrontierRoadMap();

  static void createInstance(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
    std::shared_ptr<nav2_util::LifecycleNode> node_ptr)
  {
    LOG_INFO("Creating roadmap instance");
    std::lock_guard<std::mutex> lock(instanceMutex_);
    if (frontierRoadmapPtr == nullptr) {
      frontierRoadmapPtr.reset(new FrontierRoadMap(explore_costmap_ros, node_ptr));
    } else {
      throw RoadmapExplorerException("FrontierRoadMap instance already exists!");
    }
  }

  static FrontierRoadMap & getInstance()
  {
    std::lock_guard<std::mutex> lock(instanceMutex_);
    if (frontierRoadmapPtr == nullptr) {
      throw RoadmapExplorerException("Cannot de-reference a null FrontierRoadMap! :(");
    }
    return *frontierRoadmapPtr;
  }

  static void destroyInstance()
  {
    LOG_INFO("FrontierRoadMap::destroyInstance()");
    frontierRoadmapPtr.reset();
  }

  void addNodes(const std::vector<FrontierPtr> & frontiers, bool populateClosest);

  void addRobotPoseAsNode(geometry_msgs::msg::Pose & start_pose_w, bool populateClosest);

  void constructNewEdges(const std::vector<FrontierPtr> & frontiers);

  void constructNewEdgeRobotPose(const geometry_msgs::msg::Pose & rPose);

  void publishRoadMap();

  std::size_t countTotalItemsInSpatialMap();

  void reConstructGraph(bool entireGraph, bool optimizeRoadmap);

  RoadmapPlanResult getPlan(
    double xs, double ys, bool useClosestToStart, double xe, double ye,
    bool useClosestToEnd, bool publish_plan);

  RoadmapPlanResult getPlan(
    FrontierPtr & startNode, bool useClosestToStart, FrontierPtr & endNode,
    bool useClosestToEnd);

  void publishPlan(
    const std::vector<std::shared_ptr<Node>> & plan, float r, float g,
    float b) const;

  void saveRoadmapData(const std::string & base_path, const std::string & session_name);

  void loadRoadmapData(const std::string & base_path, const std::string & session_name);

private:
  void mapDataCallback(const roadmap_explorer_msgs::msg::MapData & mapData);

  void optimizeSHM();

  void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

  std::pair<int, int> getGridCell(double x, double y);

  void populateNodes(
    const std::vector<FrontierPtr> & frontiers, bool populateClosest,
    double min_distance_between_to_add, bool addNewToQueue);

  void getNodesWithinRadius(
    const FrontierPtr & interestNode,
    std::vector<FrontierPtr> & closestNodeVector, const double radius);

  void getNodesWithinRadius(
    const geometry_msgs::msg::Point & interestPoint,
    std::vector<FrontierPtr> & closestNodeVector, const double radius);

  void getClosestNodeInHashmap(const FrontierPtr & interestNode, FrontierPtr & closestNode);

  void getClosestNodeInRoadMap(const FrontierPtr & interestNode, FrontierPtr & closestNode);

  std::mutex & getRoadmapMutex()
  {
    return roadmap_mutex_;
  }

  std::unordered_map<FrontierPtr, std::vector<FrontierPtr>, FrontierHash> & getRoadMap()
  {
    return roadmap_;
  }

  bool isConnectable(const FrontierPtr & f1, const FrontierPtr & f2);

  // Custom hash function for std::pair<int, int>
  struct spatialHash
  {
    template<class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> & pair) const
    {
      auto hash1 = std::hash<T1>{}(pair.first);
      auto hash2 = std::hash<T2>{}(pair.second);
      return hash1 ^ (hash2 << 1);           // Combine the two hashes
    }
  };

  FrontierRoadMap(const FrontierRoadMap &) = delete;
  FrontierRoadMap & operator=(const FrontierRoadMap &) = delete;
  FrontierRoadMap(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
    std::shared_ptr<nav2_util::LifecycleNode> node_ptr);

  static std::unique_ptr<FrontierRoadMap> frontierRoadmapPtr;
  static std::mutex instanceMutex_;

  nav2_costmap_2d::Costmap2D * costmap_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;

  std::mutex spatial_hash_map_mutex_;
  std::unordered_map<std::pair<int, int>, std::vector<FrontierPtr>, spatialHash> spatial_hash_map_;

  std::queue<FrontierPtr> no_kf_parent_queue_;
  std::unordered_map<int, geometry_msgs::msg::PoseStamped> latest_keyframe_poses_;
  std::unordered_map<std::pair<int, int>, std::vector<int>, spatialHash> spatial_kf_map_;
  std::unordered_map<int, std::vector<Eigen::Vector3f>> keyframe_mapping_;

  std::unordered_map<FrontierPtr, std::vector<FrontierPtr>, FrontierHash> roadmap_;
  std::unordered_map<FrontierPtr, std::vector<FrontierPtr>, FrontierHash> unconnectable_roadmap_;
  std::mutex roadmap_mutex_;
  double max_connection_length_;
  double max_graph_reconstruction_distance_;
  std::shared_ptr<nav2_util::LifecycleNode> node_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    marker_pub_roadmap_;
  rclcpp::Subscription<roadmap_explorer_msgs::msg::MapData>::SharedPtr map_data_subscription_;
  std::shared_ptr<FrontierRoadmapAStar> astar_planner_;

  // for testing planning
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    marker_pub_plan_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr roadmap_plan_test_sub_;
  std::vector<geometry_msgs::msg::Point> clicked_points_;

  double GRID_CELL_SIZE;
  double RADIUS_TO_DECIDE_EDGES;
  double MIN_DISTANCE_BETWEEN_TWO_FRONTIER_NODES;
  double MIN_DISTANCE_BETWEEN_ROBOT_POSE_AND_NODE;
};

#define frontierRoadmapInstance (FrontierRoadMap::getInstance())
} // namespace roadmap_explorer

#endif // FRONTIER_ROADMAP_HPP_