// node_graph.cpp

#include "roadmap_explorer/planners/FrontierRoadmap.hpp"
#include "roadmap_explorer/util/GeometryUtils.hpp"

std::unique_ptr<roadmap_explorer::FrontierRoadMap> roadmap_explorer::FrontierRoadMap::
frontierRoadmapPtr = nullptr;
std::mutex roadmap_explorer::FrontierRoadMap::instanceMutex_;

namespace roadmap_explorer
{
FrontierRoadMap::FrontierRoadMap(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
  std::shared_ptr<nav2_util::LifecycleNode> node_ptr)
: costmap_(explore_costmap_ros->getCostmap()),
  explore_costmap_ros_(explore_costmap_ros)
{
  // the below getValue cannot be changed at runtime. Hence they are not called anywhere else.
  max_graph_reconstruction_distance_ = parameterInstance.getValue<double>(
    "frontierRoadmap.max_graph_reconstruction_distance");
  GRID_CELL_SIZE = parameterInstance.getValue<double>("frontierRoadmap.grid_cell_size");
  RADIUS_TO_DECIDE_EDGES = parameterInstance.getValue<double>(
    "frontierRoadmap.radius_to_decide_edges");

  // these can be changed at runtime and hence are called elsewhere in the code.
  MIN_DISTANCE_BETWEEN_TWO_FRONTIER_NODES = parameterInstance.getValue<double>(
    "frontierRoadmap.min_distance_between_two_frontier_nodes");
  MIN_DISTANCE_BETWEEN_ROBOT_POSE_AND_NODE = parameterInstance.getValue<double>(
    "frontierRoadmap.min_distance_between_robot_pose_and_node");
  LOG_INFO("Max frontier distance: " << max_graph_reconstruction_distance_);

  max_connection_length_ = RADIUS_TO_DECIDE_EDGES * 1.5;
  marker_pub_roadmap_ = node_ptr->create_publisher<visualization_msgs::msg::MarkerArray>(
    "frontier_roadmap", 10);
  marker_pub_roadmap_->on_activate();
  marker_pub_plan_ = node_ptr->create_publisher<visualization_msgs::msg::MarkerArray>(
    "frontier_roadmap_plan", 10);
  marker_pub_plan_->on_activate();
  map_data_subscription_ = node_ptr->create_subscription<roadmap_explorer_msgs::msg::MapData>(
    "map_data", 10, std::bind(&FrontierRoadMap::mapDataCallback, this, std::placeholders::_1));
  astar_planner_ = std::make_shared<FrontierRoadmapAStar>();


  // Subscriber to handle clicked points
  roadmap_plan_test_sub_ = node_ptr->create_subscription<geometry_msgs::msg::PointStamped>(
    "/clicked_point", 10,
    std::bind(&FrontierRoadMap::clickedPointCallback, this, std::placeholders::_1));
  // RosVisualizerInstance = std::make_shared<RosVisualizer>(node_, costmap_);
}

FrontierRoadMap::~FrontierRoadMap()
{
  LOG_INFO("FrontierRoadMap::~FrontierRoadMap()");
  roadmap_.clear();
  unconnectable_roadmap_.clear();
  spatial_hash_map_.clear();
  map_data_subscription_.reset();
  marker_pub_roadmap_->on_deactivate();
  marker_pub_plan_->on_deactivate();
  marker_pub_roadmap_.reset();
  marker_pub_plan_.reset();
}

void FrontierRoadMap::mapDataCallback(const roadmap_explorer_msgs::msg::MapData & mapData)
{
  // LOG_TRACE("Locking mapDataCallback");
  std::lock_guard<std::mutex> lock(spatial_hash_map_mutex_);
  // LOG_TRACE("acquired mapDataCallback");
  // LOG_TRACE("Map data recieved.");
  // LOG_TRACE("Processing " << mapData.graph.poses_id.size() << " poses");
  latest_keyframe_poses_.clear();
  spatial_kf_map_.clear();
  for (int i = 0; i < (int)mapData.graph.poses_id.size(); i++) {
    // LOG_TRACE("Processing pose " << i << " with ID " << mapData.graph.poses_id[i]);
    latest_keyframe_poses_[mapData.graph.poses_id[i]] = mapData.graph.poses[i];
    if (spatial_kf_map_.count(
        getGridCell(
          mapData.graph.poses[i].pose.position.x,
          mapData.graph.poses[i].pose.position.y)) == 0)
    {
      spatial_kf_map_[getGridCell(
          mapData.graph.poses[i].pose.position.x,
          mapData.graph.poses[i].pose.position.y)] = {};
    }
    spatial_kf_map_[getGridCell(
        mapData.graph.poses[i].pose.position.x,
        mapData.graph.poses[i].pose.position.y)].push_back(mapData.graph.poses_id[i]);
  }
  // LOG_TRACE("Processing no_kf_parent_queue len: " << no_kf_parent_queue_.size());

  while (rclcpp::ok()) {
    // LOG_TRACE("Processing no_kf_parent_queue len: " << no_kf_parent_queue_.size());
    if (no_kf_parent_queue_.empty()) {
      // LOG_TRACE("no_kf_parent_queue is empty");
      break;
    }
    auto frontier = no_kf_parent_queue_.front();
    no_kf_parent_queue_.pop();

    auto frontier_pose = frontier->getGoalPoint();
    // LOG_TRACE("Processing frontier at position: " << frontier_pose.x << ", " << frontier_pose.y << ", " << frontier_pose.z);
    Eigen::Vector3f frontier_point_w(frontier_pose.x, frontier_pose.y, frontier_pose.z);

    auto frontier_grid_cell = getGridCell(frontier_pose.x, frontier_pose.y);
    std::vector<int> frontier_parent_kfs;
    if (spatial_kf_map_.count(frontier_grid_cell) == 0) {
      auto grid_cell = frontier_grid_cell;
      bool foundClosestNode = false;
      int searchRadiusMultiplier = 1;
      while (!foundClosestNode && rclcpp::ok()) {
        int searchRadius = GRID_CELL_SIZE * searchRadiusMultiplier;
        if (searchRadius > 7) {
          LOG_CRITICAL("Cannot find closest KF node within 7m. This is not ok.")
          // throw RoadmapExplorerException("FrontierPtr parent keyframe not found in spatial hash map");
          break;
        }
        for (int dx = -searchRadius; dx <= searchRadius; ++dx) {
          for (int dy = -searchRadius; dy <= searchRadius; ++dy) {
            auto neighbor_cell = std::make_pair(grid_cell.first + dx, grid_cell.second + dy);
            // LOG_INFO("Neighbour cell at x: " << neighbor_cell.first << " and y: " << neighbor_cell.second);
            if (spatial_kf_map_.count(neighbor_cell) > 0) {
              frontier_parent_kfs = spatial_kf_map_[neighbor_cell];
              foundClosestNode = true;
              break;
            }
          }
          if (foundClosestNode) {
            break;
          }
        }
        ++searchRadiusMultiplier;
      }
    } else {
      frontier_parent_kfs = spatial_kf_map_[frontier_grid_cell];
    }
    // LOG_TRACE("Found " << frontier_parent_kfs.size() << " parent keyframes for frontier");

    for (auto kf_id : frontier_parent_kfs) {
      // LOG_TRACE("Processing parent keyframe with ID " << kf_id);
      if (latest_keyframe_poses_.count(kf_id) == 0) {
        continue;
      }
      auto kf_affine_tf = getTransformFromPose(latest_keyframe_poses_[kf_id].pose);
      auto frontier_point_c = kf_affine_tf.inverse() * frontier_point_w;
      keyframe_mapping_[kf_id].push_back(frontier_point_c);
    }
  }

  // LOG_TRACE("NO KF Parent queue size: " << no_kf_parent_queue_.size());
}

void FrontierRoadMap::optimizeSHM()
{
  MIN_DISTANCE_BETWEEN_TWO_FRONTIER_NODES = parameterInstance.getValue<double>(
    "frontierRoadmap.min_distance_between_two_frontier_nodes");
  std::vector<FrontierPtr> optimized_frontiers;
  // LOG_TRACE("Optimize SHM");
  {
    // LOG_TRACE("Locking optimizeSHM");
    std::lock_guard<std::mutex> lock(spatial_hash_map_mutex_);
    // LOG_TRACE("acquired optimizeSHM");
    spatial_hash_map_.clear();
    for (auto &[key, value] : keyframe_mapping_) {
      if (latest_keyframe_poses_.count(key) == 0) {
        continue;
      }
      auto kf_affine_pose = getTransformFromPose(latest_keyframe_poses_[key].pose);
      for (auto & frontier_point_c : value) {
        auto frontier_point_w = kf_affine_pose * frontier_point_c;
        FrontierPtr frontier_point = std::make_shared<Frontier>();
        frontier_point->setGoalPoint(frontier_point_w.x(), frontier_point_w.y());
        frontier_point->setUID(generateUID(frontier_point));
        optimized_frontiers.push_back(frontier_point);
      }
    }
  }
  populateNodes(optimized_frontiers, true, MIN_DISTANCE_BETWEEN_TWO_FRONTIER_NODES, false);
}

void FrontierRoadMap::clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  clicked_points_.push_back(msg->point);

  // If we have two points, calculate the plan
  if (clicked_points_.size() == 2) {
    auto p1 = clicked_points_[0];
    auto p2 = clicked_points_[1];

    LOG_INFO("Calculating plan from " << p1.x << ", " << p1.y << " to " << p2.x << p2.y);

    getPlan(p1.x, p1.y, true, p2.x, p2.y, true, true);

    // Reset the clicked points for next input
    clicked_points_.clear();
  }
}

std::pair<int, int> FrontierRoadMap::getGridCell(double x, double y)
{
  int cell_x = static_cast<int>(std::floor(x / GRID_CELL_SIZE));
  int cell_y = static_cast<int>(std::floor(y / GRID_CELL_SIZE));
  return std::make_pair(cell_x, cell_y);
}

void FrontierRoadMap::populateNodes(
  const std::vector<FrontierPtr> & frontiers,
  bool populateClosest, double min_distance_between_to_add,
  bool addNewToQueue)
{
  // LOG_TRACE("Locking populateNodes");
  std::lock_guard<std::mutex> lock(spatial_hash_map_mutex_);
  // LOG_TRACE("UnLocking populateNodes");
  for (auto & new_frontier : frontiers) {
    bool isNew = true;
    auto new_point = new_frontier->getGoalPoint();
    auto grid_cell = getGridCell(new_point.x, new_point.y);
    // make a cell in the spatial hash map if it does not exist already
    if (spatial_hash_map_.count(grid_cell) == 0) {
      spatial_hash_map_[grid_cell] = {};
    }
    if (!populateClosest) {
      // LOG_TRACE(new_frontier);
      if (isNew) {
        if (addNewToQueue) {
          no_kf_parent_queue_.push(new_frontier);
        }
        spatial_hash_map_[grid_cell].push_back(new_frontier);
      }
      continue;
    }
    // LOG_INFO("New Point x is: " << new_point.x);
    // LOG_INFO("New Point y is: " << new_point.y);
    // LOG_INFO("Grid cell x is: " << grid_cell.first);
    // LOG_INFO("Grid cell y is: " << grid_cell.second);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        auto neighbor_cell = std::make_pair(grid_cell.first + dx, grid_cell.second + dy);
        // LOG_INFO("Neighbour cell at x: " << neighbor_cell.first << " and y: " << neighbor_cell.second);
        if (spatial_hash_map_.count(neighbor_cell) > 0) {
          for (const auto & existing_frontier : spatial_hash_map_[neighbor_cell]) {
            // LOG_INFO("Checking frontier at x: " << existing_frontier->getGoalPoint().x << " and y: " << existing_frontier->getGoalPoint().y);
            if (distanceBetweenFrontiers(
                new_frontier,
                existing_frontier) < min_distance_between_to_add)
            {
              isNew = false;
              // LOG_INFO("The frontier at x: " << new_frontier->getGoalPoint().x << " and y: "
              //                                                                << new_frontier->getGoalPoint().y << " is very close (spatial hash). Discarding..");
              break;
            }
          }
        }
        if (!isNew) {
          break;
        }
      }
      if (!isNew) {
        break;
      }
    }
    if (isNew) {
      // LOG_TRACE("Adding frontier at x: " << new_frontier->getGoalPoint().x << " and y: " << new_frontier->getGoalPoint().y);
      spatial_hash_map_[grid_cell].push_back(new_frontier);
      if (addNewToQueue) {
        no_kf_parent_queue_.push(new_frontier);
      }
      if (spatial_hash_map_[grid_cell].size() > 20) {
        throw RoadmapExplorerException("The size is too big");
      }
    }
  }
}

void FrontierRoadMap::addNodes(const std::vector<FrontierPtr> & frontiers, bool populateClosest)
{
  MIN_DISTANCE_BETWEEN_TWO_FRONTIER_NODES = parameterInstance.getValue<double>(
    "frontierRoadmap.min_distance_between_two_frontier_nodes");
  EventLoggerInstance.startEvent("addNodes");
  // LOG_DEBUG("Going to add these many frontiers to the spatial hash map:" << frontiers.size());
  EventLoggerInstance.startEvent("populateNodes");
  populateNodes(frontiers, populateClosest, MIN_DISTANCE_BETWEEN_TWO_FRONTIER_NODES, true);
  EventLoggerInstance.endEvent("populateNodes", 2);
  EventLoggerInstance.endEvent("addNodes", 2);
}

void FrontierRoadMap::addRobotPoseAsNode(
  geometry_msgs::msg::Pose & start_pose_w,
  bool populateClosest)
{
  MIN_DISTANCE_BETWEEN_ROBOT_POSE_AND_NODE = parameterInstance.getValue<double>(
    "frontierRoadmap.min_distance_between_robot_pose_and_node");
  LOG_DEBUG("ADDING ROBOT POSE TO ROADMAP ...");
  FrontierPtr start = std::make_shared<Frontier>();
  start->setGoalPoint(start_pose_w.position.x, start_pose_w.position.y);
  start->setUID(generateUID(start));
  std::vector<FrontierPtr> frontier_vec;
  frontier_vec.push_back(start);
  populateNodes(frontier_vec, populateClosest, MIN_DISTANCE_BETWEEN_ROBOT_POSE_AND_NODE, true);
}

void FrontierRoadMap::constructNewEdges(const std::vector<FrontierPtr> & frontiers)
{
  LOG_INFO("Reconstructing new frontier edges");
  // Add each point as a child of the parent if no obstacle is present;
  std::vector<FrontierPtr> closestNodes;
  for (const auto & point : frontiers) {
    closestNodes.clear();
    FrontierPtr closestFrontier;
    getClosestNodeInHashmap(point, closestFrontier);
    // Ensure the closestFrontier is added if not already present
    if (roadmap_.find(closestFrontier) == roadmap_.end()) {
      roadmap_mutex_.lock();
      roadmap_[closestFrontier] = {};
      roadmap_mutex_.unlock();
    }
    getNodesWithinRadius(closestFrontier, closestNodes, RADIUS_TO_DECIDE_EDGES);
    int numChildren = 0;
    for (auto & closestNode : closestNodes) {
      if (closestNode == closestFrontier) {
        continue;
      }
      roadmap_mutex_.lock();
      // initialize the closest node memory to add children.
      if (roadmap_.find(closestNode) == roadmap_.end()) {
        roadmap_[closestNode] = {};
      }
      // skip if edge exists already
      auto & addition1 = roadmap_[closestFrontier];
      auto & addition2 = roadmap_[closestNode];
      if (
        std::find(addition1.begin(), addition1.end(), closestNode) != addition1.end() ||
        std::find(addition2.begin(), addition2.end(), closestFrontier) != addition2.end())
      {
        roadmap_mutex_.unlock();
        continue;
      }

      // skip if edge does not exist but it was already determined to be unconnectable
      auto & addition3 = unconnectable_roadmap_[closestFrontier];
      auto & addition4 = unconnectable_roadmap_[closestNode];
      if (
        std::find(addition3.begin(), addition3.end(), closestNode) != addition3.end() ||
        std::find(addition4.begin(), addition4.end(), closestFrontier) != addition4.end())
      {
        roadmap_mutex_.unlock();
        continue;
      }

      // an edge does not exist and is not connectable
      if (isConnectable(closestNode, closestFrontier)) {
        roadmap_[closestFrontier].push_back(closestNode);
        roadmap_[closestNode].push_back(closestFrontier);
        numChildren++;
      } else {
        if (unconnectable_roadmap_.find(closestNode) == unconnectable_roadmap_.end()) {
          unconnectable_roadmap_[closestNode] = {};
        }
        if (unconnectable_roadmap_.find(closestFrontier) == unconnectable_roadmap_.end()) {
          unconnectable_roadmap_[closestFrontier] = {};
        }
        unconnectable_roadmap_[closestFrontier].push_back(closestNode);
        unconnectable_roadmap_[closestNode].push_back(closestFrontier);
        // LOG_INFO("Not connectable");
      }
      roadmap_mutex_.unlock();
    }
    LOG_TRACE(
      "Clearing roadmap and constructing new edges for: " << closestFrontier << " with " << numChildren <<
        " children");
  }
  // roadmap_mutex_.lock();
  // roadmap_.clear();
  // roadmap_mutex_.unlock();
}

void FrontierRoadMap::constructNewEdgeRobotPose(const geometry_msgs::msg::Pose & rPose)
{
  LOG_INFO("Reconstructing new robot pose edges");
  FrontierPtr rPoseF = std::make_shared<Frontier>();
  rPoseF->setGoalPoint(rPose.position.x, rPose.position.y);
  rPoseF->setUID(generateUID(rPoseF));
  std::vector<FrontierPtr> toAdd;
  toAdd.push_back(rPoseF);
  constructNewEdges(toAdd);
}

void FrontierRoadMap::reConstructGraph(bool entireGraph, bool optimizeRoadmap)
{
  if (optimizeRoadmap) {
    optimizeSHM();
  }
  LOG_INFO("Reconstructing entire graph within radius: " << max_graph_reconstruction_distance_);
  LOG_DEBUG("Reconstruct graph!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  geometry_msgs::msg::PoseStamped robotPose;
  explore_costmap_ros_->getRobotPose(robotPose);
  // Add each point as a child of the parent if no obstacle is present
  // LOG_TRACE("Locking reConstructGraph");
  std::lock_guard<std::mutex> lock(spatial_hash_map_mutex_);
  // LOG_TRACE("UnLocking reConstructGraph");
  if (entireGraph) {
    roadmap_.clear();
    unconnectable_roadmap_.clear();
  }
  for (const auto & pair : spatial_hash_map_) {
    if (!entireGraph) {
      if (sqrt(
          pow(
            robotPose.pose.position.x - pair.first.first,
            2) +
          pow(
            robotPose.pose.position.y - pair.first.second,
            2)) > max_graph_reconstruction_distance_)
      {
        LOG_DEBUG(
          "Skipping x: " << pair.first.first << ", y: " << pair.first.second << " from roadmap");
        continue;
      }
    }
    std::vector<FrontierPtr> closestNodes;
    for (const auto & point : pair.second) {
      closestNodes.clear();
      roadmap_mutex_.lock();
      roadmap_[point] = {};
      roadmap_mutex_.unlock();
      // }
      getNodesWithinRadius(point, closestNodes, RADIUS_TO_DECIDE_EDGES);
      // Ensure the point is added if not already present
      // if (roadmap_.find(point) == roadmap_.end())
      // {

      for (auto & closestNode : closestNodes) {
        if (closestNode == point) {
          continue;
        }
        if (isConnectable(closestNode, point)) {
          roadmap_mutex_.lock();
          auto & addition1 = roadmap_[point];
          if (std::find(addition1.begin(), addition1.end(), closestNode) == addition1.end()) {
            roadmap_[point].push_back(closestNode);
          }
          roadmap_mutex_.unlock();
        } else {
          // LOG_INFO("Not connectable");
        }
      }
    }
  }
  // roadmap_mutex_.lock();
  // roadmap_.clear();
  // roadmap_mutex_.unlock();
}

void FrontierRoadMap::getNodesWithinRadius(
  const FrontierPtr & interestNode,
  std::vector<FrontierPtr> & closestNodeVector,
  const double radius)
{
  // Get the central grid cell of the interest node
  auto interest_point = interestNode->getGoalPoint();
  auto center_cell = getGridCell(interest_point.x, interest_point.y);

  // Calculate the number of grid cells to check in each direction (radius in cells)
  int cell_radius = static_cast<int>(std::ceil(radius / GRID_CELL_SIZE));
  for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
    for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
      auto neighbor_cell = std::make_pair(center_cell.first + dx, center_cell.second + dy);
      // The spatial hash map is not protected with the mutex here because the function itself is protected at the caller line.
      if (spatial_hash_map_.count(neighbor_cell) > 0) {
        for (const auto & existing_frontier : spatial_hash_map_[neighbor_cell]) {
          if (distanceBetweenFrontiers(interestNode, existing_frontier) < radius) {
            closestNodeVector.push_back(existing_frontier);
          }
        }
      }
    }
  }
}

void FrontierRoadMap::getNodesWithinRadius(
  const geometry_msgs::msg::Point & interestPoint,
  std::vector<FrontierPtr> & closestNodeVector,
  const double radius)
{
  // Get the central grid cell of the interest node
  auto center_cell = getGridCell(interestPoint.x, interestPoint.y);

  // Calculate the number of grid cells to check in each direction (radius in cells)
  int cell_radius = static_cast<int>(std::ceil(radius / GRID_CELL_SIZE));
  for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
    for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
      auto neighbor_cell = std::make_pair(center_cell.first + dx, center_cell.second + dy);
      // The spatial hash map is not protected with the mutex here because the function itself is protected at the caller line.
      if (spatial_hash_map_.count(neighbor_cell) > 0) {
        for (const auto & existing_frontier : spatial_hash_map_[neighbor_cell]) {
          if (distanceBetweenPoints(interestPoint, existing_frontier->getGoalPoint()) < radius) {
            closestNodeVector.push_back(existing_frontier);
          }
        }
      }
    }
  }
}

void FrontierRoadMap::getClosestNodeInHashmap(
  const FrontierPtr & interestNode,
  FrontierPtr & closestNode)
{
  // LOG_TRACE("Locking getClosestNodeInHashmap");
  std::lock_guard<std::mutex> lock(spatial_hash_map_mutex_);
  // LOG_TRACE("UnLocking getClosestNodeInHashmap");
  auto grid_cell = getGridCell(interestNode->getGoalPoint().x, interestNode->getGoalPoint().y);
  double min_distance = std::numeric_limits<double>::max();
  bool foundClosestNode = false;
  int searchRadiusMultiplier = 1;
  while (!foundClosestNode && rclcpp::ok()) {
    int searchRadius = GRID_CELL_SIZE * searchRadiusMultiplier;
    if (searchRadius > 10) {
      LOG_CRITICAL("Cannot find closest node within 10m in hashmap. This is not ok.")
      break;
    }
    for (int dx = -searchRadius; dx <= searchRadius; ++dx) {
      for (int dy = -searchRadius; dy <= searchRadius; ++dy) {
        auto neighbor_cell = std::make_pair(grid_cell.first + dx, grid_cell.second + dy);
        // LOG_INFO("Neighbour cell at x: " << neighbor_cell.first << " and y: " << neighbor_cell.second);
        if (spatial_hash_map_.count(neighbor_cell) > 0) {
          for (const auto & existing_frontier : spatial_hash_map_[neighbor_cell]) {
            double distance = distanceBetweenFrontiers(interestNode, existing_frontier);
            if (distance < min_distance) {
              foundClosestNode = true;
              min_distance = distance;
              closestNode = existing_frontier;
            }
          }
        }
      }
    }
    ++searchRadiusMultiplier;
  }
}

void FrontierRoadMap::getClosestNodeInRoadMap(
  const FrontierPtr & interestNode,
  FrontierPtr & closestNode)
{
  // LOG_TRACE("Locking getClosestNodeInRoadMap");
  std::lock_guard<std::mutex> lock(spatial_hash_map_mutex_);
  // LOG_TRACE("UnLocking getClosestNodeInRoadMap");
  auto grid_cell = getGridCell(interestNode->getGoalPoint().x, interestNode->getGoalPoint().y);
  double min_distance = std::numeric_limits<double>::max();
  bool foundClosestNode = false;
  int searchRadiusMultiplier = 1;
  while (!foundClosestNode && rclcpp::ok()) {
    int searchRadius = GRID_CELL_SIZE * searchRadiusMultiplier;
    if (searchRadius > 10) {
      LOG_CRITICAL("Cannot find closest node within 10m in roadmap. This is not ok.")
      break;
    }
    for (int dx = -searchRadius; dx <= searchRadius; ++dx) {
      for (int dy = -searchRadius; dy <= searchRadius; ++dy) {
        auto neighbor_cell = std::make_pair(grid_cell.first + dx, grid_cell.second + dy);
        // LOG_INFO("Neighbour cell at x: " << neighbor_cell.first << " and y: " << neighbor_cell.second);
        if (spatial_hash_map_.count(neighbor_cell) > 0) {
          for (const auto & existing_frontier : spatial_hash_map_[neighbor_cell]) {
            roadmap_mutex_.lock();
            if (roadmap_.count(existing_frontier) > 0) {
              double distance = distanceBetweenFrontiers(interestNode, existing_frontier);
              if (distance < min_distance) {
                foundClosestNode = true;
                min_distance = distance;
                closestNode = existing_frontier;
              }
            }
            roadmap_mutex_.unlock();
          }
        }
      }
    }
    ++searchRadiusMultiplier;
  }
}

RoadmapPlanResult FrontierRoadMap::getPlan(
  double xs, double ys, bool useClosestToStart, double xe,
  double ye, bool useClosestToEnd, bool publish_plan)
{
  RoadmapPlanResult planResult;
  if (xs == xe && ys == ye) {
    planResult.path_exists = true;
    planResult.path_length_m = 0.0;
    planResult.path = std::vector<std::shared_ptr<Node>>();
    return planResult;
  }
  FrontierPtr start = std::make_shared<Frontier>();
  start->setGoalPoint(xs, ys);
  start->setUID(generateUID(start));
  FrontierPtr start_closest = std::make_shared<Frontier>();
  {
    if (!useClosestToStart) {
      if (roadmap_.count(start) == 0) {
        std::vector<FrontierPtr> frontier_vec;
        frontier_vec.push_back(start);
        addNodes(frontier_vec, false);
        start_closest = start;
      } else {
        LOG_INFO("Start node: " << xs << ", " << ys << " already in the roadmap");
      }
    } else {
      getClosestNodeInRoadMap(start, start_closest);
    }
  }

  FrontierPtr goal = std::make_shared<Frontier>();
  goal->setGoalPoint(xe, ye);
  goal->setUID(generateUID(goal));
  FrontierPtr goal_closest = std::make_shared<Frontier>();
  {
    if (!useClosestToEnd) {
      if (roadmap_.count(goal) == 0) {
        std::vector<FrontierPtr> frontier_vec;
        frontier_vec.push_back(goal);
        addNodes(frontier_vec, false);
        goal_closest = goal;
      } else {
        LOG_INFO("End node: " << xe << ", " << ye << " already in the roadmap");
      }
    } else {
      getClosestNodeInRoadMap(goal, goal_closest);
    }
  }

  roadmap_mutex_.lock();
  LOG_DEBUG("Getting path from: " << start << " to " << goal);
  LOG_DEBUG("In turn calculating from:" << start_closest << " to " << goal_closest);
  auto aStarResult = astar_planner_->getPlan(start_closest, goal_closest, roadmap_);
  planResult.path = aStarResult.first;
  planResult.path_length_m = aStarResult.second;
  planResult.path_exists = true;
  if (planResult.path.size() == 0) {
    LOG_WARN("Could not compute path from: " << xs << ", " << ys << " to " << xe << ", " << ye);
    planResult.path_exists = false;
    roadmap_mutex_.unlock();
    return planResult;
  }
  // LOG_INFO("Plan size: %d", plan.size());
  roadmap_mutex_.unlock();
  LOG_TRACE("Path information:");
  for (auto & fullPoint : planResult.path) {
    LOG_TRACE(fullPoint->frontier << " , ");
  }
  LOG_TRACE("Path length in m: " << planResult.path_length_m);
  if (publish_plan) {
    publishPlan(planResult.path, 1.0, 0.0, 0.0);
  }
  return planResult;
}

RoadmapPlanResult FrontierRoadMap::getPlan(
  FrontierPtr & startNode, bool useClosestToStart,
  FrontierPtr & endNode, bool useClosestToEnd)
{
  return getPlan(
    startNode->getGoalPoint().x,
    startNode->getGoalPoint().y, useClosestToStart,
    endNode->getGoalPoint().x, endNode->getGoalPoint().y, useClosestToEnd, false);
}

bool FrontierRoadMap::isConnectable(const FrontierPtr & f1, const FrontierPtr & f2)
{
  std::vector<nav2_costmap_2d::MapLocation> traced_cells;
  RayTracedCells cell_gatherer(costmap_, traced_cells, 253, 254, 0, 255);
  double resolution = costmap_->getResolution();
  if (resolution <= 0.0) {
    throw RoadmapExplorerException("Costmap resolution is 0 or negative.");
  }
  unsigned int max_length = static_cast<unsigned int>(max_connection_length_ / resolution);
  if (!getTracedCells(
      f1->getGoalPoint().x, f1->getGoalPoint().y, f2->getGoalPoint().x,
      f2->getGoalPoint().y, cell_gatherer, max_length, costmap_))
  {
    return false;
  }
  if (cell_gatherer.hasHitObstacle()) {
    return false;
  }
  if (resolution > 0.0 &&
    cell_gatherer.getNumUnknown() > RADIUS_TO_DECIDE_EDGES / resolution * 0.3)
  {
    return false;
  }
  rosVisualizerInstance.visualizeMarkers("connecting_cells", cell_gatherer.getCells());

  return true;
}

std::size_t FrontierRoadMap::countTotalItemsInSpatialMap()
{
  std::lock_guard<std::mutex> lock(spatial_hash_map_mutex_);
  std::size_t total_items = 0;
  std::vector<FrontierPtr> master_frontier_list;
  for (const auto & cell : spatial_hash_map_) {
    master_frontier_list.insert(
      master_frontier_list.end(), cell.second.begin(),
      cell.second.end());
    total_items += cell.second.size();           // Add the size of each grid's list to the total count
  }
  rosVisualizerInstance.visualizePointCloud(
    "spatial_hashmap_points", master_frontier_list, "map",
    50.0f);
  // LOG_INFO("Total items in the spatial map is: " << total_items);
  return total_items;
}

void FrontierRoadMap::publishRoadMap()
{
  if (marker_pub_roadmap_->get_subscription_count() == 0) {
    return;
  }
  visualization_msgs::msg::MarkerArray marker_array;

  // Marker for nodes
  visualization_msgs::msg::Marker node_marker;
  node_marker.header.frame_id = "map";
  node_marker.header.stamp = rclcpp::Clock().now();
  node_marker.ns = "nodes";
  node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  node_marker.action = visualization_msgs::msg::Marker::ADD;
  node_marker.pose.orientation.w = 1.0;
  node_marker.scale.x = 0.2;       // Size of each sphere
  node_marker.scale.y = 0.2;
  node_marker.scale.z = 0.2;
  node_marker.color.a = 0.02;       // 1.0: Fully opaque
  node_marker.color.r = 0.0;
  node_marker.color.g = 1.0;
  node_marker.color.b = 0.0;

  // Marker for edges
  visualization_msgs::msg::Marker edge_marker;
  edge_marker.header.frame_id = "map";
  edge_marker.header.stamp = rclcpp::Clock().now();
  edge_marker.ns = "edges";
  edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  edge_marker.action = visualization_msgs::msg::Marker::ADD;
  edge_marker.pose.orientation.w = 1.0;
  edge_marker.scale.x = 0.02;       // Thickness of the lines
  edge_marker.color.a = 0.02;       // 1.0 - Fully opaque
  edge_marker.color.r = 0.3;        // Red channel
  edge_marker.color.g = 0.7;        // Green channel
  edge_marker.color.b = 1.0;        // Blue channel

  LOG_TRACE("Roadmap points: ")
  int num_roadmap_points = 0;
  int max_roadmap_children = 0;
  // Populate the markers
  for (const auto & pair : roadmap_) {
    ++num_roadmap_points;
    LOG_TRACE("parent:")
    FrontierPtr parent = pair.first;
    LOG_TRACE(parent);
    geometry_msgs::msg::Point parent_point = parent->getGoalPoint();
    parent_point.z = 0.02;
    node_marker.points.push_back(parent_point);
    int roadmap_children = 0;

    for (const auto & child : pair.second) {
      ++roadmap_children;
      LOG_TRACE("Children:")
      LOG_TRACE(child);
      geometry_msgs::msg::Point child_point = child->getGoalPoint();
      child_point.z = 0.02;
      node_marker.points.push_back(child_point);
      child_point.z = 0.02;

      edge_marker.points.push_back(parent_point);
      edge_marker.points.push_back(child_point);
    }
    max_roadmap_children = std::max(roadmap_children, max_roadmap_children);
    LOG_TRACE("=======")
  }
  LOG_TRACE("***********Roadmap points end: ")

  // Add the markers to the marker array
  marker_array.markers.push_back(node_marker);
  marker_array.markers.push_back(edge_marker);
  LOG_INFO("Number of roadmap nodes: " << num_roadmap_points);
  LOG_INFO("Max number of children for a node: " << max_roadmap_children);

  // Publish the markers
  marker_pub_roadmap_->publish(marker_array);
}

void FrontierRoadMap::publishPlan(
  const std::vector<std::shared_ptr<Node>> & plan, float r,
  float g, float b) const
{
  if (plan.size() == 0) {
    return;
  }
  // Create a MarkerArray to hold the markers for the plan
  visualization_msgs::msg::MarkerArray marker_array;

  // Marker for nodes
  visualization_msgs::msg::Marker node_marker;
  node_marker.header.frame_id = "map";
  node_marker.header.stamp = rclcpp::Clock().now();
  node_marker.ns = "plan_nodes";
  node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  node_marker.action = visualization_msgs::msg::Marker::ADD;
  node_marker.pose.orientation.w = 1.0;
  node_marker.scale.x = 0.2;       // Size of each sphere
  node_marker.scale.y = 0.2;
  node_marker.scale.z = 0.2;
  node_marker.color.a = 1.0;       // Fully opaque
  node_marker.color.r = r;
  node_marker.color.g = g;
  node_marker.color.b = b;       // Blue color for plan nodes

  // Marker for edges
  visualization_msgs::msg::Marker edge_marker;
  edge_marker.header.frame_id = "map";
  edge_marker.header.stamp = rclcpp::Clock().now();
  edge_marker.ns = "plan_edges";
  edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  edge_marker.action = visualization_msgs::msg::Marker::ADD;
  edge_marker.pose.orientation.w = 1.0;
  edge_marker.scale.x = 0.15;       // Thickness of the lines
  edge_marker.color.a = 1.0;        // Fully opaque
  edge_marker.color.r = r;
  edge_marker.color.g = g;
  edge_marker.color.b = b;

  // Populate the markers
  for (const auto & node : plan) {
    geometry_msgs::msg::Point point = node->frontier->getGoalPoint();
    node_marker.points.push_back(point);
  }

  for (size_t i = 0; i < plan.size() - 1; ++i) {
    // LOG_TRACE("New start point in plan: " << plan[i]->frontier->getGoalPoint().x << ", " << plan[i]->frontier->getGoalPoint().y);
    // LOG_TRACE("New end point in plan: " << plan[i + 1]->frontier->getGoalPoint().x << ", " << plan[i + 1]->frontier->getGoalPoint().y);
    geometry_msgs::msg::Point start_point = plan[i]->frontier->getGoalPoint();
    geometry_msgs::msg::Point end_point = plan[i + 1]->frontier->getGoalPoint();

    edge_marker.points.push_back(start_point);
    edge_marker.points.push_back(end_point);
  }

  // Add the markers to the marker array
  marker_array.markers.push_back(node_marker);
  marker_array.markers.push_back(edge_marker);

  // Publish the markers
  marker_pub_plan_->publish(marker_array);
}
}
