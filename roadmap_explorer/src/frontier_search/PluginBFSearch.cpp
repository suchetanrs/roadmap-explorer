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

#include <roadmap_explorer/frontier_search/PluginBFSearch.hpp>

namespace roadmap_explorer
{

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

FrontierBFSearch::FrontierBFSearch()
{
  costmap_ = nullptr;
  LOG_INFO("FrontierBFSearch::FrontierBFSearch");
}

FrontierBFSearch::~FrontierBFSearch()
{
  LOG_INFO("FrontierBFSearch::~FrontierBFSearch()");
}

void FrontierBFSearch::configure(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, std::string name, std::shared_ptr<nav2_util::LifecycleNode> node)
{
  if (explore_costmap_ros == nullptr) {
    throw std::runtime_error("Given input costmap is null");
  }
  costmap_ = explore_costmap_ros->getCostmap();

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".min_frontier_cluster_size", rclcpp::ParameterValue(
      1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_frontier_cluster_size", rclcpp::ParameterValue(
      20.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".lethal_threshold", rclcpp::ParameterValue(
      160));

  min_frontier_cluster_size_ = node->get_parameter(
    name + ".min_frontier_cluster_size").as_double();
  max_frontier_cluster_size_ = node->get_parameter(
    name + ".max_frontier_cluster_size").as_double();
  lethal_threshold_ = node->get_parameter(
    name + ".lethal_threshold").as_int();

  LOG_DEBUG("min_frontier_cluster_size: " << min_frontier_cluster_size_);
  LOG_DEBUG("max_frontier_cluster_size: " << max_frontier_cluster_size_);
  LOG_DEBUG("lethal_threshold: " << static_cast<int>(lethal_threshold_));
}

void FrontierBFSearch::reset()
{
  every_frontier_list_.clear();
}

FrontierSearchResult FrontierBFSearch::searchFrom(
  geometry_msgs::msg::Point position,
  std::vector<FrontierPtr> & output_frontier_list,
  double max_frontier_search_distance)
{
  // Clear the accumulated frontier list from previous searches
  every_frontier_list_.clear();

  //  frontier_list to store the detected frontiers.
  std::vector<FrontierPtr> frontier_list;

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    LOG_CRITICAL("Robot out of costmap bounds, cannot search for frontiers");
    return FrontierSearchResult::ROBOT_OUT_OF_BOUNDS;
  }

  map_ = costmap_->getCharMap();
  auto size_x_ = costmap_->getSizeInCellsX();
  auto size_y_ = costmap_->getSizeInCellsY();

  // initialize flag arrays to keep track of visited and frontier cells
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);

  // initialize breadth first search queue
  std::queue<unsigned int> bfs;

  // find closest clear cell to start search
  unsigned int clear, pos = costmap_->getIndex(mx, my);
  if (nearestFreeCell(clear, pos, lethal_threshold_, *costmap_)) {
    bfs.push(clear);
  } else {
    bfs.push(pos);
    LOG_WARN(
      "Could not find nearby clear cell to start search, pushing current position of robot to start search");
    return FrontierSearchResult::CANNOT_FIND_CELL_TO_SEARCH;
  }
  visited_flag[bfs.front()] = true;

  auto distance_to_check_ = max_frontier_search_distance +
    (max_frontier_cluster_size_ * costmap_->getResolution() * DIAGONAL_FACTOR);
  distance_to_check_ = std::pow(distance_to_check_, 2);

  while (rclcpp::ok() && !bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // iterate over 4-connected neighbourhood
    for (unsigned & nbr : nhood4(idx, *costmap_)) {
      // add to queue all free, unvisited cells, use descending search in case initialized on non-free cell
      if (map_[nbr] < nav2_costmap_2d::LETHAL_OBSTACLE && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        unsigned int nbr_mx, nbr_my;
        double nbr_wx, nbr_wy;
        costmap_->indexToCells(nbr, nbr_mx, nbr_my);
        costmap_->mapToWorld(nbr_mx, nbr_my, nbr_wx, nbr_wy);
        if (distanceBetweenPointsSq(
            position, nbr_wx,
            nbr_wy) < distance_to_check_)
        {
          bfs.push(nbr);
        }
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free neighbour)
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        std::vector<FrontierPtr> new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
        for (auto & curr_frontier : new_frontier) {
          if (curr_frontier->getSize() > min_frontier_cluster_size_) {
            frontier_list.push_back(curr_frontier);
            LOG_TRACE("PUSHING NEW FRONTIER TO LIST: UID: " << curr_frontier->getUID());
            LOG_TRACE("Size: " << curr_frontier->getSize());
            LOG_TRACE(
              "Goal Point: (" << curr_frontier->getGoalPoint().x << ", " << curr_frontier->getGoalPoint().y <<
                ")");
          }
        }
      }
    }
  }
  output_frontier_list = frontier_list;

  // Check for duplicates
  if (findDuplicates(frontier_list).size() > 0) {
    throw RoadmapExplorerException("Duplicate frontiers found.");
  }
  if (frontier_list.size() == 0) {
    LOG_INFO("Search was true but no frontiers found.");
    return FrontierSearchResult::NO_FRONTIERS_FOUND;
  }

  return FrontierSearchResult::SUCCESSFUL_SEARCH;
}

std::vector<FrontierPtr> FrontierBFSearch::buildNewFrontier(
  unsigned int initial_cell,
  unsigned int reference,
  std::vector<bool> & frontier_flag)
{
  int currentFrontierSize = 1;
  std::vector<FrontierPtr> calculated_frontiers;
  std::vector<std::pair<double, double>> frontier_cell_indices;       // used to find the median value in case that is needed to be assigned to frontier.
  // record initial contact point for frontier
  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  double wix, wiy;
  costmap_->mapToWorld(ix, iy, wix, wiy);
  every_frontier_list_.push_back({wix, wiy});
  frontier_cell_indices.push_back(std::make_pair(wix, wiy));

  // push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  // cache reference position in world coords
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);

  while (rclcpp::ok() && !bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // try adding cells in 8-connected neighborhood to frontier
    for (unsigned int & nbr : nhood8(idx, *costmap_)) {
      // check if neighbour is a potential frontier cell
      if (isNewFrontierCell(nbr, frontier_flag)) {
        // mark cell as frontier
        frontier_flag[nbr] = true;
        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        // add to every frontier list
        std::vector<double> coord_val;
        coord_val.push_back(wx);
        coord_val.push_back(wy);

        every_frontier_list_.push_back(coord_val);
        frontier_cell_indices.push_back(std::make_pair(wx, wy));

        // update frontier size
        currentFrontierSize = currentFrontierSize + 1;

        // add to queue for breadth first search
        bfs.push(nbr);

        if (currentFrontierSize > max_frontier_cluster_size_) {
          FrontierPtr output = std::make_shared<Frontier>();
          LOG_DEBUG("*************");
          LOG_DEBUG("Getting centroid");
          auto cluster_centroid =
            getCentroidOfCells(
            frontier_cell_indices,
            (costmap_->getResolution() * DIAGONAL_FACTOR * CENTROID_OFFSET_MULTIPLIER));
          SortByMedianFunctor sortFunctor(cluster_centroid);
          std::sort(frontier_cell_indices.begin(), frontier_cell_indices.end(), sortFunctor);
          auto goal_point =
            frontier_cell_indices[static_cast<int>(frontier_cell_indices.size() / 2)];
          output->setGoalPoint(goal_point.first, goal_point.second);
          output->setSize(currentFrontierSize);
          LOG_DEBUG("Cluster size: " << frontier_cell_indices.size());
          LOG_DEBUG("x, y goal: " << goal_point.first << " , " << goal_point.second);
          LOG_DEBUG("Cluster components: ");
          for (auto i : frontier_cell_indices) {
            LOG_DEBUG_N(
              "x: " << i.first << " y: " << i.second << " atan: " <<
                atan2(i.second - cluster_centroid.second, i.first - cluster_centroid.first) << " ");
          }
          LOG_DEBUG("");
          frontier_cell_indices.clear();
          output->setUID(generateUID(output));
          LOG_TRACE("1PUSHING NEW FRONTIER TO LIST: UID: " << output->getUID());
          LOG_TRACE("1Size: " << output->getSize());
          LOG_TRACE(
            "1Initial Point: (" << output->getGoalPoint().x << ", " << output->getGoalPoint().y << ", " << output->getGoalPoint().z <<
              ")");
          LOG_TRACE("**************");
          calculated_frontiers.push_back(output);
          currentFrontierSize = 0;
        }
      }
    }
  }
  if (currentFrontierSize > min_frontier_cluster_size_) {
    FrontierPtr output = std::make_shared<Frontier>();
    auto cluster_centroid =
      getCentroidOfCells(
      frontier_cell_indices,
      (costmap_->getResolution() * DIAGONAL_FACTOR * CENTROID_OFFSET_MULTIPLIER));
    SortByMedianFunctor sortFunctor(cluster_centroid);
    std::sort(frontier_cell_indices.begin(), frontier_cell_indices.end(), sortFunctor);
    auto goal_point = frontier_cell_indices[static_cast<int>(frontier_cell_indices.size() / 2)];
    output->setGoalPoint(goal_point.first, goal_point.second);
    output->setSize(currentFrontierSize);
    LOG_DEBUG("Cluster size: " << frontier_cell_indices.size());
    LOG_DEBUG("x, y goal: " << goal_point.first << " , " << goal_point.second);
    LOG_DEBUG("Cluster components: ");
    for (auto i : frontier_cell_indices) {
      LOG_DEBUG_N(
        ", x: " << i.first << " y: " << i.second << " atan: " <<
          atan2(i.second - cluster_centroid.second, i.first - cluster_centroid.first));
    }
    LOG_DEBUG("");
    frontier_cell_indices.clear();
    output->setUID(generateUID(output));
    LOG_TRACE("2PUSHING NEW FRONTIER TO LIST: UID: " << output->getUID());
    LOG_TRACE("2Size: " << output->getSize());
    LOG_TRACE(
      "2Initial Point: (" << output->getGoalPoint().x << ", " << output->getGoalPoint().y << ", " << output->getGoalPoint().z <<
        ")");
    LOG_TRACE("2*************====================");
    calculated_frontiers.push_back(output);
  }
  return calculated_frontiers;
}

bool FrontierBFSearch::isNewFrontierCell(unsigned int idx, const std::vector<bool> & frontier_flag)
{
  // check that cell is unknown and not already marked as frontier
  if (!isUnknown(map_[idx]) || frontier_flag[idx]) {
    return false;
  }

  bool has_one_free_neighbour = false;
  bool has_one_lethal_neighbour = false;

  // frontier cells should have at least one cell in 4-connected neighbourhood that is free
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (isFree(map_[nbr])) {
      has_one_free_neighbour = true;
    }
    if (isLethal(map_[nbr])) {
      has_one_lethal_neighbour = true;
    }
  }
  
  // Reject cells adjacent to lethal obstacles, otherwise return true if adjacent to free space
  if (has_one_lethal_neighbour) {
    return false;
  }
  return has_one_free_neighbour;
}

std::vector<std::vector<double>> FrontierBFSearch::getAllFrontiers()
{
  return every_frontier_list_;
}

std::pair<double, double> FrontierBFSearch::getCentroidOfCells(
  std::vector<std::pair<double, double>> & cells,
  double distance_to_offset)
{
  double sumX = 0;
  double sumY = 0;

  for (const auto & point : cells) {
    sumX += point.first;
    sumY += point.second;
  }

  double centerX = static_cast<double>(sumX) / cells.size();
  double centerY = static_cast<double>(sumY) / cells.size();

  bool offset_centroid = false;
  double varX = 0, varY = 0;
  for (const auto & point : cells) {
    if (sqrt(
        pow(
          point.first - centerX,
          2) + pow(point.second - centerY, 2)) < costmap_->getResolution() * 3)
    {
      offset_centroid = true;
    }
    varX += abs(point.first - centerX);
    varY += abs(point.second - centerY);
  }
  LOG_DEBUG("Centroid before: " << centerX << " , " << centerY);
  LOG_DEBUG("VarX: " << varX);
  LOG_DEBUG("VarY: " << varY);

  if (varX > varY && offset_centroid) {
    centerY -= distance_to_offset;
  }
  if (varX < varY && offset_centroid) {
    centerX -= distance_to_offset;
  }

  LOG_DEBUG("Centroid: " << centerX << " , " << centerY);

  return std::make_pair(centerX, centerY);
}

std::vector<FrontierPtr> FrontierBFSearch::findDuplicates(const std::vector<FrontierPtr> & vec)
{
  std::vector<FrontierPtr> duplicates;

  // Iterate through the vector
  for (size_t i = 0; i < vec.size(); ++i) {
    // Compare the current element with all subsequent elements
    for (size_t j = i + 1; j < vec.size(); ++j) {
      if (vec[i] == vec[j]) {
        // If a duplicate is found, add it to the duplicates vector
        duplicates.push_back(vec[i]);
        break;             // Break the inner loop to avoid adding the same duplicate multiple times
      }
    }
  }

  return duplicates;
}

}  // namespace roadmap_explorer

#include <pluginlib/class_list_macros.hpp>
// Register the plugin
PLUGINLIB_EXPORT_CLASS(roadmap_explorer::FrontierBFSearch, roadmap_explorer::FrontierSearchBase)
