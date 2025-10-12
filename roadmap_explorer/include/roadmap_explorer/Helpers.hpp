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

#ifndef HELPERS_HPP_
#define HELPERS_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav_msgs/msg/path.hpp>

#include "roadmap_explorer/Frontier.hpp"
#include "roadmap_explorer/util/GeometryUtils.hpp"

namespace roadmap_explorer
{


// ===================================  Ray tracing utility class  ===================================

// The output of unknown_cells_ here is the total unknown cells until the end regardless of whether it hit a obstacle or not.

class RayTracedCells
{
public:
  RayTracedCells(
    nav2_costmap_2d::Costmap2D * costmap,
    std::vector<nav2_costmap_2d::MapLocation> & cells,
    int obstacle_min, int obstacle_max,
    int trace_min, int trace_max)
  : costmap_(costmap), cells_(cells),
    obstacle_min_(obstacle_min), obstacle_max_(obstacle_max),
    trace_min_(trace_min), trace_max_(trace_max)
  {
    hit_obstacle = false;
    unknown_cells_ = 0;
    all_cells_count_ = 0;
  }

  inline void operator()(unsigned int offset)
  {
    nav2_costmap_2d::MapLocation loc;
    costmap_->indexToCells(offset, loc.x, loc.y);
    bool presentflag = false;
    for (auto item : cells_) {
      if (item.x == loc.x && item.y == loc.y) {
        presentflag = true;
      }
    }
    if (presentflag == false) {
      ++all_cells_count_;
      auto cost = (int)costmap_->getCost(offset);
      if (cost <= trace_max_ && cost >= trace_min_ && !hit_obstacle) {
        cells_.push_back(loc);
      }
      if (cost >= obstacle_min_ && cost <= obstacle_max_) {
        hit_obstacle = true;
      }
      if (cost == 255) {
        unknown_cells_++;
      }
    }
  }

  std::vector<nav2_costmap_2d::MapLocation> getCells()
  {
    return cells_;
  }

  size_t getCellsSize()
  {
    return all_cells_count_;
  }

  bool hasHitObstacle()
  {
    return hit_obstacle;
  }

  size_t getNumUnknown()
  {
    return unknown_cells_;
  }

private:
  nav2_costmap_2d::Costmap2D * costmap_;
  std::vector<nav2_costmap_2d::MapLocation> & cells_;
  bool hit_obstacle;
  int obstacle_min_, obstacle_max_;
  int trace_min_, trace_max_;
  int unknown_cells_ = 0;
  int all_cells_count_ = 0;
};

inline int sign(int x)
{
  return x > 0 ? 1.0 : -1.0;
}

void bresenham2D(
  RayTracedCells & at, unsigned int abs_da, unsigned int abs_db, int error_b,
  int offset_a,
  int offset_b, unsigned int offset,
  unsigned int max_length,
  int resolution_cut_factor);

bool getTracedCells(
  double start_wx, double start_wy, double end_wx, double end_wy, RayTracedCells & cell_gatherer,
  double max_length,
  nav2_costmap_2d::Costmap2D * exploration_costmap_);

// ===================================  End of Ray tracing related functions  ===================================

bool isCircleFootprintInLethal(
  const nav2_costmap_2d::Costmap2D * costmap, unsigned int center_x,
  unsigned int center_y, double radius_in_cells);

// declare these globally to avoid repeated memory allocation and to optimize nhood functions
extern std::vector<unsigned int> nhood4_values;
extern std::vector<unsigned int> nhood8_values;

std::vector<unsigned int> nhood4(unsigned int idx, const nav2_costmap_2d::Costmap2D & costmap);

std::vector<unsigned int> nhood8(unsigned int idx, const nav2_costmap_2d::Costmap2D & costmap);

std::vector<unsigned int> nhood20(unsigned int idx, const nav2_costmap_2d::Costmap2D & costmap);

bool nearestFreeCell(
  unsigned int & result, unsigned int start, unsigned char val,
  const nav2_costmap_2d::Costmap2D & costmap);

}

#endif // HELPERS_HPP
