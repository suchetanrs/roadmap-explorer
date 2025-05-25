#ifndef HELPERS_HPP_
#define HELPERS_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_navfn_planner/navfn.hpp>

#include "roadmap_explorer/Frontier.hpp"
#include "roadmap_explorer/util/GeometryUtils.hpp"
#include "roadmap_explorer/planners/theta_star.hpp"

namespace roadmap_explorer
{

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
  int resolution_cut_factor,
  nav2_costmap_2d::Costmap2D * exploration_costmap_);

bool getTracedCells(
  double start_wx, double start_wy, double end_wx, double end_wy, RayTracedCells & cell_gatherer,
  double max_length,
  nav2_costmap_2d::Costmap2D * exploration_costmap_);

// TODO(suchetan): A problematic function that needs fixing. Provide options to choose radius / nhood cell size. Also provide options to choose the lethal cell count threshold.
bool surroundingCellsMapped(
  geometry_msgs::msg::Point & checkPoint,
  nav2_costmap_2d::Costmap2D & exploration_costmap_);

bool isCircleFootprintInLethal(
  const nav2_costmap_2d::Costmap2D * costmap, unsigned int center_x,
  unsigned int center_y, double radius_in_cells);

extern std::vector<unsigned int> nhood4_values;
extern std::vector<unsigned int> nhood8_values;

std::vector<unsigned int> nhood4(unsigned int idx, const nav2_costmap_2d::Costmap2D & costmap);

std::vector<unsigned int> nhood8(unsigned int idx, const nav2_costmap_2d::Costmap2D & costmap);

std::vector<unsigned int> nhood20(unsigned int idx, const nav2_costmap_2d::Costmap2D & costmap);

bool nearestFreeCell(
  unsigned int & result, unsigned int start, unsigned char val,
  const nav2_costmap_2d::Costmap2D & costmap);

Eigen::Affine3f getTransformFromPose(geometry_msgs::msg::Pose & pose);

bool computePathBetweenPoints(
  nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Point & start_point,
  const geometry_msgs::msg::Point & goal_point,
  bool planner_allow_unknown,
  nav2_costmap_2d::Costmap2D * exploration_costmap_);

bool computePathBetweenPointsThetaStar(
  nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Point & start_point,
  const geometry_msgs::msg::Point & goal_point,
  bool planner_allow_unknown,
  nav2_costmap_2d::Costmap2D * exploration_costmap_);
}

#endif // HELPERS_HPP
