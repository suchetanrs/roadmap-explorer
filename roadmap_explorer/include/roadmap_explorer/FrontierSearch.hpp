#ifndef FRONTIER_SEARCH_HPP_
#define FRONTIER_SEARCH_HPP_

#include <list>
#include <vector>
#include <queue>
#include <limits>

#include <rclcpp/rclcpp.hpp>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>

#include <geometry_msgs/msg/point.hpp>

#include "roadmap_explorer/util/Logger.hpp"
#include "roadmap_explorer/util/EventLogger.hpp"
#include "roadmap_explorer/Frontier.hpp"
#include "roadmap_explorer/Helpers.hpp"
#include "roadmap_explorer/util/GeometryUtils.hpp"
#include "roadmap_explorer/Parameters.hpp"

namespace roadmap_explorer
{

/**
     * @brief Thread-safe implementation of a frontier-search task for an input costmap.
     */
class FrontierSearch
{

public:
  /**
       * @brief Constructor for search task
       * @param costmap Reference to costmap data to search.
       */
  FrontierSearch(nav2_costmap_2d::Costmap2D & costmap);

  void reset()
  {
    every_frontier_list.clear();
  }

  void incrementSearchDistance(double value)
  {
    max_frontier_distance_ += value;
  }

  void resetSearchDistance()
  {
    max_frontier_distance_ = original_search_distance_;
  }

  /**
       * @brief Runs search implementation, outward from the start position
       * @param position Initial position to search from
       * @return List of frontiers, if any
       */
  std::vector<FrontierPtr> searchFrom(geometry_msgs::msg::Point position);

  /**
       * @brief Getter function for all frontiers unclustered.
       * @return every_frontier variable value.
       */
  std::vector<std::vector<double>> getAllFrontiers();

protected:
  /**
       * @brief Starting from an initial cell, build a frontier from valid adjacent cells
       * @param initial_cell Index of cell to start frontier building
       * @param reference Reference index to calculate position from
       * @param frontier_flag Flag vector indicating which cells are already marked as frontiers
       * @return
       */
  std::vector<FrontierPtr> buildNewFrontier(
    unsigned int initial_cell, unsigned int reference,
    std::vector<bool> & frontier_flag);

  /**
       * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate for a new frontier.
       * @param idx Index of candidate cell
       * @param frontier_flag Flag vector indicating which cells are already marked as frontiers
       * @return
       */
  bool isNewFrontierCell(unsigned int idx, const std::vector<bool> & frontier_flag);

  std::pair<double, double> getCentroidOfCells(
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
            2) + pow(point.second - centerY, 2)) < costmap_.getResolution() * 3)
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

private:
  inline bool isLethal(unsigned char value)
  {
    return (int)value >= lethal_threshold_ && value != nav2_costmap_2d::NO_INFORMATION;
  }

  inline bool isUnknown(unsigned char value)
  {
    return value == nav2_costmap_2d::NO_INFORMATION;
  }

  inline bool isFree(unsigned char value)
  {
    return (int)value < lethal_threshold_;
  }

  nav2_costmap_2d::Costmap2D & costmap_;
  unsigned char * map_;
  unsigned int size_x_, size_y_;
  std::vector<std::vector<double>> every_frontier_list;
  int min_frontier_cluster_size_;
  int max_frontier_cluster_size_;
  double max_frontier_distance_;
  double original_search_distance_;
  unsigned char lethal_threshold_;
};

// Define a custom functor with an extra argument
class SortByMedianFunctor
{
public:
  SortByMedianFunctor(std::pair<double, double> centroid)
  : centroid(centroid) {}

  bool operator()(const std::pair<double, double> & a, const std::pair<double, double> & b) const
  {
    // auto angle_a = atan2(a.second - centroid.second, a.first - centroid.first); // delta y / delta x
    // if(angle_a < 0) angle_a = angle_a + (2 * M_PI);
    // auto angle_b = atan2(b.second - centroid.second, b.first - centroid.first);
    // if(angle_b < 0) angle_b = angle_b + (2 * M_PI);
    // return angle_a < angle_b;

    auto angle_a = atan2(a.second - centroid.second, a.first - centroid.first);         // delta y / delta x
    if (angle_a < 0) {
      angle_a = angle_a + (2 * M_PI);
    }
    auto angle_b = atan2(b.second - centroid.second, b.first - centroid.first);
    if (angle_b < 0) {
      angle_b = angle_b + (2 * M_PI);
    }
    if (0 <= angle_a && angle_a <= M_PI / 2 && 3 * M_PI / 2 <= angle_b && angle_b <= 2 * M_PI) {
      return false;
    }
    if (0 <= angle_b && angle_b <= M_PI / 2 && 3 * M_PI / 2 <= angle_a && angle_a <= 2 * M_PI) {
      return true;
    }
    return angle_a < angle_b;
  }

private:
  std::pair<double, double> centroid;
};

}
#endif
