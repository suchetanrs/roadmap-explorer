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

#include "roadmap_explorer/frontier_search/BaseFrontierSearch.hpp"

namespace roadmap_explorer
{

class FrontierBFSearch : public FrontierSearchBase
{

public:
  FrontierBFSearch();

  ~FrontierBFSearch() override;

  void configure(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros, std::string name, std::shared_ptr<nav2_util::LifecycleNode> node) override;

  void reset() override;

  FrontierSearchResult searchFrom(
    geometry_msgs::msg::Point position,
    std::vector<FrontierPtr> & output_frontier_list,
    double max_frontier_search_distance) override;

  std::vector<std::vector<double>> getAllFrontiers() override;

protected:
  std::vector<FrontierPtr> buildNewFrontier(
    unsigned int initial_cell, unsigned int reference,
    std::vector<bool> & frontier_flag);

  bool isNewFrontierCell(unsigned int idx, const std::vector<bool> & frontier_flag);

  // TODO(suchetan): Is there a better way to do this? Using a graph "medoid" or geodesic (path) median?
  std::pair<double, double> getCentroidOfCells(
    std::vector<std::pair<double, double>> & cells,
    double distance_to_offset);

private:
  inline bool isLethal(unsigned char value) const
  {
    return (int)value >= lethal_threshold_ && value != nav2_costmap_2d::NO_INFORMATION;
  }

  inline bool isUnknown(unsigned char value) const
  {
    return value == nav2_costmap_2d::NO_INFORMATION;
  }

  inline bool isFree(unsigned char value) const
  {
    return (int)value < lethal_threshold_;
  }

  std::vector<FrontierPtr> findDuplicates(const std::vector<FrontierPtr> & vec);

  // Constants for geometric calculations
  static constexpr double DIAGONAL_FACTOR = 1.414;  // sqrt(2) approximation
  static constexpr double CENTROID_OFFSET_MULTIPLIER = 2.0;
  static constexpr int CENTROID_RESOLUTION_FACTOR = 3;

  unsigned char * map_;
  std::vector<std::vector<double>> every_frontier_list_;
  int min_frontier_cluster_size_;
  int max_frontier_cluster_size_;
  unsigned char lethal_threshold_;

  nav2_costmap_2d::Costmap2D * costmap_ = nullptr; ///< Pointer to the costmap used for frontier detection
};

// Define a custom functor with an extra argument
class SortByMedianFunctor
{
public:
  SortByMedianFunctor(std::pair<double, double> centroid)
  : centroid(centroid) {}

  bool operator()(const std::pair<double, double> & a, const std::pair<double, double> & b) const
  {
    auto angle_a = atan2(a.second - centroid.second, a.first - centroid.first);         // delta y / delta x
    if (angle_a < 0) {
      angle_a = angle_a + (2 * M_PI);
    }
    auto angle_b = atan2(b.second - centroid.second, b.first - centroid.first);
    if (angle_b < 0) {
      angle_b = angle_b + (2 * M_PI);
    }
    // Handle angle wraparound at 0/2π boundary
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
