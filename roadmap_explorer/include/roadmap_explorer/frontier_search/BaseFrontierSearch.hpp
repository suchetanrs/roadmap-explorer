#ifndef FRONTIER_SEARCH_BASE_HPP_
#define FRONTIER_SEARCH_BASE_HPP_

#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "roadmap_explorer/Frontier.hpp"

namespace roadmap_explorer
{

enum class FrontierSearchResult
{
  ROBOT_OUT_OF_BOUNDS = 0,
  CANNOT_FIND_CELL_TO_SEARCH = 1,
  SUCCESSFUL_SEARCH = 2
};

/**
 * @brief Base class for frontier search algorithms
 * 
 * This abstract base class defines the interface that all frontier search
 * algorithms must implement. It provides a plugin-based architecture for
 * different frontier detection strategies.
 */
class FrontierSearchBase
{
public:
  /**
   * @brief Default constructor
   */
  FrontierSearchBase() = default;

  /**
   * @brief Virtual destructor
   */
  virtual ~FrontierSearchBase() = default;

  /**
   * @brief Configure the frontier search with a costmap
   * @param costmap Pointer to the costmap for frontier detection
   */
  virtual void configure(nav2_costmap_2d::Costmap2D * costmap)
  {
    costmap_ = costmap;
  }

  /**
   * @brief Reset the frontier search state
   * 
   * Clears any internal state or cached data from previous searches.
   */
  virtual void reset() = 0;

  /**
   * @brief Increment the search distance by a given value
   * @param value The amount to increment the search distance
   * @return true if increment was successful, false if maximum distance exceeded
   */
  virtual bool setFrontierSearchDistance(double value) = 0;

  /**
   * @brief Get the current frontier search distance
   * @return Current search distance
   */
  virtual double getFrontierSearchDistance() const
  {
    return frontier_search_distance_;
  }

  /**
   * @brief Search for frontiers from a given position
   * @param position The starting position for the search
   * @param output_frontier_list Output vector to store found frontiers
   * @return Result of the search operation
   */
  virtual FrontierSearchResult searchFrom(
    geometry_msgs::msg::Point position,
    std::vector<FrontierPtr> & output_frontier_list) = 0;

  /**
   * @brief Get all frontiers found in the last search
   * @return Vector of all frontier points as [x, y] coordinates
   */
  virtual std::vector<std::vector<double>> getAllFrontiers() = 0;

protected:
  double frontier_search_distance_;
  nav2_costmap_2d::Costmap2D * costmap_ = nullptr;
};

}  // namespace roadmap_explorer

#endif  // FRONTIER_SEARCH_BASE_HPP_
