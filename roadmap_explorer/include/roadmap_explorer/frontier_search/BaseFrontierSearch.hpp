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

/**
 * @brief Enumeration of possible results from frontier search operations
 */
enum class FrontierSearchResult
{
  ROBOT_OUT_OF_BOUNDS = 0,        ///< Robot position is outside the costmap bounds
  CANNOT_FIND_CELL_TO_SEARCH = 1, ///< No valid cells found to start the search
  SUCCESSFUL_SEARCH = 2           ///< Search completed successfully
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
   * @param costmap Pointer to the costmap for frontier detection.
   */
  virtual void configure(nav2_costmap_2d::Costmap2D * costmap) = 0;

  /**
   * @brief Reset the frontier search state
   * 
   * Clears any internal state or cached data from previous searches.
   */
  virtual void reset() = 0;

  /**
   * @brief Set the frontier search distance to a given value
   * @param value The new search distance value
   * @return true if the distance was set successfully, false if the value exceeds maximum allowed distance
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
   * @param position The starting position for the search in world coordinates
   * @param output_frontier_list Output vector to store found frontiers (will be cleared and populated)
   * @return Result of the search operation indicating success or failure reason
   */
  virtual FrontierSearchResult searchFrom(
    geometry_msgs::msg::Point position,
    std::vector<FrontierPtr> & output_frontier_list) = 0;

  /**
   * @brief Get all frontiers found in the last search operation
   * @return Vector of all frontier points as [x, y] coordinates in world frame
   * @note This method should only be called after a successful searchFrom() operation
   */
  virtual std::vector<std::vector<double>> getAllFrontiers() = 0;

protected:
  double frontier_search_distance_; ///< Maximum distance to search for frontiers from the robot position

  nav2_costmap_2d::Costmap2D * costmap_ = nullptr; ///< Pointer to the costmap used for frontier detection
};

}  // namespace roadmap_explorer

#endif  // FRONTIER_SEARCH_BASE_HPP_
